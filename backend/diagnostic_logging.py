"""ROS-compatible platform logging, with bounded file retention."""
import contextvars
import json
import ipaddress
import logging
import re
import sys
import time
import uuid
from contextlib import contextmanager
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler
from pathlib import Path
from urllib.parse import urlsplit


_CONTEXT = contextvars.ContextVar("diagnostic_context", default={})
_SECRET = re.compile(r"(?i)((?:password|token|secret|authorization|cookie|api[_-]?key)[\"']?\s*[=:]\s*[\"']?)(?:Bearer\s+)?[^\s,;\"'}]+")
_BEARER = re.compile(r"(?i)(bearer\s+)[^\s,;\"'}]+")


def safe_text(value, limit=600):
    """Bound untrusted fields, escape newlines at serialization, redact credentials."""
    text = _SECRET.sub(lambda m: m.group(1) + "[REDACTED]", str(value))
    return _BEARER.sub(lambda m: m.group(1) + "[REDACTED]", text)[:limit]


def current_context():
    return dict(_CONTEXT.get())


def trace_headers():
    """Correlation only; these headers never carry authentication identity."""
    fields = _CONTEXT.get()
    return {header: fields[key] for key, header, pattern in (
        ("request_id", "X-Diagnostic-Parent-Request", r"[a-f0-9]{32}"),
        ("job_id", "X-Diagnostic-Job", r"[a-f0-9]{32}"),
        ("session_id", "X-Diagnostic-Session", r"[A-Za-z0-9_-]{8,96}"))
        if isinstance(fields.get(key), str) and re.fullmatch(pattern, fields[key])}


def update_context(**fields):
    _CONTEXT.set({**_CONTEXT.get(), **fields})


@contextmanager
def context(**fields):
    token = _CONTEXT.set({**_CONTEXT.get(), **fields})
    try:
        yield
    finally:
        _CONTEXT.reset(token)


def fields_text(fields):
    return " ".join(f"{key}={json.dumps(safe_text(value) if isinstance(value, str) else value, ensure_ascii=False, separators=(',', ':'))}"
                    for key, value in fields.items() if value is not None)


def event(logger, level, name, *, exc_info=False, **fields):
    logger.log(level, fields_text({"event": name, **_CONTEXT.get(), **fields}), exc_info=exc_info)


class RequestLoggingMixin:
    """Trace failures and operations, while keeping successful polling quiet."""

    def parse_request(self):
        parsed = super().parse_request()
        if parsed and ipaddress.ip_address(self.client_address[0]).is_loopback:
            fields = {}
            for key, header, pattern in (
                ("parent_request_id", "X-Diagnostic-Parent-Request", r"[a-f0-9]{32}"),
                ("job_id", "X-Diagnostic-Job", r"[a-f0-9]{32}"),
                ("session_id", "X-Diagnostic-Session", r"[A-Za-z0-9_-]{8,96}")):
                value = self.headers.get(header, "")
                if re.fullmatch(pattern, value):
                    fields[key] = value
            update_context(**fields)
        return parsed

    def handle_one_request(self):
        started = time.monotonic()
        self._diagnostic_status = None
        self._diagnostic_error = None
        self._diagnostic_trace = None
        logger = logging.getLogger("opendelivery.http")
        with context(request_id=uuid.uuid4().hex):
            try:
                super().handle_one_request()
            except (BrokenPipeError, ConnectionResetError):
                self.close_connection = True
                event(logger, logging.DEBUG, "http.client_disconnected")
            except Exception:
                self._diagnostic_status = 500
                self._diagnostic_error = "unhandled request exception"
                self._diagnostic_trace = sys.exc_info()
                raise
            finally:
                status = self._diagnostic_status
                if status is not None:
                    elapsed = round((time.monotonic() - started) * 1000)
                    method = getattr(self, "command", "unknown")
                    try:
                        path = urlsplit(getattr(self, "path", "")).path
                    except ValueError:
                        path = "invalid path"
                    operation = method not in ("GET", "HEAD", "OPTIONS") and path != "/api/robot/motion/teleop"
                    history = path.startswith("/api/assistant/sessions")
                    slow = elapsed >= 2000 and not path.endswith("/stream") and path != "/api/assistant/chat"
                    level = logging.ERROR if status >= 500 else logging.WARNING if status >= 400 or slow or self._diagnostic_error else logging.INFO if operation or history else logging.DEBUG
                    event(logger, level, "http.request_completed", method=method, path=path,
                          status=status, duration_ms=elapsed, peer=self.client_address[0],
                          error=self._diagnostic_error, exc_info=self._diagnostic_trace)

    def send_response(self, code, message=None):
        self._diagnostic_status = code
        if code >= 400 and message and not self._diagnostic_error:
            self._diagnostic_error = safe_text(message)
        super().send_response(code, message)
        # Always generate the correlation ID locally; never trust a client value.
        self.send_header("X-Request-ID", _CONTEXT.get().get("request_id", ""))

    def record_response_error(self, payload, status):
        if isinstance(payload, dict) and (status >= 400 or payload.get("ok") is False or payload.get("success") is False):
            self._diagnostic_error = safe_text(payload.get("error") or payload.get("message") or "request failed")
            if status >= 500 and sys.exc_info()[0] is not None:
                self._diagnostic_trace = sys.exc_info()


class RosFormatter(logging.Formatter):
    def format(self, record):
        level = {"WARNING": "WARN", "CRITICAL": "FATAL"}.get(record.levelname, record.levelname)
        message = record.getMessage()
        if record.exc_info:
            message += "\n" + self.formatException(record.exc_info)
        message = safe_text(message, limit=16000)
        stamp = datetime.fromtimestamp(record.created, timezone.utc).isoformat(timespec="milliseconds")
        return f"[{level}] [{record.created:.9f}] [{record.name}]: time={stamp} {message}"


def configure(log_dir):
    logger = logging.getLogger("opendelivery")
    if logger.handlers:
        return logger
    logger.setLevel(logging.INFO)
    logger.propagate = False
    formatter = RosFormatter()
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    logger.addHandler(console)
    try:
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        handler = RotatingFileHandler(Path(log_dir) / "platform.log", maxBytes=5 * 1024 * 1024,
                                      backupCount=5, encoding="utf-8")
        handler.namer = lambda name: name + ".log"
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    except OSError:
        logger.warning("platform log file unavailable", exc_info=True)
    return logger
