"""Independent stdout collector: survives API reloads, bounds logs and repeats."""
import argparse
import logging
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from logging.handlers import RotatingFileHandler

import diagnostic_logging as diagnostics

MAX_LINE_BYTES = 64 * 1024


class ManagedLogWriter:
    def __init__(self, path, node_id, *, max_bytes=5 * 1024 * 1024, backups=5, interval_s=10):
        self.node_id = node_id
        self.interval_s = interval_s
        self.handler = RotatingFileHandler(path, maxBytes=max_bytes, backupCount=backups, encoding="utf-8")
        self.handler.namer = lambda name: name + ".log"
        self.handler.setFormatter(logging.Formatter("%(message)s"))
        self.last_line = None
        self.repeats = 0
        self.last_report = time.monotonic()

    def _write(self, text):
        self.handler.handle(logging.LogRecord("node_manager", logging.INFO, "", 0, text, (), None))

    def event(self, name, level="INFO", **fields):
        now = time.time()
        self._write(f"[{level}] [{now:.9f}] [node_manager]: " + diagnostics.fields_text({
            "time": datetime.fromtimestamp(now, timezone.utc).isoformat(timespec="milliseconds"),
            "event": name, "node_id": self.node_id, **fields}))

    def _flush_repeats(self):
        if self.repeats:
            self.event("managed.output_repeated", level="WARN", repeated_lines=self.repeats,
                       sample=diagnostics.safe_text(self.last_line))
            self.repeats = 0
        self.last_report = time.monotonic()

    def write(self, line):
        line = line.rstrip("\r\n")
        if line == self.last_line:
            self.repeats += 1
            if time.monotonic() - self.last_report >= self.interval_s:
                self._flush_repeats()
        else:
            self._flush_repeats()
            self._write(line)
            self.last_line = line

    def close(self):
        self._flush_repeats()
        self.event("managed.output_closed")
        self.handler.close()


def collect(stream, writer):
    while True:
        line = stream.readline(MAX_LINE_BYTES + 1)
        if not line:
            break
        truncated = len(line) > MAX_LINE_BYTES
        if truncated:
            # Drain an oversized physical line in bounded chunks.
            while not line.endswith(b"\n"):
                more = stream.readline(MAX_LINE_BYTES + 1)
                if not more or more.endswith(b"\n"):
                    break
            line = line[:MAX_LINE_BYTES]
        writer.write(line.decode("utf-8", errors="replace"))
        if truncated:
            writer.event("managed.output_line_truncated", level="WARN", limit_bytes=MAX_LINE_BYTES)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", required=True)
    parser.add_argument("--node", required=True)
    args = parser.parse_args()
    Path(args.path).parent.mkdir(parents=True, exist_ok=True)
    writer = ManagedLogWriter(args.path, args.node)
    writer.event("managed.output_opened")
    try:
        collect(sys.stdin.buffer, writer)
    finally:
        writer.close()


if __name__ == "__main__":
    main()
