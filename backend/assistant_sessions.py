"""Persistent, user-owned assistant conversations behind the shared login proxy."""

import ipaddress
import logging
import os
import sqlite3
import time
import uuid
from pathlib import Path
from urllib.parse import urlsplit

from openclaw_chat import SESSION_RE
import diagnostic_logging as diagnostics

LOGGER = logging.getLogger("opendelivery.sessions")


def authenticated_user(headers, peer):
    # Identity headers are useful only when the TCP peer is the login proxy.
    # nginx must overwrite this header, including for guest requests.
    trusted = os.environ.get("OPEN_DELIVERY_AUTH_PROXIES", "100.64.0.2").split(",")
    try:
        address = ipaddress.ip_address(peer)
        is_proxy = any(address in ipaddress.ip_network(item.strip()) for item in trusted if item.strip())
    except ValueError:
        return None
    if is_proxy and headers.get("X-Auth-User") is not None:
        user = str(headers.get("X-Auth-User") or "").strip()
    elif address.is_loopback:
        # Local workstation access uses a fixed owner, never a client-supplied
        # identity. Check Host and Origin too, since the API allows CORS.
        def is_local_host(host):
            if host == "localhost":
                return True
            try:
                return ipaddress.ip_address(host or "").is_loopback
            except ValueError:
                return False

        try:
            host = urlsplit("//" + str(headers.get("Host") or "")).hostname
            origin = headers.get("Origin")
            if not is_local_host(host):
                return None
            if origin:
                source = urlsplit(str(origin))
                if source.scheme not in ("http", "https") or not is_local_host(source.hostname):
                    return None
        except ValueError:
            return None
        user = os.environ.get("OPEN_DELIVERY_LOCAL_USER", "linbo").strip()
    else:
        return None
    return user if user and user != "guest" and len(user) <= 100 else None


class SessionStore:
    def __init__(self, path):
        self.path = Path(path)

    def connect(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        db = sqlite3.connect(self.path, timeout=10)
        db.row_factory = sqlite3.Row
        db.executescript("""
            CREATE TABLE IF NOT EXISTS sessions (
                id TEXT PRIMARY KEY, owner TEXT NOT NULL, legacy INTEGER NOT NULL,
                active INTEGER NOT NULL, title TEXT NOT NULL, created REAL NOT NULL,
                updated REAL NOT NULL
            );
            CREATE UNIQUE INDEX IF NOT EXISTS active_owner ON sessions(owner) WHERE active=1;
            CREATE TABLE IF NOT EXISTS messages (
                id INTEGER PRIMARY KEY, session_id TEXT NOT NULL, role TEXT NOT NULL,
                text TEXT NOT NULL, created REAL NOT NULL
            );
            CREATE INDEX IF NOT EXISTS session_messages ON messages(session_id,id);
        """)
        os.chmod(self.path, 0o600)
        return db

    def ensure(self, owner, legacy_id="", history=None):
        with self.connect() as db:
            db.execute("BEGIN IMMEDIATE")
            row = db.execute("SELECT * FROM sessions WHERE owner=? AND active=1", (owner,)).fetchone()
            if row is None:
                sid = str(legacy_id or "")
                if not SESSION_RE.fullmatch(sid) or db.execute("SELECT 1 FROM sessions WHERE id=?", (sid,)).fetchone():
                    sid = "opendelivery-" + uuid.uuid4().hex
                now = time.time()
                db.execute("INSERT INTO sessions VALUES (?,?,1,1,?,?,?)", (sid, owner, "原有对话", now, now))
                for item in (history if isinstance(history, list) else [])[-50:]:
                    if isinstance(item, dict) and item.get("role") in ("user", "assistant") and isinstance(item.get("text"), str):
                        db.execute("INSERT INTO messages(session_id,role,text,created) VALUES (?,?,?,?)",
                                   (sid, item["role"], item["text"][:8000], now))
                row = db.execute("SELECT * FROM sessions WHERE id=?", (sid,)).fetchone()
            sid = row["id"]
        return self.get(owner, sid)

    def reset(self, owner, expected_id):
        with self.connect() as db:
            db.execute("BEGIN IMMEDIATE")
            row = db.execute("SELECT id FROM sessions WHERE owner=? AND active=1", (owner,)).fetchone()
            if row is None or row["id"] != expected_id:
                raise ValueError("对话已更新，请刷新后重试")
            db.execute("UPDATE sessions SET active=0 WHERE owner=? AND active=1", (owner,))
            sid, now = "opendelivery-" + uuid.uuid4().hex, time.time()
            db.execute("INSERT INTO sessions VALUES (?,?,0,1,?,?,?)", (sid, owner, "新对话", now, now))
        return self.get(owner, sid)

    def get(self, owner, sid):
        with self.connect() as db:
            row = db.execute("SELECT * FROM sessions WHERE owner=? AND id=?", (owner, sid)).fetchone()
            if row is None:
                return None
            result = dict(row)
            result["messages"] = [dict(item) for item in db.execute(
                "SELECT role,text,created FROM messages WHERE session_id=? ORDER BY id", (sid,))]
            return result

    def list(self, owner):
        with self.connect() as db:
            return [dict(row) for row in db.execute(
                "SELECT s.*, (SELECT count(*) FROM messages m WHERE m.session_id=s.id) AS message_count "
                "FROM sessions s WHERE owner=? ORDER BY active DESC,updated DESC", (owner,))]

    def append(self, owner, sid, role, text):
        with self.connect() as db:
            row = db.execute("SELECT * FROM sessions WHERE owner=? AND id=?", (owner, sid)).fetchone()
            if row is None:
                raise ValueError("对话不存在")
            now = time.time()
            db.execute("INSERT INTO messages(session_id,role,text,created) VALUES (?,?,?,?)",
                       (sid, role, str(text), now))
            title = str(text)[:60] if role == "user" and row["title"] in ("原有对话", "新对话") else row["title"]
            db.execute("UPDATE sessions SET updated=?,title=? WHERE id=?", (now, title, sid))


STORE = SessionStore(os.environ.get("OPEN_DELIVERY_ASSISTANT_DB") or
                     Path(__file__).parent / "data" / "assistant_sessions.sqlite")


def format_actions(actions, texts=None):
    from assistant_language import ZH
    texts = texts or ZH
    return "\n".join((item.get("summary") or texts["completed"]) if item.get("ok") or item.get("confirmation_required")
                     else texts["failed"] + str(item.get("error") or "unknown error") for item in actions)


def handle_request(handler, path, method):
    if not path.startswith("/api/assistant/") or path.startswith("/api/assistant/jobs/"):
        return False
    user = authenticated_user(handler.headers, handler.client_address[0])
    diagnostics.update_context(user=user or "guest")
    if method == "POST" and path in ("/api/assistant/session", "/api/assistant/reset", "/api/assistant/chat"):
        try:
            if int(handler.headers.get("Content-Length") or 0) > 512 * 1024:
                handler._send_json({"error": "请求过大"}, 413)
                return True
        except ValueError:
            handler._send_json({"error": "invalid Content-Length"}, 400)
            return True
        data = handler._read_json_body()
        if data is None:
            return True
        if path == "/api/assistant/session":
            session = STORE.ensure(user, data.get("session_id"), data.get("history")) if user else None
            diagnostics.event(LOGGER, logging.INFO, "assistant.session_resolved", authenticated=bool(user),
                              session_id=session["id"] if session else None,
                              legacy=bool(session and session["legacy"]),
                              peer=handler.client_address[0])
            handler._send_json({"authenticated": bool(user), "user": user, "session": session})
            return True
        if path == "/api/assistant/reset":
            if not user:
                handler._send_json({"error": "请登录后开启新对话"}, 403)
                return True
            try:
                session = STORE.reset(user, data.get("session_id"))
                diagnostics.event(LOGGER, logging.INFO, "assistant.session_reset", previous_session_id=data.get("session_id"),
                                  session_id=session["id"])
                handler._send_json({"session": session})
            except ValueError as err:
                handler._send_json({"error": str(err)}, 409)
            return True
        session = None
        if user:
            session = STORE.get(user, data.get("session_id"))
            if session is None or not session["active"]:
                handler._send_json({"error": "对话已更新，请刷新后重试"}, 409)
                return True
        from openclaw_chat import run_chat, MAX_MESSAGE_CHARS
        message = str(data.get("message") or "").strip()
        if not message or len(message) > MAX_MESSAGE_CHARS:
            handler._send_json({"error": "消息不能为空，且不能超过 4000 字"}, 400)
            return True
        if session:
            STORE.append(user, session["id"], "user", message)
        diagnostics.update_context(session_id=session["id"] if session else str(data.get("session_id") or "")[:96])
        try:
            out = run_chat(message, data.get("session_id"), data.get("context") or {},
                           defer_mutations=True, isolated_session=bool(session and not session["legacy"]),
                           conversation=(user, session["id"]) if session else None)
        except (ValueError, TimeoutError, RuntimeError) as err:
            if session:
                STORE.append(user, session["id"], "assistant", "请求失败：" + str(err))
            handler._send_json({"error": str(err)}, 400 if isinstance(err, ValueError) else
                               504 if isinstance(err, TimeoutError) else 503)
            return True
        if session:
            text = format_actions(out["actions"], out.get("status_text")) if out.get("actions") else out["reply"]
            STORE.append(user, session["id"], "assistant", text)
        handler._send_json(out)
        return True
    if method == "GET" and path == "/api/assistant/sessions":
        if not user:
            handler._send_json({"error": "请登录后查看会话管理"}, 403)
        else:
            handler._send_json({"sessions": STORE.list(user)})
        return True
    if method == "GET" and path.startswith("/api/assistant/sessions/"):
        if not user:
            handler._send_json({"error": "请登录后查看会话管理"}, 403)
        else:
            session = STORE.get(user, path.rsplit("/", 1)[-1])
            handler._send_json({"session": session} if session else {"error": "对话不存在"}, 200 if session else 404)
        return True
    return False
