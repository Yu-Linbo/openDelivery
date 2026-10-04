"""Exercise the real chat UI and session APIs with a simulated trusted login proxy.

No robot commands or LLM calls are made; all state is in a temporary database.
"""
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock
from urllib.parse import urlparse
import json

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend"))
import assistant_sessions
import openclaw_chat
from assistant_language import response_texts

browser = os.environ.get("AGENT_BROWSER_BIN") or shutil.which("agent-browser")
if not browser:
    sys.exit("Set AGENT_BROWSER_BIN to agent-browser.")
calls = []


def chat(message, session_id, context, **kwargs):
    calls.append((session_id, kwargs["isolated_session"]))
    language, texts = response_texts(message)
    reply = "Test reply: " if language == "en" else "测试回复："
    return {"reply": reply + message, "actions": [], "session_id": session_id,
            "language": language, "status_text": texts}


class Handler(BaseHTTPRequestHandler):
    user = "guest"

    def log_message(self, *args):
        pass

    def _send_json(self, payload, status=200):
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(payload, ensure_ascii=False).encode())

    def _read_json_body(self):
        return json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))

    def do_POST(self):
        if self.user != "local":
            self.headers["X-Auth-User"] = self.user
        if not assistant_sessions.handle_request(self, urlparse(self.path).path, "POST"):
            self.send_error(404)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/__user":
            selected = urlparse(self.path).query
            Handler.user = selected if selected in ("linbo", "local") else "guest"
            self._send_json({"ok": True})
            return
        if self.user != "local":
            self.headers["X-Auth-User"] = self.user
        if assistant_sessions.handle_request(self, path, "GET"):
            return
        if path == "/":
            source = (ROOT / "web/app.js").read_text()
            chat_source = "function initOpenClawChat() {" + source.split("function initOpenClawChat() {", 1)[1]
            html = (ROOT / "web/index.html").read_text()
            panel = html[html.index('    <button id="openclaw-chat-trigger"'):html.index('    <script src="./i18n.js')]
            data = ('<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">'
                    '<link rel="stylesheet" href="/styles.css"></head><body>' + panel + '<script src="/i18n.js"></script>' +
                    '<script>const API_BASE_URL=location.origin;const floorSelect=null;const relocRobotId=null;'
                    'const selectedDetailRobotId="";async function fetchJson(url,options){const r=await fetch(url,options);'
                    'const p=await r.json();if(!r.ok)throw Error(p.error);return p;}' + chat_source + '</script></body></html>').encode()
            content_type = "text/html; charset=utf-8"
        else:
            name = path.lstrip("/")
            if name not in ("assistant-admin.html", "assistant-admin.css", "assistant-admin.js", "styles.css", "i18n.js"):
                self.send_error(404)
                return
            data = (ROOT / "web" / name).read_bytes()
            if name == "assistant-admin.html":
                data = data.replace(b"<head>", b"<head><script>window.API_BASE_URL=location.origin;</script>")
            content_type = "text/html" if name.endswith("html") else "text/css" if name.endswith("css") else "text/javascript"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.end_headers()
        self.wfile.write(data)


def ev(js):
    return shlex.join(["eval", js])


with tempfile.TemporaryDirectory() as temp, \
        mock.patch.dict(os.environ, {"OPEN_DELIVERY_AUTH_PROXIES": "127.0.0.1"}), \
        mock.patch.object(assistant_sessions, "STORE", assistant_sessions.SessionStore(Path(temp) / "history.sqlite")), \
        mock.patch.object(openclaw_chat, "run_chat", side_effect=chat):
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    url = "http://127.0.0.1:" + str(server.server_port)
    commands = [
        "open " + url,
        "click #openclaw-chat-trigger",
        ev("""(async()=>{await new Promise(r=>setTimeout(r,100));
          OpenDeliveryI18n.setLocale('en');await new Promise(r=>setTimeout(r,40));
          const welcome=document.querySelector('#openclaw-chat-messages .assistant');
          if(!welcome.textContent.startsWith('Hello. I can query robot status'))throw Error('English welcome absent');
          if(welcome.hasAttribute('data-i18n-ignore'))throw Error('Welcome cannot follow UI language');
          OpenDeliveryI18n.setLocale('zh-CN');await new Promise(r=>setTimeout(r,40));
          if(!welcome.textContent.startsWith('你好，我可以查询机器人状态'))throw Error('Chinese welcome absent');
          OpenDeliveryI18n.setLocale('en');return 'welcome follows selected language';})()"""),
        ev("""(async()=>{await new Promise(r=>setTimeout(r,100));
          if(!document.getElementById('openclaw-chat-reset').hidden||!document.getElementById('openclaw-admin-link').hidden)throw Error('guest controls visible');
          const r=await fetch('/api/assistant/reset',{method:'POST',body:'{}'});if(r.status!==403)throw Error('guest reset allowed');
          const s=await fetch('/api/assistant/sessions');if(s.status!==403)throw Error('guest admin allowed');
          return 'guest permission checks passed';})()"""),
        "fill #openclaw-chat-input 旧对话消息",
        "click #openclaw-chat-send",
        ev("""(async()=>{for(let i=0;i<100&&document.getElementById('openclaw-chat-send').disabled;i++)await new Promise(r=>setTimeout(r,20));
          if(!document.getElementById('openclaw-chat-messages').textContent.includes('测试回复'))throw Error('guest reply absent');
          await fetch('/__user?linbo');location.reload();return 'guest continued old conversation';})()"""),
        "click #openclaw-chat-trigger",
        ev("""(async()=>{for(let i=0;i<100&&document.getElementById('openclaw-chat-reset').hidden;i++)await new Promise(r=>setTimeout(r,20));
          if(document.getElementById('openclaw-chat-reset').hidden)throw Error('login controls absent');
          if(!document.getElementById('openclaw-chat-messages').textContent.includes('旧对话消息'))throw Error('legacy history lost');
          return 'login restored legacy history';})()"""),
        "click #openclaw-chat-reset",
        ev("""(async()=>{for(let i=0;i<100&&document.getElementById('openclaw-chat-reset').disabled;i++)await new Promise(r=>setTimeout(r,20));
          if(document.querySelector('.openclaw-chat-message.user'))throw Error('reset retained messages');
          await new Promise(r=>setTimeout(r,40));
          if(!document.querySelector('#openclaw-chat-messages .assistant').textContent.startsWith('Hello. I can query robot status'))throw Error('New session welcome ignores selected English');
          OpenDeliveryI18n.setLocale('zh-CN');
          const p=await(await fetch('/api/assistant/sessions')).json();if(p.sessions.length!==2)throw Error('archive missing');
          return 'new chat and archive passed';})()"""),
        "fill #openclaw-chat-input <img src=x onerror=alert(1)> 新消息",
        "click #openclaw-chat-send",
        ev("""(async()=>{for(let i=0;i<100&&document.getElementById('openclaw-chat-send').disabled;i++)await new Promise(r=>setTimeout(r,20));
          if(document.querySelector('#openclaw-chat-messages img'))throw Error('unsafe rendering');
          OpenDeliveryI18n.setLocale('en');await new Promise(r=>setTimeout(r,40));
          const replies=document.querySelectorAll('.openclaw-chat-message.assistant');
          if(!replies[replies.length-1].textContent.startsWith('测试回复：'))throw Error('Chinese reply changed with UI language');
          OpenDeliveryI18n.setLocale('zh-CN');return 'Chinese reply follows request even with English UI';})()"""),
        "fill #openclaw-chat-input go floor1 take delivery to 3 floor",
        "click #openclaw-chat-send",
        ev("""(async()=>{for(let i=0;i<100&&document.getElementById('openclaw-chat-send').disabled;i++)await new Promise(r=>setTimeout(r,20));
          const replies=document.querySelectorAll('.openclaw-chat-message.assistant');
          if(!replies[replies.length-1].textContent.startsWith('Test reply: '))throw Error('English reply lost');
          if(!replies[replies.length-1].hasAttribute('data-i18n-ignore'))throw Error('Reply is coupled to UI language');
          return 'English request receives English reply with Chinese UI';})()"""),
        "set viewport 390 844",
        "screenshot /tmp/openclaw-new-chat.png",
        "open " + url + "/assistant-admin.html",
        ev("""(async()=>{for(let i=0;i<100&&document.querySelectorAll('.session').length<2;i++)await new Promise(r=>setTimeout(r,20));
          if(document.querySelectorAll('.session').length!==2)throw Error('admin list failed');
          if(document.body.scrollWidth>innerWidth)throw Error('admin mobile overflow');
          if(document.querySelector('#messages img'))throw Error('admin unsafe rendering');
          document.querySelectorAll('.session')[1].click();
          for(let i=0;i<100&&!document.getElementById('messages').textContent.includes('旧对话消息');i++)await new Promise(r=>setTimeout(r,20));
          if(!document.getElementById('messages').textContent.includes('旧对话消息'))throw Error('archived messages missing');
          return 'admin history, mobile layout and text escaping passed';})()"""),
        "screenshot /tmp/openclaw-admin-history.png",
        ev("""(async()=>{await fetch('/__user?guest');document.getElementById('refresh').click();
          for(let i=0;i<100&&document.getElementById('login').hidden;i++)await new Promise(r=>setTimeout(r,20));
          if(document.getElementById('login').hidden||document.querySelector('#messages .message'))throw Error('logout retained admin access');
          return 'expired login denies admin and clears messages';})()"""),
        ev("""(async()=>{await fetch('/__user?local');location.href='/';return 'switch to direct localhost access';})()"""),
        "click #openclaw-chat-trigger",
        ev("""(async()=>{for(let i=0;i<100&&document.getElementById('openclaw-chat-reset').hidden;i++)await new Promise(r=>setTimeout(r,20));
          if(document.getElementById('openclaw-chat-reset').hidden||document.getElementById('openclaw-admin-link').hidden)throw Error('localhost default login controls absent');
          const state=await(await fetch('/api/assistant/session',{method:'POST',body:'{}'})).json();
          if(!state.authenticated||state.user!=='linbo')throw Error('localhost default owner absent');
          return 'localhost defaults to logged-in owner with new chat and admin';})()"""),
        "errors",
    ]
    args = ['--session', 'assistant-test', '--args', '--no-sandbox,--no-zygote,--single-process,--disable-dev-shm-usage,--disable-gpu']
    try:
        result = subprocess.run([browser, *args, 'batch', *commands], capture_output=True, text=True)
        print(result.stdout)
        print(result.stderr)
        if result.returncode:
            sys.exit(result.returncode)
        assert len(calls) == 3 and not calls[0][1] and calls[1][1] and calls[2][1], calls
        print('Legacy guest context and isolated new context verified.')
    finally:
        subprocess.run([browser, *args, 'close'], capture_output=True)
        server.shutdown()
