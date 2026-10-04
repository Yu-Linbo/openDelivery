"""Real-browser regressions using temporary sessions and simulated task states.

No production conversation, robot command, model call or saved map is changed.
The path endpoint deliberately permits caching to reproduce a cached second leg.
"""
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'backend'))
import assistant_sessions
import openclaw_chat as chat
from assistant_language import response_texts

browser = os.environ.get('AGENT_BROWSER_BIN') or shutil.which('agent-browser')
if not browser:
    sys.exit('Set AGENT_BROWSER_BIN to agent-browser.')
app = (ROOT / 'web/app.js').read_text()
html = (ROOT / 'web/index.html').read_text()
panel = html[html.index('    <button id="openclaw-chat-trigger"'):html.index('    <script src="./i18n.js')]
chat_js = app[app.index('function initOpenClawChat() {'):]
poll_js = app[app.index('function stopSensorPolling() {'):app.index('function stopScanStream() {')]
poll_js += app[app.index('async function pollRobotSensor('):app.index('function bindMapInteractions()')]
optional_js = app[app.index('async function fetchJsonOptional('):app.index('async function fetchFloors(')]


def fake_chat(message, session_id, context, **kwargs):
    language, texts = response_texts(message)
    jid = ('a' if language == 'en' else 'b') * 32
    conversation = kwargs.get('conversation')
    if conversation:
        assistant_sessions.STORE.append(*conversation, 'assistant', texts['accepted'])
    chat._JOBS[jid] = {'job_id': jid, 'status': 'running', 'results': [], 'events': [],
                      'event_seq': 0, 'conversation': conversation, 'status_text': texts}
    chat._publish_job_event(jid, texts['step'].format(index=1, total=1, action=texts['action_navigate_to_point'], robot_id='robot2'))
    Handler.jid = jid
    return {'job_id': jid, 'reply': texts['accepted'], 'actions': [], 'status_text': texts, 'language': language}


class Handler(BaseHTTPRequestHandler):
    path_version = 1
    jid = ''

    def log_message(self, *args):
        pass

    def _send_json(self, payload, status=200, cache='no-store'):
        raw = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', cache)
        self.send_header('Content-Length', str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _read_json_body(self):
        return json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))))

    def auth(self):
        self.headers['X-Auth-User'] = 'browser-progress-test'

    def do_POST(self):
        self.auth()
        if not assistant_sessions.handle_request(self, urlparse(self.path).path, 'POST'):
            self.send_error(404)

    def do_GET(self):
        self.auth()
        path = urlparse(self.path).path
        if path == '/__second_path':
            Handler.path_version = 2
            self._send_json({'ok': True})
        elif path.endswith('/planned_path'):
            self._send_json({'points': [[Handler.path_version, 0], [Handler.path_version, 1]]}, cache='max-age=300')
        elif path == '/__advance':
            phase = urlparse(self.path).query
            job = chat._JOBS[self.jid]
            texts = job['status_text']
            if phase in ('complete', 'fail'):
                job['status'] = 'completed' if phase == 'complete' else 'failed'
                message = texts['plan_completed'].format(count=1) if phase == 'complete' else texts['failed'] + 'navigation failed'
                chat._publish_job_event(self.jid, message, 'terminal')
            else:
                task = {'task_status': 'Navigating', 'current_index': int(phase), 'total_count': 2,
                        'work_queue': ['navigation:elevator_waiting:test_101', 'navigation:goal:test_101']}
                token = chat._JOB_PROGRESS.set(lambda m: chat._publish_job_event(self.jid, m))
                try:
                    chat._navigation_progress('robot2', task, {'status': {'floor': 'test_101'}}, texts)
                finally:
                    chat._JOB_PROGRESS.reset(token)
            self._send_json({'ok': True})
        elif path.startswith('/api/assistant/jobs/'):
            self._send_json(chat.get_action_job(path.rsplit('/', 1)[1]))
        elif assistant_sessions.handle_request(self, path, 'GET'):
            return
        elif path == '/':
            prelude = '''const API_BASE_URL=location.origin, floorSelect=null, relocRobotId=null, selectedDetailRobotId="";
const latestPathByRobot={},latestScanByRobot={},sensorPollControllers=new Set();
let sensorPollGeneration=0,sensorPollInFlight=false,sensorPollTimer=null,scanStreamActive=false;
const scan2dToggle={checked:false},plannedPathToggle={checked:true};
const paints=[];function scheduleMapPaint(){paints.push(latestPathByRobot.robot2.points[0][0]);}
function startScanStream(){}function stopScanStream(){}function getRobotsOnCurrentMap(){return[{id:'robot2'}];}
async function fetchJson(url,options){const r=await fetch(url,options),p=await r.json();if(!r.ok)throw Error(p.error);return p;}
window.scriptErrors=[];window.addEventListener('error',e=>scriptErrors.push(e.message));
'''
            source = '<!doctype html><html><head><meta charset="utf-8"><link rel="stylesheet" href="/styles.css"></head><body>' + panel
            source += '<script src="/i18n.js"></script><script>' + prelude + optional_js + poll_js + chat_js + '\nstartSensorPolling();</script></body></html>'
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(source.encode())
        elif path in ('/i18n.js', '/styles.css'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/javascript' if path.endswith('.js') else 'text/css')
            self.end_headers()
            self.wfile.write((ROOT / 'web' / path[1:]).read_bytes())
        else:
            self.send_error(404)


def ev(source):
    return shlex.join(['eval', source])


with tempfile.TemporaryDirectory() as temp, mock.patch.dict(os.environ, {'OPEN_DELIVERY_AUTH_PROXIES': '127.0.0.1'}), \
        mock.patch.object(assistant_sessions, 'STORE', assistant_sessions.SessionStore(Path(temp) / 'sessions.sqlite')), \
        mock.patch.object(chat, 'run_chat', side_effect=fake_chat):
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    commands = ['open http://127.0.0.1:' + str(server.server_port),
        ev('''(async()=>{await new Promise(r=>setTimeout(r,500));
if(!paints.includes(1))throw Error('first path missing');await fetch('/__second_path');
await new Promise(r=>setTimeout(r,700));if(!paints.includes(2))throw Error('second cached path needs reload');
OpenDeliveryI18n.setLocale('en');
if(OpenDeliveryI18n.pointName({name:'呼梯点'})!=='Elevator call point')throw Error('call point untranslated');
if(OpenDeliveryI18n.pointName({name:'前台取货点'})!=='Reception pickup point')throw Error('pickup untranslated');
if(OpenDeliveryI18n.pointName({name:'梯内点'})!=='Inside-elevator point')throw Error('inside point untranslated');
if(OpenDeliveryI18n.pointName({name:'卧室'})!=='Bedroom')throw Error('bedroom untranslated');
if(!OpenDeliveryI18n.pointName({name:'重定位点 2026-08-17 02:07:14'}).startsWith('Relocalization point '))throw Error('generated point untranslated');
if(OpenDeliveryI18n.pointName({name:'测试点',name_en:'Test destination'})!=='Test destination')throw Error('explicit English name missing');
OpenDeliveryI18n.setLocale('zh-CN');
if(OpenDeliveryI18n.pointName({name:'测试点',name_en:'Test destination'})!=='测试点')throw Error('Chinese name lost');
return 'same-map second path and bilingual point names passed';})()'''),
        'click #openclaw-chat-trigger', 'fill #openclaw-chat-input Go to the elevator', 'click #openclaw-chat-send',
        ev('''(async()=>{await new Promise(r=>setTimeout(r,300));await fetch('/__advance?0');
await new Promise(r=>setTimeout(r,1800));
if(!document.getElementById('openclaw-chat-messages').textContent.includes('Subtask 1/2'))throw Error('first subtask feedback missing');
location.reload();return 'first subtask feedback arrived; reloading during execution';})()'''),
        'click #openclaw-chat-trigger',
        ev('''(async()=>{await new Promise(r=>setTimeout(r,300));await fetch('/__advance?1');
await new Promise(r=>setTimeout(r,1800));const box=document.getElementById('openclaw-chat-messages');
if(!box.textContent.includes('Subtask 2/2'))throw Error('feedback not resumed after reload');
if([...box.children].filter(n=>n.textContent.includes('Subtask 1/2')).length!==1)throw Error('restored feedback duplicated');
await fetch('/__advance?complete');await new Promise(r=>setTimeout(r,1800));
if(!box.textContent.includes('Task completed. All 1 planned steps finished.'))throw Error('completion missing');
if(document.getElementById('openclaw-chat-send').disabled)throw Error('send remains blocked');
if(Object.keys(localStorage).some(k=>k.endsWith('_pending_job')))throw Error('completed job remains pending');
return 'subtask feedback, reload recovery, English completion with Chinese UI passed';})()'''),
        'fill #openclaw-chat-input 去电梯', 'click #openclaw-chat-send',
        ev('''(async()=>{await new Promise(r=>setTimeout(r,300));await fetch('/__advance?fail');
await new Promise(r=>setTimeout(r,1800));
if(!document.getElementById('openclaw-chat-messages').textContent.includes('执行失败：'))throw Error('failure feedback missing');
if(scriptErrors.length)throw Error(scriptErrors.join(';'));
return 'Chinese failure feedback and zero JavaScript errors passed';})()'''),
        'screenshot /tmp/opendelivery-task-feedback.png']
    args = ['--session', 'task-feedback-test', '--args', '--no-sandbox,--no-zygote,--single-process,--disable-dev-shm-usage,--disable-gpu']
    try:
        result = subprocess.run([browser, *args, 'batch', *commands], capture_output=True, text=True)
        print(result.stdout, result.stderr)
        if result.returncode:
            sys.exit(result.returncode)
    finally:
        subprocess.run([browser, *args, 'close'], capture_output=True)
        server.shutdown()
        chat._JOBS.pop('a' * 32, None)
        chat._JOBS.pop('b' * 32, None)
