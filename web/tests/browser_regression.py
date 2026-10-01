"""Browser regressions for a running OpenDelivery stack.

Requires agent-browser, robot1 in the presence cache, and a saved map.
Edits are discarded; motion and map save APIs are never invoked.
"""
import os
import shlex
import shutil
import subprocess
import sys

browser = os.environ.get('AGENT_BROWSER_BIN') or shutil.which('agent-browser')
if not browser:
    sys.exit('agent-browser is required; set AGENT_BROWSER_BIN to its executable.')
url = os.environ.get('OPEN_DELIVERY_WEB_URL', 'http://127.0.0.1:8000')
args = os.environ.get('AGENT_BROWSER_ARGS', '--no-sandbox,--no-zygote,--single-process,--disable-dev-shm-usage,--disable-gpu')


def evaluate(source):
    return shlex.join(['eval', source])


commands = ['open ' + url]
for width in [320, 390, 768, 1024, 1280, 1440]:
    commands.append('set viewport %s 900' % width)
    for locale in ['zh-CN', 'en']:
        commands.append(evaluate("OpenDeliveryI18n.setLocale('%s')" % locale))
        for view in ['monitor', 'gazebo', 'ros', 'settings', 'logs']:
            commands.append(evaluate("""(async()=>{
              activateView('%s'); await new Promise(r=>setTimeout(r,30));
              if(document.body.scrollWidth>innerWidth+1)
                throw Error(location.hash+' '+OpenDeliveryI18n.locale+' '+innerWidth+' overflow '+document.body.scrollWidth);
              return 'layout passed';
            })()""" % view))

commands += [
    'set viewport 390 844',
    evaluate("""OpenDeliveryI18n.setLocale('en');activateView('logs');
      document.getElementById('btn-robot-presence').click();
      if(document.body.scrollWidth>innerWidth+1)throw Error('presence overflow');
      'presence passed'"""),
    'screenshot /tmp/od-audit-mobile-after.png',
    evaluate("document.getElementById('btn-robot-presence').click();activateView('monitor');openRobotDetail('robot1');"),
    evaluate("""(async()=>{
      for(let i=0;i<100&&!robotDetailPayload;i++)await new Promise(r=>setTimeout(r,100));
      if(!robotDetailPayload)throw Error('detail failed');
      robotDetailActiveTab='params';renderRobotDetail();
      const field=document.querySelector('#robot-detail-params-form input');field.value='0.31';
      await refreshRobotDetail();
      if(field!==document.querySelector('#robot-detail-params-form input')||field.value!=='0.31')
        throw Error('parameter edit lost');
      closeRobotDetail();return 'parameter refresh passed';
    })()"""),
    evaluate("""(()=>{
      const form=document.getElementById('openclaw-chat-form');let submits=0;
      const onSubmit=e=>{submits++;e.preventDefault();};form.addEventListener('submit',onSubmit);
      const input=document.getElementById('openclaw-chat-input');input.value='机器人状态';
      input.dispatchEvent(new KeyboardEvent('keydown',{key:'Enter',isComposing:true,bubbles:true,cancelable:true}));
      form.removeEventListener('submit',onSubmit);input.value='';
      if(submits)throw Error('IME submitted');return 'IME passed';
    })()"""),
    evaluate("""(async()=>{
      const fetch=window.fetch;
      window.fetch=(url,...args)=>String(url).endsWith('/api/assistant/chat')
        ?Promise.resolve(new Response(JSON.stringify({reply:'测试响应'}),{status:200,headers:{'Content-Type':'application/json'}}))
        :fetch(url,...args);
      try {
        const input=document.getElementById('openclaw-chat-input');input.value='机器人状态';
        document.getElementById('openclaw-chat-form').requestSubmit();
        for(let i=0;i<100&&document.getElementById('openclaw-chat-send').disabled;i++)await new Promise(r=>setTimeout(r,30));
        OpenDeliveryI18n.setLocale('en');await new Promise(r=>setTimeout(r,30));
        const users=document.querySelectorAll('.openclaw-chat-message.user');
        if(users[users.length-1].textContent!=='机器人状态')throw Error('user text translated');
        const label=document.createElement('span');label.textContent='正在读取…';document.body.append(label);
        await new Promise(r=>setTimeout(r,30));
        if(/[\\u3400-\\u9fff]/.test(label.textContent))throw Error('dynamic label untranslated');
        OpenDeliveryI18n.setLocale('zh-CN');
        if(label.textContent!=='正在读取…')throw Error('source text lost');label.remove();
        return 'language round trip and user text passed';
      } finally {window.fetch=fetch;}
    })()"""),
    'set viewport 1440 1000',
    evaluate("OpenDeliveryI18n.setLocale('zh-CN');activateView('monitor')"),
    'click #btn-map-editor-open',
    evaluate("""(async()=>{
      const $=name=>document.querySelector('[data-map-editor='+name+']');
      for(let i=0;i<100&&!$('loading').hidden;i++)await new Promise(r=>setTimeout(r,100));
      if(!$('loading').hidden)throw Error('editor load failed');
      const canvas=$('canvas'),rect=canvas.getBoundingClientRect();
      canvas.dispatchEvent(new PointerEvent('pointerdown',{button:0,pointerId:1,
        clientX:rect.x+rect.width/2,clientY:rect.y+rect.height/2,bubbles:true}));
      canvas.dispatchEvent(new PointerEvent('pointerup',{pointerId:1,bubbles:true}));
      if(!StandaloneMapEditor.dirty)throw Error('paint failed');
      const confirm=window.confirm;let asked=0;window.confirm=()=>{asked++;return false;};
      try {
        await StandaloneMapEditor.load(StandaloneMapEditor.mapName);
        if(!asked||!StandaloneMapEditor.dirty)throw Error('same map reload discarded edits');
        const fetch=window.fetch;let rejectSave;
        window.fetch=(url,options)=>options?.method==='POST'
          ?new Promise((resolve,reject)=>{rejectSave=reject;}):fetch(url,options);
        try {
          $('layer-save').click();
          if(!document.querySelector('.standalone-map-editor__body').inert||!$('map-name').disabled)
            throw Error('save did not lock editor');
          const name=StandaloneMapEditor.mapName;
          await StandaloneMapEditor.load('should_not_load');
          if(StandaloneMapEditor.mapName!==name)throw Error('map changed during save');
          rejectSave(Error('simulated save failure'));
          await new Promise(r=>setTimeout(r,30));
          if(!StandaloneMapEditor.dirty||document.querySelector('.standalone-map-editor__body').inert)
            throw Error('save failure did not retain dirty state');
        } finally {window.fetch=fetch;}
        window.confirm=()=>true;$('layer-discard').click();
        if(StandaloneMapEditor.dirty)throw Error('discard failed');
      } finally {window.confirm=confirm;}
      return 'editor reload and failed save protection passed';
    })()"""),
    'screenshot /tmp/od-audit-editor-after.png',
    evaluate("StandaloneMapEditor.close();activateView('monitor');resetViewToFit();renderScene();"),
    'screenshot /tmp/od-audit-desktop-after.png',
    'errors',
]
result = subprocess.run([browser, '--session', 'od-browser-regression', '--args', args, 'batch', *commands],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
print(result.stdout)
subprocess.run([browser, '--session', 'od-browser-regression', '--args', args, 'close'],
               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
sys.exit(result.returncode)
