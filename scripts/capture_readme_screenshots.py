#!/usr/bin/env python3
"""Capture the running Web console for both READMEs (English by default)."""
import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=os.environ.get("OPEN_DELIVERY_WEB_URL", "http://127.0.0.1:8000"))
    parser.add_argument("--locale", choices=("en", "zh-CN"), default="en")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "docs/images")
    parser.add_argument("--robot-id", default="", help="Online robot to show (default: first online robot)")
    parser.add_argument("--bag", default="", help="Archived bag path for replay (default: newest archived bag)")
    parser.add_argument("--only", default="", help="Comma-separated screenshot names without .png")
    parser.add_argument("--require-running-task", action="store_true", help="Require an executing task for robot-task")
    options = parser.parse_args()
    browser = os.environ.get("AGENT_BROWSER_BIN") or shutil.which("agent-browser")
    if not browser:
        parser.error("agent-browser is required; install it or set AGENT_BROWSER_BIN")
    options.output.mkdir(parents=True, exist_ok=True)
    command = [browser, "--session", "opendelivery-readme"]
    if os.environ.get("AGENT_BROWSER_EXECUTABLE_PATH"):
        command += ["--executable-path", os.environ["AGENT_BROWSER_EXECUTABLE_PATH"]]
    if os.environ.get("AGENT_BROWSER_ARGS"):
        command += ["--args", os.environ["AGENT_BROWSER_ARGS"]]

    def js(source):
        return shlex.join(["eval", source])

    def screenshot(name):
        return shlex.join(["screenshot", str(options.output / name)])

    # Read-only capture: callers prepare simulation/navigation before running this helper.
    # Opening archived bag playback uses the read-only replay API, never ROS playback.
    detail = """
        setRobotPresenceOpen(false);openRobotDetail(window.readmeRobotId);
        for(let i=0;i<150 && !robotDetailPayload;i++)await new Promise(r=>setTimeout(r,100));
        if(!robotDetailPayload)throw Error('Robot detail failed to load');
    """
    preparations = {
        "robot-task": detail + """
          const task=robotDetailPayload.task;
          if(window.readmeRequireTask && (!task || !/Navigating|Patrolling|Following|Cleaning|Calling|Riding|MovingModel|SwitchingMap|Relocalizing/.test(task.task_status)))
            throw Error('Start a task before capturing robot-task');
          document.querySelector('[data-robot-detail-tab=tasks]').click();
        """,
        "monitor": "closeRobotDetail();setRobotPresenceOpen(false);",
        "robot-presence": """
          closeRobotDetail();await fetchRobotStatusCache();renderRobotPresencePanel();setRobotPresenceOpen(true);
        """,
        "robot-detail": detail,
        "robot-tree": detail + "document.querySelector('[data-robot-detail-tab=tree]').click();",
        "robot-resources": detail + "document.querySelector('[data-robot-detail-tab=cpu]').click();",
        "robot-parameters": detail + "document.querySelector('[data-robot-detail-tab=params]').click();",
        "map-editor": """
          closeRobotDetail();setRobotPresenceOpen(false);await StandaloneMapEditor.open(activeFloor);
          if(!document.querySelector('[data-map-editor=loading]').hidden)throw Error('Map editor did not load');
        """,
        "gazebo": """
          closeRobotDetail();setRobotPresenceOpen(false);StandaloneMapEditor.close();
          document.querySelector('[data-view=gazebo]').click();
          for(let i=0;i<150;i++){
            if(document.getElementById('gazebo-camera-wrap').classList.contains('gazebo-camera-wrap--fresh'))break;
            await new Promise(r=>setTimeout(r,100));
          }
          if(!document.getElementById('gazebo-camera-wrap').classList.contains('gazebo-camera-wrap--fresh'))
            throw Error('No live Gazebo camera frame; start the simulation first');
          await new Promise(r=>setTimeout(r,1000));
        """,
        "ros-nodes": """
          closeRobotDetail();setRobotPresenceOpen(false);StandaloneMapEditor.close();
          document.querySelector('[data-view=ros]').click();await refreshRosNodesStatus();
          if(!document.querySelector('#ros-robot-groups .ros-node-card'))throw Error('ROS node list is empty');
        """,
        "settings": """
          closeRobotDetail();setRobotPresenceOpen(false);StandaloneMapEditor.close();
          document.querySelector('[data-view=settings]').click();await fetchRobotStatusCache();
          document.getElementById('settings-robot-id').value=window.readmeRobotId;
          await loadSettingsForRobot(window.readmeRobotId);
          if(document.querySelector('#settings-form input').disabled)throw Error('Settings did not load');
        """,
        "logs": """
          closeRobotDetail();setRobotPresenceOpen(false);StandaloneMapEditor.close();
          document.querySelector('[data-view=logs]').click();
          document.getElementById('log-bag-robot-select').value=window.readmeRobotId;await refreshLogBags();
          if(!logBagEntries.length)throw Error('No recorded bags available');
        """,
        "log-playback": """
          closeRobotDetail();setRobotPresenceOpen(false);StandaloneMapEditor.close();
          document.querySelector('[data-view=logs]').click();
          document.getElementById('log-bag-robot-select').value=window.readmeRobotId;await refreshLogBags();
          const index=logBagEntries.findIndex(entry=>window.readmeBag ? entry.bag===window.readmeBag : !entry.live && entry.downloadable && entry.bytes>0);
          if(index<0)throw Error('No archived bag available');
          selectedLogBagIndices=new Set([index]);renderLogBagList();renderLogBagFiles();
          await openSelectedLogBagReplay();
          if(!bagReplayState.data || !bagReplayState.mapBitmap)throw Error('Bag replay did not load');
          const samples=bagReplayTimeline('poses');
          const images=bagReplayTimeline('images');
          bagReplayState.currentTime=images.at(-1)?.t || samples[Math.floor(samples.length/2)]?.t || bagReplayState.data.duration/2;
          await loadBagReplayMap(bagReplayMapAt(bagReplayState.currentTime),bagReplayState.loadToken);
          document.getElementById('log-viewer').open=false;
          setBagReplayPlaying(true);renderBagReplayFrame();
          await new Promise(r=>setTimeout(r,800));
        """,
    }
    preparations["replay-logs"] = preparations["log-playback"].replace(".open=false;", ".open=true;")
    selected = options.only.split(",") if options.only else list(preparations)
    unknown = set(selected) - preparations.keys()
    if unknown:
        parser.error("Unknown screenshot names: " + ", ".join(sorted(unknown)))
    commands = [
        shlex.join(["open", options.url.split("#", 1)[0] + "#monitor"]),
        "set viewport 1600 1100",
        js("""(async()=>{
          for(let i=0;i<150;i++){
            if(typeof OpenDeliveryI18n!=='undefined' && typeof mapBitmap!=='undefined' && mapBitmap)break;
            await new Promise(r=>setTimeout(r,100));
          }
          if(typeof mapBitmap==='undefined'||!mapBitmap)throw Error('No saved map loaded');
          OpenDeliveryI18n.setLocale(%s);
          await fetchRobotStatusCache();
          const robot=robotStatusCacheItems.find(r=>r.online && (!%s || r.id===%s));
          if(!robot)throw Error('Start an online robot before capturing the gallery');
          window.readmeRobotId=robot.id;window.readmeFloor=robot.floor;
          window.readmeRequireTask=%s;window.readmeBag=%s;
          floorSelect.value=robot.floor;await loadFloorMap(robot.floor);
          for(const id of ['semantic-map-toggle','custom-points-toggle','planned-path-toggle','scan-2d-toggle']){
            const node=document.getElementById(id);if(node && !node.checked)node.click();
          }
          resetViewToFit();renderScene();return 'Live robot: '+robot.id;
        })()""" % (json.dumps(options.locale), json.dumps(options.robot_id), json.dumps(options.robot_id), str(options.require_running_task).lower(), json.dumps(options.bag))),
    ]
    for name in selected:
        commands += [
            js("""(async()=>{
              document.getElementById('openclaw-chat-close').click();
              setBagReplayDialogOpen(false);StandaloneMapEditor.close();
              activateView('monitor');window.scrollTo(0,0);
              %s
              await new Promise(requestAnimationFrame);await new Promise(requestAnimationFrame);
              await new Promise(r=>setTimeout(r,150));
              return %s;
            })()""" % (preparations[name], json.dumps(name + " ready"))),
            screenshot(name + ".png"),
        ]
    commands += ["errors", "close"]
    try:
        result = subprocess.run(command + ["batch", "--bail", *commands], check=False)
        return result.returncode
    finally:
        subprocess.run(command + ["close"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    sys.exit(main())
