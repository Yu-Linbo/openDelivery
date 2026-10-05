# OpenDelivery

**English** | [简体中文](README.zh-CN.md)

OpenDelivery is a ROS 2 workspace for experimenting with multi-robot delivery. It brings together Gazebo simulation, SLAM, localization, Nav2 navigation, task management, and a bilingual Web console for monitoring and controlling robots across floor maps.

## Features

- **Mapping and localization:** build maps with `slam_toolbox`; use Gazebo ground truth for simulation, laser-based localization, or AMCL for legacy maps.
- **Robot tasks:** point-to-point navigation, patrol, path following, and pause/resume/terminate controls, with per-robot task queues and status. Simulated elevator tasks handle travel between floors.
- **Web operations:** monitor maps, robot poses, laser scans, and planned paths; pick navigation goals, switch maps, relocalize, and use keyboard/button teleoperation.
- **Map editing:** edit occupancy grids and semantic layers; manage elevator, standby, custom, and relocalization points.
- **Debugging and replay:** inspect ROS nodes and robot parameters, view Gazebo's overhead camera, and replay recorded bags with synchronized maps, cameras, status, and searchable logs in the browser.
- **OpenClaw assistant:** a chat panel for API-backed robot operations, with persistent conversations and session administration; requires a configured assistant backend.

The console supports English and Simplified Chinese through its language selector. Cross-floor elevator execution currently uses a simulated elevator.

## Delivery demo

[Watch the delivery demo](docs/videos/opendelivery-autonomous-delivery.mp4) (about 6 minutes, English UI): a short OpenClaw request starts robot2 on floor 1, picks up the delivery, and sends it through the simulated elevator to floor 4. The video tours the console, shows the complete assistant feedback, and finishes with the full bag replay at 2×. The TXT log opens for five seconds before collapsing.

See the [recording details and verification](docs/videos/README.md) for the successful task IDs and replay checks.

## Web console tour

### OpenDelivery overview

The console combines floor maps, semantic regions, robot poses, laser scans, planned paths, navigation, relocalization, and teleoperation. This view shows robot1 online in Gazebo on `test_103`.

![OpenDelivery overview with an online robot, laser scan, semantic map, and planned path](docs/images/monitor.png)

### A running delivery task

A real simulated navigation task shows its task ID, `Navigating` status, work queue, and planned route to the bedroom.

![Running simulated delivery task with its navigation route and task work queue](docs/images/robot-task.png)

### Robot status popup and details

The floating status popup lists online/offline robots, floor maps, heartbeat state, and simulation controls. The detail panel adds localization, task state, pose, sensor availability, ROS node count, and process information.

![Floating robot status popup showing online and offline robots](docs/images/robot-presence.png)

![Robot details with live status, localization, pose, sensors, and resources](docs/images/robot-detail.png)

### Robot behavior tree and CPU / memory

Inspect the navigation pipeline and the robot's related processes and ROS nodes.

![Robot navigation behavior tree view](docs/images/robot-tree.png)

![Robot process CPU and memory usage with its ROS nodes](docs/images/robot-resources.png)

### Gazebo simulation

View the live overhead camera across the four simulation zones, inspect frame freshness, adjust the camera, and select robot positioning coordinates.

![Gazebo page with a live overhead view of four simulation zones and camera controls](docs/images/gazebo.png)

### ROS nodes

ROS nodes are grouped by robot and subsystem, with running state and available lifecycle controls.

![ROS node page showing robot simulation and SLAM nodes with lifecycle controls](docs/images/ros-nodes.png)

### Settings and per-robot parameters

Select a robot to configure maximum linear/angular speed and obstacle inflation radius. These settings are also accessible from the robot detail panel.

![Settings page bound to robot1 with speed limits and obstacle inflation radius](docs/images/settings.png)

![Robot detail parameter tab for robot1](docs/images/robot-parameters.png)

### Map editor

Edit occupancy obstacles, semantic labels, and named points in one editor, with undo and save controls.

![Map editor with occupancy and semantic layer tools and saved points](docs/images/map-editor.png)

### Bag archives, playback, and synchronized logs

Browse archived bags and task tags, then play recorded maps, robot state, scans, paths, and front/downward camera images. The log panel follows playback time and supports filtering and seeking from timestamps. Playback reads archived data without publishing ROS commands.

![Robot bag archive with task tags and related recordings](docs/images/logs.png)

![Bag actively playing with recorded map, robot state, and camera frames](docs/images/log-playback.png)

![Bag playback with synchronized searchable text logs](docs/images/replay-logs.png)

All screenshots are captured automatically from the real console in English. Live views use robot1's local Gazebo simulation; playback uses an archived robot1 recording.

## Quick start

Use a configured ROS 2 environment (this workspace targets Foxy), Gazebo, `slam_toolbox`, Nav2, `colcon`, and Python with Pillow. See the [ROS workspace guide](src/README.md) and [technical overview](docs/technical-overview.zh-CN.md) for build and simulation details.

```bash
cd /path/to/openDelivery
./start_web_stack.sh
```

Open the console at <http://localhost:8000>; the API runs at <http://localhost:8001>. The launcher loads the ROS/workspace environment and builds custom message consumers when needed. Local simulation defaults to `ROS_LOCALHOST_ONLY=1`; set it to `0` when connecting ROS nodes across machines.

## Refresh the screenshots

Run the Web/API stack with an online simulated robot, a live Gazebo camera, saved maps, and archived bags. Start a navigation task to capture it in progress, then run:

```bash
npm install -g agent-browser
agent-browser install
python3 scripts/capture_readme_screenshots.py --robot-id robot1 --require-running-task
```

The helper captures 14 views, defaults to English, and writes PNG files to `docs/images/`. Use `--only gazebo,ros-nodes,settings` to refresh selected views and `--bag <archive-path>` to select a recording with camera images. Without `--require-running-task`, the task tab captures its current state. Use `--url http://host:8000` for another console, or `--locale zh-CN --output /tmp/opendelivery-zh` for Chinese screenshots. `AGENT_BROWSER_BIN` can point to an existing CLI installation. The helper reads live state and plays recorded data; it does not start robots, dispatch tasks, or save settings/maps.

## Documentation

- [中文介绍](README.zh-CN.md)
- [Architecture, robot nodes, localization, and Web data flow (中文)](docs/technical-overview.zh-CN.md)
- [ROS packages, builds, and simulation (中文)](src/README.md)
- [HTTP API contract](backend/API.md)
- [Logging and replay (中文)](docs/logging.md)
- [Assistant conversations and administration (中文)](docs/assistant-sessions.md)

Source lives in `src/`, Web assets in `web/`, and the HTTP/ROS bridge in `backend/`. Generated builds, maps, logs, and bags are excluded from Git.
