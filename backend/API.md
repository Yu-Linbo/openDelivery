# OpenDelivery Backend API

Backend base URL defaults to `http://<host>:8001`.

Web pages must use these HTTP APIs for all runtime data and operations. The frontend must not read workspace files directly; maps, robot state, logs, and downloads are exposed by backend endpoints.

## Web APIs

These endpoints are used by `web/app.js`.

### Bootstrap

- `GET /api/web/bootstrap`
  - Purpose: initial Web payload.
  - Returns: `floors`, `robot_status_cache`, `pose`, `log_bag`, `timestamp`.

### Robot Presence And Lifecycle

- `GET /api/robot/status/cache`
  - Purpose: list known robots from persisted status plus live pose/managed process state.
  - Returns: `{ items: [...] }`.

- `GET /api/robot/pose`
  - Purpose: current pose snapshot.
  - Returns: `{ timestamp, source, robots }`.

- `GET /api/robot/pose/stream`
  - Purpose: SSE stream for pose updates.

- `POST /api/ros/lifecycle/startup`
  - Body: `{ "robot_id": "robot2", "sim_mode": "sim" }`
  - Purpose: start selected robot simulation stack.

- `POST /api/ros/lifecycle/startup-cancel`
  - Body: `{ "robot_id": "robot2" }`
  - Purpose: cancel frontend pending startup state.

- `POST /api/ros/lifecycle/shutdown`
  - Body: `{ "robot_id": "robot2" }`
  - Purpose: shutdown selected robot simulation stack.

### Maps And Monitor

- `GET /api/floors`
  - Purpose: list available map floors.
  - Returns: `{ floors: ["nh_102", ...] }`.

- `GET /api/maps/{floor}`
  - Purpose: read map data through backend.
  - Returns: `{ floor, pgm, yaml }`.

- `GET /api/mapping/live?robot_id={robot}`
  - Purpose: live mapping occupancy grid snapshot.

- `POST /api/mapping/save`
  - Body: `{ "map_name": "...", "robot_id": "robot2" }`
  - Purpose: save current mapping result.

- `GET /api/robot/{robot_id}/scan_2d`
  - Purpose: current 2D scan overlay.

- `GET /api/robot/{robot_id}/planned_path`
  - Purpose: current planned path overlay.

### Robot Commands

- `POST /api/robot/command`
  - Purpose: map switch and localization commands.
  - Typical body: `{ "type": "localize_nav_command", "robot_id": "robot2", "mode": "both", "map_name": "nh_102", "x": 0, "y": 0, "yaw": 0 }`.

### Gazebo Web View

- `GET /api/gazebo/topdown/state`
  - Purpose: read topdown camera model pose.

- `GET /api/gazebo/top_camera/status`
  - Purpose: topdown image availability and freshness.

- `GET /api/gazebo/top_camera`
  - Purpose: topdown image frame.

- `POST /api/gazebo/set_model_state`
  - Purpose: move Gazebo model/camera through backend.

### Logs

- `GET /api/log_bag/matches`
  - Purpose: list all robots with `log_bag/*/backup/match.json`.

- `GET /api/log_bag/matches?robot_name={robot}`
  - Purpose: read one robot's `match.json`.
  - If not found, returns the robot entry with `no_log: true` and empty `bags`.

- `POST /api/log_bag/download`
  - Body: `{ "files": ["log_bag/robot2/backup/bags/20260609T080918_terminal_bag", "log_bag/robot2/backup/logs/20260609T080918_terminal_log.txt"] }`
  - Purpose: download selected log files/directories as a zip.
  - Zip layout: flat top-level entries only — `*_terminal_bag/` directories and `*_terminal_log.txt` files (no `log_bag/.../backup/...` prefix).

## Web And AI Shared APIs

These are safe for both Web and future AI workflows when called through backend.

- `GET /api/robot/waypoints?robot_id={robot}`
  - List waypoints.

- `POST /api/robot/waypoints/record`
  - Body: `{ "robot_id": "robot2", "name": "home", "x": 0, "y": 0, "yaw": 0 }`
  - Record waypoint.

- `POST /api/robot/waypoints/goto`
  - Body: `{ "robot_id": "robot2", "name": "home" }`
  - Navigate to waypoint.

- `POST /api/robot/motion/goto`
  - Body: `{ "robot_id": "robot2", "x": 0, "y": 0, "yaw": 0 }`
  - Send Nav2 goal.

- `POST /api/robot/motion/velocity`
  - Body: `{ "robot_id": "robot2", "linear": 0.1, "angular": 0.0, "seconds": 1, "confirmed": true }`
  - Confirmed timed velocity command.

## Reserved For AI / Debug

These endpoints are not primary Web user workflows. Keep them backend-only and use with care.

- `POST /api/ros/read-only`
  - Body: `{ "cmd": "ros2 topic list", "timeout": 15 }`
  - Read-only ROS introspection allowlist.

- `GET /api/ros/lifecycle/status`
  - Lifecycle component status.

- `POST /api/ros/lifecycle/transition`
  - Component lifecycle transition.

- `GET /api/ros/debug/nodes`
  - Debug node table.

- `POST /api/ros/debug/nodes/lifecycle_set`
  - Debug lifecycle transition by node name.

- `GET /api/ros/nodes/status`
  - Managed/discovered ROS node status.

- `POST /api/ros/nodes/create`
  - Create supported managed ROS node.

- `POST /api/ros/nodes/control`
  - Start/pause/restart managed node.

- `POST /api/ros/nodes/discovered/kill`
  - Kill discovered ROS node.

- `GET /api/ros/threads/status`
  - Backend ROS bridge thread status.

- `POST /api/ros/threads/control`
  - Backend ROS bridge thread control.
