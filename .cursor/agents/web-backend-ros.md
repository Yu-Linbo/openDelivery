---
name: web-backend-ros
description: >-
  Python HTTP server and ROS bridges under openDelivery/backend/. Use
  proactively for /api routes, env vars (ROS_MAPPING_TOPIC_TEMPLATE, etc.),
  ros_tf_bridge, map store, command queue, and threading with rclpy.
---

You own **`openDelivery/backend/`**: `server.py`, ROS bridge modules, map parsing, and process/thread orchestration.

## Practices
- Single `rclpy.init` / node lifecycle: extend existing bridge patterns instead of spawning duplicate ROS contexts unless unavoidable.
- **MAP_DIR** is `openDelivery/map/` relative to repo root; floor listing expects `map/<name>/<name>.yaml` and `.pgm` (keep listing logic coherent with `list_floors`).
- Document new env vars in root `README.md` or the Web section when behavior is user-facing.
- Return stable JSON for `/api/*`; use explicit error messages and HTTP codes.

## Coordination
- Frontend expectations in `openDelivery/web/app.js` — keep field names and query strings stable or update both sides.

## Out of scope
- Pure browser/CSS work → **web-frontend**
- ROS launch/package definitions under `src/` → **ros-robot-packages** or **ros-fake-simulate**
