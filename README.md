# OpenDelivery Web Stack

This module is focused on web display and interaction.
Robot core capabilities such as SLAM and navigation are implemented separately in `src`.

## One-Click Start

```bash
cd /home/ubuntu/project_openclaw/openDelivery
./start_web_stack.sh
```

After startup:

- Frontend: `http://localhost:8000`
- Backend API: `http://localhost:8001`

Press `Ctrl+C` in the same terminal to stop both services.

If ports are occupied, the script automatically picks the next available ports and prints them.

## Backend APIs

- `GET /api/floors`
- `GET /api/maps/{floor}`
- `GET /api/robot/pose`
- `GET /api/robot/status`
- `GET /api/robot/pose/stream` (Server-Sent Events)

## Robot pose: ROS2 TF (default)

Default: **`ROBOT_POSE_MODE=ros2_tf`** — poses come from TF only; **no fake trajectories**.

If ROS2 / `rclpy` / `tf2_ros` is not available, the API returns an **empty `robots` list** until TF works.  
For **local demo only** (fake moving robots): `export ROBOT_POSE_MODE=mock`.

Subscribes to **`/tf`** (and by default **`/tf_static`**) — standard `tf2_msgs/msg/TFMessage`.

Expected TF tree (example):

```text
map
 ├── robot1/odom → robot1/base_link
 └── robot2/odom → robot2/base_link
```

Lookup is **`map` → `robot1/base_link`** (chains through `robot1/odom` in the buffer).

**Current map (floor id for the web UI):** bridge subscribes to **`/robot1/current_map`** (`std_msgs/String`, same as `robot_id`).  
Until the first message, `ROS_CURRENT_MAP` applies. Disable with `ROS_SUBSCRIBE_CURRENT_MAP_TOPIC=0`.

```bash
export ROBOT_POSE_MODE=ros2_tf
export ROS_TF_TOPIC=/tf
export ROS_TF_STATIC_TOPIC=/tf_static   # export ROS_TF_STATIC_TOPIC= to disable
export ROS_FRAME_MAP=map
export ROS_FRAME_BASE=robot1/base_link
export ROS_ROBOT_ID=robot1
export ROS_ROBOT_NAME=robot1
export ROS_CURRENT_MAP=nh_102   # must match a floor id under map/ for the web filter
./start_web_stack.sh
```

When lookup fails: `localization` is `lost`. Pose is **last good** value by default, or **zeros** if:

```bash
export ROS_TF_LOST_POSE=zero   # default is hold (keep last)
```

Multi-robot (JSON overrides the single-robot env vars above):

```bash
export ROS_ROBOTS_TF_JSON='[
  {"id":"robot1","name":"送餐-01","map_frame":"map","base_frame":"robot1/base_link","current_map":"nh_102"},
  {"id":"robot2","name":"送餐-02","map_frame":"map","base_frame":"robot2/base_link","current_map":"nh_103"}
]'
```

(`tf_topic` per robot is optional; defaults to `ROS_TF_TOPIC` or `/tf`.)

Optional tuning: `ROS_TF_LOOKUP_HZ` (default 20), `ROS_TF_CACHE_SEC` (default 30).

Requires ROS2 Python: `rclpy`, `tf2_ros`, `tf2_msgs` (same environment as your robot stack).

If `ros2_tf` fails to start, the server keeps **empty robots** (no mock) and prints a hint. Use `ROBOT_POSE_MODE=mock` only if you explicitly want demo data.

## Notes

- Web UI shows `localization: lost` with dimmed icon and an amber label; status line shows `[丢失]`.
