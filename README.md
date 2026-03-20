# OpenDelivery Web Stack

This module is focused on web display and interaction.
Robot core capabilities such as SLAM and navigation are implemented separately in `src`.

## One-Click Start

```bash
cd /home/ubuntu/project_openclaw/opendelivery
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

## Notes

- Current robot pose source defaults to mock mode for web integration testing.
- You can later replace backend pose source with ROS2 (`rclpy/rclcpp`) without changing frontend contracts.
