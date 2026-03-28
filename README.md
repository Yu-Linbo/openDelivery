# OpenDelivery Source Workspace

ROS 2 工作区：源码在 `src/`，构建产物在 `build/`、`install/`（已加入 `.gitignore`）。

## 技术路线

- **建图与定位**：`slam_toolbox`
- **导航**：`nav2`（Navigation2）
- **地图加载**：`nav2_map_server`（`map_server`）
- **不再使用**：RTAB-Map

以**源码方式**组织依赖（如 `slam_toolbox`），便于二次开发与调试。

## `src` 目录说明

功能包列表、构建命令与运行流程见 **`src/README.md`**（替代原 `src/readme.md` 中的历史 catkin/`ud_*` 说明）。

## 源码依赖（可选克隆）

在 `src` 下按需拉取：

```bash
cd /path/to/openDelivery/src

# slam_toolbox：Foxy 使用 foxy-devel（默认 ros2 分支可能不匹配旧发行版）
git clone https://github.com/SteveMacenski/slam_toolbox.git
cd slam_toolbox && git checkout foxy-devel && cd ..

# nav2（含 map_server 等）
git clone https://github.com/ros-navigation/navigation2.git
```

按发行版切换对应分支或 tag。

## 仿真与 SLAM 联调（简要）

依赖示例（Foxy）：`ros-foxy-gazebo-ros-pkgs`、`ros-foxy-xacro`、`ros-foxy-robot-state-publisher`。

```bash
cd /path/to/openDelivery
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to simulate slam_bringup
source install/setup.bash
ros2 launch simulate headless_sim.launch.py
```

另一终端（已 `source install/setup.bash`）：

```bash
ros2 launch slam_bringup mapping.launch.py
```

详见 `src/simulate/README.md`（目录说明）与 `src/simulate/simulate/README.md`（仿真包文档）。

## Web 栈

```bash
cd /path/to/openDelivery
./start_web_stack.sh
```

前端与 API 见 `web/`、`backend/`。

- **`GET /api/floors`**：返回已保存的地图目录名，并与配置中的机器人条目合并出 synthetic 楼层 **`{robot_id}_mapping`**（用于建图视图）。
- **建图视图**：在监控页选择 **`robot1_mapping`** 这类楼层时，前端轮询 **`GET /api/mapping/live?robot_id=<id>`**，数据来自 ROS **`/<robot_name>/mapping`** 上的 `OccupancyGrid`（由 `backend/ros_tf_bridge.py` 订阅；可用环境变量 **`ROS_MAPPING_TOPIC_TEMPLATE`** 改写，默认 `"/{id}/mapping"`）。可选订阅全局 `/map`：设 **`ROS_SUBSCRIBE_GLOBAL_MAP=1`** 且配置 **`ROS_OCCUPANCY_MAP_TOPIC`**。
- **保存地图**：仅在建图楼层时显示「保存为 / 保存地图」工具条；写入 `map/<名称>/`（`nav2_map_server` 的 `map_saver_cli`）。**切图** 时：后端向 `/<robot_id>/robot_status` 发布 `RobotStatus`，在消息里设置 `current_map`（已保存楼层用该楼层名；建图楼层用「保存为」输入框中的名称）。
- **`GET /api/robot/pose`**（及 SSE）：每台机器人的当前楼层在 JSON 字段 **`active_floor`** 中（由 TF 桥从 `RobotStatus` 合并而来）；前端不再使用已废弃的 `/*/current_map` 话题，也不单独轮询该话题。

保存依赖与仿真包说明见 `src/simulate/simulate/README.md`。
</think>


<｜tool▁calls▁begin｜><｜tool▁call▁begin｜>
Shell
