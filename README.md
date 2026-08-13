# OpenDelivery

ROS 2 多机器人配送实验工作区，包含 Gazebo、GMapping + AMCL、Nav2、状态管理和 Web 控制台。源码在 `src/`，构建产物在 `build/`、`install/`（已加入 `.gitignore`）。

## 技术路线

- **建图与定位**：GMapping + AMCL
- **导航**：`nav2`（Navigation2）
- **地图加载**：`nav2_map_server`（`map_server`）
- **不再使用**：RTAB-Map

GMapping 以项目内源码维护，AMCL 使用 ROS 2 Foxy 的 `nav2_amcl` 系统包。

## 单机节点一览（仿真上线）

入口：`src/system/system/scripts/sim_bringup.sh <robot_id>`（Web「仿真上线」同路径）。图名以 **`/robot2/...`** 为例；多机则换命名空间。

### 分层职责（不要混）

| 层级 | 谁 | 管什么 |
|------|-----|--------|
| **整机开机 / 模式** | `sim_bringup` + `slam/lifecycle_manager` + Nav2 `lifecycle_manager_navigation` | 起进程、mapping/localize、导航栈 configure→activate |
| **业务任务 / 多任务** | `task_manager`、`health_monitor`、心跳 `robot_status` / `task_status`、Web | 切图、重定位、建图/空闲、就绪判定 |
| **单次导航执行** | `bt_navigator` + BT XML | 一个 NavigateToPose：规划→跟踪→恢复 |

- **Lifecycle**：模块电源（能否跑）；可多层 Manager 编排。
- **BT（Nav2）**：只负责**单次导航任务内部**逻辑，不管开机周期与多任务队列。
- **任务管理**：跨任务、跨模式用传统任务管理触发；需要走时再调 Nav2。

```text
sim_bringup / launch
    ├── heartbeat                    ← Lifecycle
    ├── slam/lifecycle_manager      ← 整机级：SLAM 模式 + 跟踪 heartbeat / nav lifecycle
    └── lifecycle_manager_navigation ← 导航子系统：controller / planner / BT / costmap …
              └── bt_navigator       ← 单次导航 BT
```

相对 ROS1 的单体 `move_base`：Nav2 把全局规划、局部控制、恢复、BT、代价地图拆成独立 Lifecycle 节点。

### 每台机器人：节点与功能

| 图名（相对 `/robot2`） | 功能 |
|------------------------|------|
| **系统** | |
| `heartbeat` | Lifecycle；发布 `/robot2/robot_status`（楼层、任务态、定位态）；提供 `set_heartbeat_params` |
| `log_bag`（`robot_log_recorder`） | 终端日志与 rosbag 录制 / 回收 |
| **Manager** | |
| `health_monitor` | 看心跳与位姿协方差，推进 localizing→ready；采纳重定位后的 localization_lost |
| `task_manager` | 订阅 `localize_nav_command` / `set_robot_task`；写心跳；发布 `/robot2/initial` 初值位姿 |
| `slam/lifecycle_manager` | 整机级：切换 SLAM mapping/localize/inactive；跟踪 heartbeat 与导航 lifecycle 状态 |
| **SLAM（同时只跑一种）** | |
| `slam/mapping` | GMapping 建图；出 `/robot2/mapping` 与 `map→odom` |
| `slam/localizing` | AMCL 定位；出 `/robot2/map` 与 `map→odom`；听 `/robot2/initial`（remap 自 `initialpose`） |
| **仿真本体** | |
| `simulate/robot_state_publisher` | 由 URDF 发布 `robot2/*` 连杆 TF |
| `simulate/spawn_entity` | 一次性把模型刷进 Gazebo（短生命周期） |
| **Gazebo 插件节点（URDF 挂载）** | |
| `diff_drive_controller` | 差速；`cmd_vel`→运动；发布 `odom→base` |
| `laser_controller` | 2D 激光 → `/robot2/scan_2d` |
| `imu_plugin` | IMU |
| `camera_controller` | 车载相机 |
| **Nav2** | |
| `lifecycle_manager_navigation` | 按序 configure/activate 下方导航节点 |
| `controller_server` | 局部轨迹跟踪，输出 `cmd_vel`（原 move_base local planner） |
| `planner_server` | 全局路径规划（原 global planner） |
| `recoveries_server` | 恢复行为（清代价图、原地转等） |
| `bt_navigator` | Behavior Tree：编排一次导航任务 |
| `waypoint_follower` | 多航点顺序导航 |
| `controller_server_rclcpp_node` 等 | Nav2 内部辅助 rclcpp 节点（实现细节，一般不直接调） |
| **代价地图** | |
| `global_costmap/global_costmap` | 全局代价图；订 `/robot2/map` 或 `/robot2/mapping`（由 `grid_mode`） |
| `local_costmap/local_costmap` | 局部代价图（跟车窗口） |
| `global_costmap_client` / `local_costmap_client` | 规划器/控制器侧代价图客户端 |
| `*/…_rclcpp_node` | 代价图相关内部辅助节点 |

### 全局（多机共用）：节点与功能

| 图名 | 功能 |
|------|------|
| `/gazebo` | Gazebo 物理与传感器仿真（gzserver） |
| `/drawn_model/topdown_camera/...` | 世界俯视相机（Web 顶视） |
| `/open_delivery_web_tf_bridge` | Web 后端：TF/位姿、scan、路径、切图命令桥 |
| `/open_delivery_gazebo_set_state` | Web：`/gazebo/set_model_state` 持久客户端（瞬移等） |

现场核对：`ros2 node list`（或 CLI `ros-read 'ros2 node list'`）。包与 TF 约定见 **`src/README.md`**。

## `src` 目录说明

功能包列表、构建命令与运行流程见 **`src/README.md`**（替代原 `src/readme.md` 中的历史 catkin/`ud_*` 说明）。

## 源码依赖

GMapping 源码位于 `src/slam/slam_gmapping`，基于 ROS 2 `eloquent-devel` 端口并补充多机器人 frame 参数。
Nav2 使用系统 ROS 安装提供的包，不需要把 Navigation2 源码复制到本工作区。

## 仿真与 SLAM 联调（简要）

依赖示例（Foxy）：`ros-foxy-gazebo-ros-pkgs`、`ros-foxy-xacro`、`ros-foxy-robot-state-publisher`。

```bash
cd /path/to/openDelivery
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to simulate manager
source install/setup.bash
ros2 launch simulate simulate.launch.py
```

另一终端（已 `source install/setup.bash`）：

```bash
ros2 launch manager manager.launch.py namespace:=robot2 initial_slam_mode:=mapping
```

详见 **`src/README.md`**（含 TF 约定、仿真与构建说明）。

## Web 栈

```bash
cd /home/ubuntu/project_openclaw/openDelivery
./start_web_stack.sh
```

默认前端 `http://localhost:8000`，后端 `http://localhost:8001`。前端与 API 分别见 `web/`、`backend/`；接口契约见 **`backend/API.md`**。

### 监控页当前能力

- **切图 / 重定位**：机器人 ID 下拉只列在线机器人；可填写坐标或在地图上选择位姿、导航目标。
- **地图点位**：支持电梯点、待机点和自定义点位。电梯点、待机点始终显示；自定义点按开关显示。
- **语义叠加**：语义 PNG 可按开关以透明层叠加，不替换占用栅格底图。
- **地图编辑**：支持栅格画笔/擦除、语义标注/擦除，以及点位新增、移动、删除。跨层修改会一起保存；切图前有未保存确认。
- **简易遥控**：按住方向按钮或 `W/A/S/D` 发布机器人命名空间下的 `cmd_vel`，松手、页面失焦、切换机器人或网络租约超时都会停车。

### 地图目录约定

每张已保存地图位于 `map/<floor>/`：

| 文件 | 用途 |
|------|------|
| `<floor>.pgm` / `<floor>.yaml` | ROS 占用栅格及元数据 |
| `<floor>_semantic.png` | 与栅格同尺寸的语义图 |
| `semantic_legend.json` | 固定语义颜色图例 |
| `<floor>_points.json` | 地图级点位：`elevator`、`standby`、`custom` |

地图资源由后端原子写入并校验格式、尺寸和有限坐标，不由浏览器直接访问工作区文件。

### 关键数据链路

- **`GET /api/floors`**：返回已保存的地图目录名，并与配置中的机器人条目合并出 synthetic 楼层 **`{robot_id}_mapping`**（用于建图视图）。
- **建图视图**：在监控页选择 **`robot1_mapping`** 这类楼层时，前端轮询 **`GET /api/mapping/live?robot_id=<id>`**，数据来自 ROS **`/<robot_name>/mapping`** 上的 `OccupancyGrid`（由 `backend/ros_tf_bridge.py` 订阅；可用环境变量 **`ROS_MAPPING_TOPIC_TEMPLATE`** 改写，默认 `"/{id}/mapping"`）。可选订阅全局 `/map`：设 **`ROS_SUBSCRIBE_GLOBAL_MAP=1`** 且配置 **`ROS_OCCUPANCY_MAP_TOPIC`**。
- **保存地图**：仅在建图楼层时显示「保存为 / 保存地图」工具条；写入 `map/<名称>/`（`nav2_map_server` 的 `map_saver_cli`）。**切图** 时：后端向 `/<robot_id>/robot_status` 发布 `RobotStatus`，在消息里设置 `current_map`（已保存楼层用该楼层名；建图楼层用「保存为」输入框中的名称）。
- **`GET /api/robot/pose`**（及 SSE）：每台机器人的当前楼层在 JSON 字段 **`active_floor`** 中（由 TF 桥从 `RobotStatus` 合并而来）；前端不再使用已废弃的 `/*/current_map` 话题，也不单独轮询该话题。
- **地图资源 API**：`GET /api/maps/{floor}/assets` 读取语义图例和点位；三个 POST 路由分别保存点位、栅格和语义图。
- **遥控 API**：`POST /api/robot/motion/teleop` 使用浏览器会话、单调序列号和 0.8 秒可续租租约；后端始终维持每机器人最多一个遥控发布进程。

保存依赖与仿真说明见 **`src/README.md`** §6。

## 文档分工

- `README.md`：项目总览、节点和 Web 入口。
- `backend/API.md`：HTTP API 契约与安全语义。
- `src/README.md`：ROS 包、TF、构建和运行流程。
- `src/navigation/nav_bringup/README.md`：Nav2 启动细节。
- `src/slam/slam_gmapping/README.md`：GMapping / AMCL 输入输出接口。
