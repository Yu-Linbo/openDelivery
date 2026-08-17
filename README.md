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
| **业务任务 / 多任务** | `task_manager` + `TaskInfo/TaskStatus/TaskCommand` | 点到点、巡逻、following 的分解、控制和状态汇总 |
| **导航任务执行** | `navigation_task_executor` | 将导航任务转换为 `NavigateToPose` / `FollowPath`，处理暂停、恢复和终止 |
| **单次 Nav2 执行** | `bt_navigator` + BT XML | 一个 NavigateToPose：规划→跟踪→恢复 |

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
| `task_manager` | 订阅根 `task_info/task_command`，向导航层分发任务并汇总 `task_status`；兼容 `localize_nav_command` / `set_robot_task`，发布 `/robot2/initial` |
| `slam/lifecycle_manager` | 整机级：切换 SLAM mapping/localize/inactive；跟踪 heartbeat 与导航 lifecycle 状态 |
| **SLAM（同时只跑一种）** | |
| `slam/mapping` | GMapping 建图；出 `/robot2/mapping` 与 `map→odom` |
| `slam/localizing` | AMCL 定位；出 `/robot2/map` 与 `map→odom`；听 `/robot2/initial`（remap 自 `initialpose`） |
| **仿真本体** | |
| `simulate/robot_state_publisher` | 由 URDF 发布 `robot2/*` 连杆 TF |
| `simulate/spawn_entity` | 一次性把模型刷进 Gazebo（短生命周期） |
| `fake_elevator` | 仿真乘梯/换层执行器：接收 task_manager 子任务，更新楼层、动态切图并调用重定位 |
| **Gazebo 插件节点（URDF 挂载）** | |
| `diff_drive_controller` | 差速；`cmd_vel`→运动；发布 `odom→base` |
| `laser_controller` | 2D 激光 → `/robot2/scan_2d` |
| `imu_plugin` | IMU |
| `camera_controller` | 车载相机 |
| **Nav2** | |
| `navigation/lifecycle_manager` | 按序 configure/activate 下方导航节点 |
| `navigation/task_executor` | 订阅导航层任务和命令；驱动 Nav2 action 并发布模块任务状态 |
| `navigation/controller_server` | 局部轨迹跟踪，输出导航速度（原 move_base local planner） |
| `navigation/planner_server` | 全局路径规划（原 global planner） |
| `navigation/recoveries_server` | 恢复行为（清代价图、原地转等） |
| `navigation/bt_navigator` | Behavior Tree：编排一次导航任务；内部 action 为 `/robot2/navigation/navigate_to_pose`，兼容保留 `/robot2/navigate_to_pose` |
| `navigation/waypoint_follower` | 多航点顺序导航 |
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
| `/open_delivery_web_tf_bridge` | Web 后端：TF/位姿、scan、路径、切图、根任务发布及汇总任务状态缓存 |
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

- **切图 / 重定位**：机器人 ID 下拉只列在线机器人；可填写坐标或在地图上选择位姿、导航目标；定位准确时可记录当前位姿和单帧雷达为重定位点。
- **地图导航目标**：地图选点不再由 Web 直接调用 Nav2，而是创建根 `TaskInfo`，经过 `task_manager` 和导航任务执行器下发。
- **机器人任务页**：展示根 `TaskStatus` 的任务 ID、汇总状态、进度、工作队列及模块状态。
- **地图点位**：支持电梯内点、电梯等待点、待机点、自定义点位和重定位点；重定位点与自定义点尺寸一致并复用删除功能。
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
| `<floor>_points.json` | 地图级点位：`elevator_inside`、`elevator_waiting`、`standby`、`custom`、`relocalization`；旧 `elevator` 兼容为梯内点 |
| `relocalization/*.rloc` | 重定位位姿、激光外参及单帧扫描的二进制记录；按 points JSON 清理孤立记录 |

地图资源由后端原子写入并校验格式、尺寸和有限坐标，不由浏览器直接访问工作区文件。

### 关键数据链路

- **`GET /api/floors`**：返回已保存的地图目录名，并与配置中的机器人条目合并出 synthetic 楼层 **`{robot_id}_mapping`**（用于建图视图）。
- **建图视图**：在监控页选择 **`robot1_mapping`** 这类楼层时，前端轮询 **`GET /api/mapping/live?robot_id=<id>`**，数据来自 ROS **`/<robot_name>/mapping`** 上的 `OccupancyGrid`（由 `backend/ros_tf_bridge.py` 订阅；可用环境变量 **`ROS_MAPPING_TOPIC_TEMPLATE`** 改写，默认 `"/{id}/mapping"`）。可选订阅全局 `/map`：设 **`ROS_SUBSCRIBE_GLOBAL_MAP=1`** 且配置 **`ROS_OCCUPANCY_MAP_TOPIC`**。
- **保存地图**：仅在建图楼层时显示「保存为 / 保存地图」工具条；写入 `map/<名称>/`（`nav2_map_server` 的 `map_saver_cli`）。**切图** 时：后端向 `/<robot_id>/robot_status` 发布 `RobotStatus`，在消息里设置 `current_map`（已保存楼层用该楼层名；建图楼层用「保存为」输入框中的名称）。
- **`GET /api/robot/pose`**（及 SSE）：每台机器人的当前楼层在 JSON 字段 **`active_floor`** 中（由 TF 桥从 `RobotStatus` 合并而来）；前端不再使用已废弃的 `/*/current_map` 话题，也不单独轮询该话题。
- **地图资源 API**：`GET /api/maps/{floor}/assets` 读取语义图例和点位；三个 POST 路由分别保存点位、栅格和语义图。
- **遥控 API**：`POST /api/robot/motion/teleop` 使用浏览器会话、单调序列号和 0.8 秒可续租租约；后端始终维持每机器人最多一个遥控发布进程。
- **Web 导航任务**：`POST /api/robot/motion/goto` 转成 `TaskInfo` 后按下列路径执行；Web 桥订阅根 `TaskStatus`，供机器人详情页轮询展示。

```text
Web 地图选点
  → /<robot>/task_info
  → task_manager
  → /<robot>/navigation/task_info
  → navigation/task_executor
  → /<robot>/navigation/navigate_to_pose
  → /<robot>/navigation/task_status
  → task_manager 汇总 /<robot>/task_status
  → Web 机器人任务页
```

### 任务消息与当前范围

| 消息 | 关键字段 | 说明 |
|------|----------|------|
| `TaskInfo` | `task_id`、`task_type`、`task_name`、`end_action`、`floor_ids`、`poses` | `task_type` 当前支持 `navigation`、`patrol`、`following` |
| `TaskStatus` | `task_id`、`work_queue`、`model_status`、`task_status`、索引/总数 | 根状态由 task_manager 汇总；状态输出使用 transient-local |
| `TaskCommand` | `task_id`、`command` | `Pause`、`Resume`、`Terminate` |

- 点到点和巡逻逐点调用 `NavigateToPose`；巡逻第一点为 `Navigating`，第二点起根状态为 `Patrolling`。
- following 将 poses 组成 `Path` 调用 `FollowPath`；导航层为 `Following`，根层汇总为 `Cleaning`。
- `end_action=back` 时，最后一个 pose 必须由调用方提供为返回点。
- 当 `floor_ids` 与 `RobotStatus.current_map` 不同时，`task_manager` 自动展开为：本层电梯等待点导航
  → `fake_elevator:call`（3 秒模拟呼梯）→ 本层梯内点导航 → `fake_elevator:ride`
  → 目标层电梯等待点导航（保存方向自动反转 180°）→ 用户目标点。
- 每个参与换层的地图必须恰好配置一个 `elevator_inside` 和一个 `elevator_waiting`。当前楼层未知、
  点位缺失或重复时任务会在移动前返回 Failed；显式 `floor_ids` 必须全部非空且与 poses 等长。
- 乘梯阶段先用 `/gazebo/set_model_state` 把机器人模型移动到目标层梯内点，再把 heartbeat 的
  `current_map` 更新为目标楼层并标记 `localization_lost`，随后调用 `map_server/load_map` 加载地图，
  最后固定使用上一楼层梯内点作为模式 1 的输入调用 `/<robot>/relocalize`。
- 假电梯状态包括 `Calling`、`Riding`、`MovingModel`、`SwitchingMap`、`Relocalizing` 和最终状态，
  均汇总到根 `TaskStatus`；Pause、Resume、Terminate 会转发到当前工作模块。
- 当前 `fake_elevator` 仅模拟乘梯延时和地图切换，不包含真实电梯的呼梯、进梯、门控及安全联锁；
  以后可在保持 `ElevatorInfo/ElevatorStatus/ElevatorCommand` 接口的前提下替换为真实电梯管理器。
- `test_101` 已将现有“电梯内点/呼梯点”迁移到新类型；其他楼层需要在 Web 地图编辑器中配置这两类点后才能参与换层。

保存依赖与仿真说明见 **`src/README.md`** §6。

## 文档分工

- `README.md`：项目总览、节点和 Web 入口。
- `backend/API.md`：HTTP API 契约与安全语义。
- `src/README.md`：ROS 包、TF、构建和运行流程。
- `src/navigation/nav_bringup/README.md`：Nav2 启动细节。
- `src/slam/slam_gmapping/README.md`：GMapping / AMCL 输入输出接口。
