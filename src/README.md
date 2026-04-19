# OpenDelivery `src` 功能包说明

本目录为 **ROS 2** 工作区源码根（与历史 **ROS 1 / catkin**、`ud_*` 命名工程无关）。当前路线：

- **SLAM**：`slam_toolbox`（建图 / 定位）
- **导航**：`nav2` + `nav2_map_server`（规划与加载地图，按需接入）
- **不再使用**：RTAB-Map

---

## 1. TF 树（与 `backend/ros_tf_bridge.py` 一致）

Web 后端从共享的 `/tf`、`/tf_static` 做 `map` → 各机器人 `base_link` 查询。约定典型结构为：

```
map
 ├── robot1/odom → robot1/base_link
 └── robot2/odom → robot2/base_link
```

`→` 表示从父到子的链。URDF 中常见 **`robotN/odom` → `robotN/base_footprint` → `robotN/base_link`**；桥在说明里常写到 `base_link` 为止。多机时帧名带 **`robotN/`** 前缀，与 `/*/robot_status` 动态发现一致。

**与仿真的分工（两棵 TF 树）**：**`map`** 及 **`map` → `odom`** 由 **SLAM / 定位**（如 `slam_bringup`）维护；**Gazebo `simulate.launch.py` 只发布里程计树 **`robotN/odom` → … → `robotN/base_link`**，**不**发布 `map`，也不在仿真里做 `map→odom` 静态 TF。联调全栈时再同时起 SLAM 与仿真，由 SLAM 把 `map` 接到各机器人 `odom`。

---

## 2. 顶层模块目录（规划占位）

以下目录用于按职责拆分功能包（无 `ud_` 前缀）；空目录用 `.gitkeep` 占位以便 Git 跟踪：

| 目录 | 用途 |
|------|------|
| `algorithm/` | 算法与业务逻辑 |
| `driver/` | 传感器、底盘等驱动 |
| `slam/` | SLAM 相关包与参数（见下） |
| `calibration/` | 标定 |
| `navigation/` | Nav2 配置与 launch |
| `system/` | 整机 bringup |
| `common/` | 公共库与消息（按需） |
| `perception/` | 感知 |
| `tool/` | 工具脚本 |

---

## 3. 节点命名（约定）

在 **`/<robot_name>/`** 下：单节点包用 **包名** 作为节点名（如 `/robot2/heartbeat`）；包内多节点用 **`/<robot_name>/<包名>/<节点名>`**（如 `/robot2/slam_bringup/mapping`、`/robot2/simulate/robot_state_publisher`）。传感器/里程计等话题仍以 **`/<robot_name>/...`** 为前缀，与命名空间分层解耦。

---

## 4. 运行时各节点与作用（概览）

下文用 **`<R>`** 表示机器人命名空间（与 `robot_name` / `namespace` 一致，如 `robot2`）。**是否出现**取决于当前 launch（仅仿真、仅心跳、全栈等）。

### 4.1 仿真 `simulate`（`simulate.launch.py`）

| 节点 / 进程 | 作用 |
|-------------|------|
| **`gzserver`**（经 `gazebo_ros` 启动） | Gazebo 世界与物理仿真 |
| **`/<R>/simulate/robot_state_publisher`** | 订阅 Gazebo 发布的 `joint_states`，按 URDF 发布 **`/<R>/odom` → `base_footprint` → `base_link`** 等 TF |
| **`/<R>/simulate/spawn_entity`** | 将模型按 `robot_description` 生成到 Gazebo（通常很快结束） |
| **Gazebo 插件（随 URDF）** | 差速底盘（`cmd_vel`）、**`/<R>/scan_2d`**、IMU、相机等 |

### 4.2 心跳与管理器（`heartbeat.launch.py`，含 `manager`）

| 节点 | 作用 |
|------|------|
| **`/<R>/heartbeat`**（Lifecycle） | 周期发布 **`/<R>/robot_status`**（`RobotStatus`）；提供 **`/<R>/set_heartbeat_params`** 服务，供外部/Web 改地图名、状态枚举等 |
| **`/<R>/health_monitor`** | 监视必要节点与可选定位话题，通过 **`set_heartbeat_params`** 推进 **`robot_status`** 生命周期（如 initializing → localizing → ready） |
| **`/<R>/task_manager`** | 提供 **`/<R>/set_robot_task`** 等接口，将任务侧 **`task_status`** 写入 heartbeat（再反映到 `robot_status`） |

### 4.3 SLAM `slam_bringup`

| 节点 | 作用 |
|------|------|
| **`/<R>/slam_bringup/mapping`**（`managed_slam_lifecycle`） | **建图**：`slam_toolbox` 在线建图，发布 **`/<R>/mapping`**（`OccupancyGrid`）与 **`map`→`/<R>/odom`** 等 TF |
| **`/<R>/slam_bringup/localization`**（`managed_slam_lifecycle`） | **定位**：加载已有地图，发布 **`/<R>/map`** 与定位 TF；与 mapping **二选一** Lifecycle active |

### 4.4 导航 `nav_bringup`（`stack.launch.py` → `navigation_namespaced.launch.py`）

均在 **`/<R>/`** 下；全局代价地图通过 remap 订阅 **`/<R>/map`** 或 **`/<R>/mapping`**（由 `grid_mode` 决定）。

| 节点 | 作用 |
|------|------|
| **`/<R>/controller_server`** | 局部规划 / DWB，输出 `cmd_vel` |
| **`/<R>/planner_server`** | 全局路径（NavFn 等） |
| **`/<R>/recoveries_server`** | 脱困（旋转、后退、等待等） |
| **`/<R>/bt_navigator`** | 行为树导航（NavigateToPose 等） |
| **`/<R>/waypoint_follower`** | 多点巡逻 / 航点跟随 |
| **`/<R>/lifecycle_manager_navigation`** | 统一管理上述 Nav2 节点的 lifecycle |

### 4.5 演示 `fake_pub`（`params/launch/fake/fake_pub.launch.py`）

| 节点 | 作用 |
|------|------|
| **`/<R>/fake`** | 轻量假数据（位姿/话题），便于无 Gazebo 时与 Web 联调 |

### 4.6 Web 后端桥（非 `src` 内 ROS 包）

| 组件 | 作用 |
|------|------|
| **`backend/ros_tf_bridge.py`（ROS 2 节点）** | 聚合多机 TF、`/*/robot_status` 发现、激光/路径/topdown 相机等供 **HTTP** 使用；与上表节点通过话题/服务协同 |

---

## 5. 已有 ROS 2 包（当前仓库内）

| 包名 | 路径 | 说明 |
|------|------|------|
| **simulate** | `simulate/simulate/` | Gazebo 无头仿真；`simulate/` 为容器目录，包在子目录中 |
| **slam_bringup** | `slam/slam_bringup/` | `slam_toolbox` 建图 / 定位 launch 与参数 |
| **slam_toolbox** | `slam/slam_toolbox/` | 上游源码（通常 **`foxy-devel`** 分支，与 ROS 2 Foxy 对齐） |
| **nav_bringup** | `navigation/nav_bringup/` | 仅 Nav2 导航；代价地图直接订 `/<robot>/map` 或 `/<robot>/mapping`（`grid_mode`）；TF 由 SLAM；见 `navigation/nav_bringup/README.md` |
| **heartbeat** | `system/heartbeat/` | `RobotStatus` 周期发布、`set_heartbeat_params`；Lifecycle 节点 **`heartbeat`**；launch 内嵌 **manager** |
| **manager** | `system/manager/` | **`health_monitor`**、**`task_manager`**：状态机与任务指令转发到 heartbeat |
| **system** | `system/system/` | 元包：安装 `params/`、`sim_bringup.sh` 等；`fake_pub` 可执行安装为 **`fake_pub_node`** |
| **custom_msgs_srvs** | `common/custom_msgs_srvs/` | `RobotStatus.msg`、`SetHeartbeatParams.srv` 等接口定义 |

### 非 colcon 包（脚本）

| 内容 | 路径 | 说明 |
|------|------|------|
| **fake 演示** | `fake/scripts/` | 如 `fake_pub.py`，用于 Web 联调演示轨迹与话题 |

---

## 6. 构建（`colcon`）

在**工作区根** `openDelivery/`（与 `src/`、`install/` 同级）执行：

```bash
cd /path/to/openDelivery
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to simulate slam_bringup
# 含心跳、任务管理、Nav2 bringup（依赖会一并解析）：
# colcon build --symlink-install --packages-up-to heartbeat nav_bringup
# 若包含 slam_toolbox 源码：
# colcon build --symlink-install --packages-select slam_toolbox simulate slam_bringup
source install/setup.bash
```

**源码构建 `slam_toolbox` 时**常见系统依赖（Foxy 示例）：`libceres-dev`、`ros-foxy-bondcpp`；仿真另需：`ros-foxy-gazebo-ros-pkgs`、`ros-foxy-xacro`、`ros-foxy-robot-state-publisher`。

---

## 7. 运行流程（简版）

### 7.0 一键：建图 + 心跳 + 导航（推荐）

在仓库根目录**已** `source install/setup.bash`，且 **终端 1** 已按与 `robot_name` 一致的命名空间起好仿真或真机（提供 `/clock`、`/<robot>/odom`、`/<robot>/scan_2d` 等）：

**建图模式（示例 `robot2`）：**

```bash
cd /path/to/openDelivery
./start_heartbeat_slam_nav.sh robot2 mapping
```

等价拉起：`heartbeat`（含 **health_monitor / task_manager**，`mapping_mode:=true`）+ `slam_bringup/mapping` + `nav_bringup`（`grid_mode:=mapping`，Nav2 直接订 `/<robot>/mapping`）。可选环境变量见脚本内注释（如 `USE_SIM_TIME`、`CURRENT_MAP`）。

**Nav2 系统包（Foxy，`apt` 安装示例）：**

```bash
sudo apt-get install -y \
  ros-foxy-nav2-bt-navigator \
  ros-foxy-nav2-controller \
  ros-foxy-nav2-planner \
  ros-foxy-nav2-recoveries \
  ros-foxy-nav2-waypoint-follower \
  ros-foxy-nav2-lifecycle-manager \
  ros-foxy-nav2-bringup \
  ros-foxy-nav2-behavior-tree
```

若 `lifecycle_manager` 在 configure `bt_navigator` 时报 **无法打开某 `libnav2_*_bt_node.so`**，先确认已装 `ros-foxy-nav2-behavior-tree`；若仍异常，检查 `nav_bringup` 是否已 **`colcon build`** 且 `stack.launch.py` 生成的参数文件未被**重复嵌套**命名空间（见 `navigation/nav_bringup/README.md`）。

**`USE_SIM_TIME=true`（默认）时**必须有 **`/clock`**（Gazebo / `ros2 bag play` 等），否则 Nav2 可能在 configure 阶段长时间阻塞，看起来像卡住。

**定位 + 导航（需已有地图 yaml）：**

```bash
./start_heartbeat_slam_nav.sh robot2 normal /absolute/path/to/map.yaml
```

### 7.1 仅仿真 + SLAM（不用一键脚本时）

```bash
# 终端 1：无头 Gazebo（若 gzserver 异常退出可先：pkill -9 gzserver）
ros2 launch simulate simulate.launch.py

# 终端 2（已 source install）
ros2 launch slam_bringup mapping.launch.py
```

保存地图（需安装 `nav2_map_server` 等）：

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/sim_map
```

### 7.2 定位模式（已有地图）

```bash
ros2 launch slam_bringup localization.launch.py map_file:=/tmp/sim_map.yaml
```

### 7.3 真机/实车

在 `driver/`、`system/` 中逐步接入传感器与底盘 launch，再与 `slam_bringup` 或 `navigation/` 串联（具体 launch 待后续补充）。

---

## 8. 仿真包 `simulate`（路径 `simulate/simulate/`）

### 8.1 目录说明

- **`src/simulate/`**：容器目录，只放仿真相关 ament 包，本层不直接放 `package.xml`。
- **包本体**：`src/simulate/simulate/`（包名 **`simulate`**）。新增仿真包时可在 `src/simulate/<包名>/` 下并列创建。

### 8.2 话题与 TF（Gazebo）

命名空间由 launch **`namespace`** 决定（默认随 **`robot_name`**）。插件话题在 **`/<namespace>/...`**（激光为 **`scan_2d`**，另有 `imu/data`、`cmd_vel`、`odom`、`camera/...`），与 **`PushRosNamespace`** 一致；详见 `simulate/launch/simulate.launch.py` 与 `urdf/simple_2d_robot.urdf.xacro` 中 **`robot_namespace`**。

单机 Gazebo 差速模型在 URDF 内常见链：**`robotN/odom` → `robotN/base_footprint` → `robotN/base_link`**（帧名带 **`robotN/`** 前缀）；**不含 `map`**，与 §1 中「整栈」图的区别见上节。

### 8.3 系统依赖（ROS 2 Foxy 示例）

```bash
sudo apt-get update
sudo apt-get install -y ros-foxy-gazebo-ros-pkgs ros-foxy-xacro ros-foxy-robot-state-publisher
```

保存栅格地图（可选）：

```bash
sudo apt-get install -y ros-foxy-nav2-map-server
```

从源码编译 `slam_toolbox` 时，请使用与发行版匹配的分支（如 Foxy：**`foxy-devel`**），并安装 `libceres-dev`、`ros-foxy-bondcpp` 等。

### 8.4 故障排除

- **`gzserver` 立刻退出（exit code 255）**  
  可先结束残留进程再启动：

  ```bash
  pkill -9 gzserver || true
  ros2 launch simulate simulate.launch.py
  ```

- **`robot_description` YAML 解析报错（Foxy）**  
  launch 已对 `robot_description` 使用 `ParameterValue(..., value_type=str)`；自行改写时请保留。

### 8.5 启动（无头）

主入口 **`simulate.launch.py`**：`robot_state_publisher` 与 `spawn_entity` 在 **`PushRosNamespace`** 下；**Gazebo** 进程全局；第二台车不重起 `gzserver` 时用 **`start_gazebo:=false namespace:=<其他>`**。

`params/launch/simulate/` 与包内 `launch/` 同步，由 `system` 包安装到 **`bringup_launch/simulate/`**。

```bash
cd /path/to/openDelivery
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch simulate simulate.launch.py
# ros2 launch simulate simulate.launch.py namespace:=robot2
# ros2 launch system bringup_launch/simulate/simulate.launch.py
ros2 launch simulate test.launch.py
```

`fake_pub` 不属于本包：见 **`params/launch/fake/fake_pub.launch.py`**（`bringup_launch/fake/fake_pub.launch.py`）。

### 8.6 联动 slam_toolbox 建图 / 定位

```bash
ros2 launch slam_bringup mapping.launch.py
# ros2 run nav2_map_server map_saver_cli -f /tmp/sim_map
# ros2 launch slam_bringup localization.launch.py map_file:=/tmp/sim_map.yaml
```

---

## 9. 与 Web 栈的关系

仓库根的 `web/`、`backend/` 通过 TF 与话题与机器人交互；演示可用 `fake/scripts/fake_pub.py`。详见 **`../README.md`** 与 **`start_web_stack.sh`**。

---

## 10. 其它文档

- 工作区总览：`../README.md`
- 上游 **`slam_toolbox`** 包内说明：`slam/slam_toolbox/README.md`（第三方）
