# OpenDelivery `src` 功能包说明

本目录为 **ROS 2** 工作区源码根（与历史 **ROS 1 / catkin**、`ud_*` 命名工程无关）。当前路线：

- **SLAM**：GMapping + AMCL（建图 / 定位）
- **导航**：`nav2` + `nav2_map_server`（规划与加载地图，按需接入）
- **不再使用**：RTAB-Map

---

## 1. ROS 功能包一览

以下为 `src/` 下 **`colcon build` 可构建** 的 ament 包，按目录分组；路径均相对于 `src/`。

### 1.1 公共 `common/`

| 包名 | 路径 | 功能简述 |
|------|------|----------|
| **custom_msgs_srvs** | `common/custom_msgs_srvs/` | 项目自定义接口：`RobotStatus`（心跳/地图名/任务状态）、`SetHeartbeatParams`、`LocalizeNavCommand` 等 msg/srv，供 Web 后端与各节点共用。 |

### 1.2 仿真 `simulate/`

| 包名 | 路径 | 功能简述 |
|------|------|----------|
| **simulate** | `simulate/simulate/` | Gazebo 无头 **2D 差速车**仿真：URDF、世界、`robot_state_publisher`、实体生成；插件发布 `/<R>/scan_2d`、`odom`、`cmd_vel`、IMU、前视相机 `/<R>/front_camera/*`、前向下倾相机 `/<R>/front_down_camera/*`；**仅维护 `odom` 树，不发布 `map`**。`simulate/` 为容器目录，包在子目录中。 |

### 1.3 SLAM `slam/`

| 包名 | 路径 | 功能简述 |
|------|------|----------|
| **slam_gmapping** | `slam/slam_gmapping/slam_gmapping/` | GMapping ROS 2 节点；建图时发布 OccupancyGrid 和 `map→odom`。 |
| **openslam_gmapping** | `slam/slam_gmapping/openslam_gmapping/` | OpenSLAM GMapping 算法库。 |

SLAM 参数文件：`system/manager/config/mapper_params.yaml`、`localization_params.yaml`。

### 1.4 导航 `navigation/`

| 包名 | 路径 | 功能简述 |
|------|------|----------|
| **nav_bringup** | `navigation/nav_bringup/` | **Nav2 导航栈** launch：`controller_server`、`planner_server`、`recoveries_server`、`bt_navigator`、`waypoint_follower`、`lifecycle_manager_navigation`；全局代价地图按 `grid_mode` 订 `/<R>/map`（定位）或 `/<R>/mapping`（建图）；**不含 `map_server` / AMCL**，TF 与栅格由 SLAM 提供。详见 `navigation/nav_bringup/README.md`。 |

### 1.5 系统 `system/`

| 包名 | 路径 | 功能简述 |
|------|------|----------|
| **heartbeat** | `system/heartbeat/` | **Lifecycle 心跳节点**：周期发布 `/<R>/robot_status`（`RobotStatus`）；服务 `/<R>/set_heartbeat_params` 供 Web / manager 修改地图名、`robot_status`、`task_status` 等。 |
| **manager** | `system/manager/` | **`health_monitor`**、**`task_manager`**、**`stack_lifecycle_manager`**（含 SLAM 参数 `config/*.yaml`）：SLAM 模式切换、Lifecycle 代理、**`/<R>/stack_lifecycle`**。 |
| **log_bag** | `system/log_bag/` | 按机器人 **`robot_log_recorder`**：终端日志 + **rosbag2** 轮转录制（`log_bag/<R>/`），由 `startup.launch.py` 默认拉起。 |
| **system** | `system/system/` | **元包 / 集成层**：安装仓库 `params/` launch、`sim_bringup.sh`（Web 仿真上线唯一入口）、`startup.launch.py`（log_bag + heartbeat）；可选安装 `fake_pub` 演示节点。 |

### 1.6 演示与其它（非独立 ament 包）

| 内容 | 路径 | 功能简述 |
|------|------|----------|
| **fake_pub** | `fake/scripts/fake_pub.py` | 无 Gazebo 时的轻量假位姿/话题，便于 Web 联调；launch 在 `params/launch/fake/`，由 `system` 包安装。 |

### 1.7 规划占位目录（尚无 `package.xml`）

| 目录 | 规划用途 |
|------|----------|
| `algorithm/` | 算法与业务逻辑 |
| `driver/` | 传感器、底盘等驱动 |
| `calibration/` | 标定 |
| `perception/` | 感知 |
| `tool/` | 工具脚本 |

空目录以 `.gitkeep` 占位，便于 Git 跟踪。

### 1.8 Web 后端桥（不在 `src/`，与上表协同）

| 组件 | 路径 | 功能简述 |
|------|------|----------|
| **ros_tf_bridge** | `backend/ros_tf_bridge.py` | ROS 2 节点：订阅 `/tf`、动态发现 `/*/robot_status`、激光/路径/`/<R>/mapping` 等，缓存后供 **HTTP API**（位姿、建图视图、切图命令）。 |

---

## 2. TF 树（与 `backend/ros_tf_bridge.py` 一致）

Web 后端从共享的 `/tf`、`/tf_static` 做 `map` → 各机器人 `base_link` 查询。约定典型结构为：

```
map
 ├── robot1/odom → robot1/base_link
 └── robot2/odom → robot2/base_link
```

`→` 表示从父到子的链。URDF 中常见 **`robotN/odom` → `robotN/base_footprint` → `robotN/base_link`**；桥在说明里常写到 `base_link` 为止。多机时帧名带 **`robotN/`** 前缀，与 `/*/robot_status` 动态发现一致。

**与仿真的分工（两棵 TF 树）**：**`map`** 及 **`map` → `odom`** 由 **SLAM**（`stack_lifecycle_manager` + GMapping + AMCL）维护；**Gazebo `simulate.launch.py` 只发布里程计树 **`robotN/odom` → … → `robotN/base_link`**，**不**发布 `map`，也不在仿真里做 `map→odom` 静态 TF。联调全栈时再同时起 SLAM 与仿真，由 SLAM 把 `map` 接到各机器人 `odom`。

---

## 3. 顶层模块目录（规划占位）

职责划分与 §1.7 一致；已有实现见 **§1 ROS 功能包一览**。

---

## 4. 节点命名（约定）

在 **`/<robot_name>/`** 下：单节点包用 **包名** 作为节点名（如 `/robot2/heartbeat`）；SLAM worker 在 **`/<robot_name>/slam/mapping`** 或 **`localizing`**；栈管理为 **`/<robot_name>/slam/lifecycle_manager`**。

---

## 5. 运行时各节点与作用（概览）

下文用 **`<R>`** 表示机器人命名空间（与 `robot_name` / `namespace` 一致，如 `robot2`）。**是否出现**取决于当前 launch（仅仿真、仅心跳、全栈等）。

### 5.1 仿真 `simulate`（`simulate.launch.py`）

| 节点 / 进程 | 作用 |
|-------------|------|
| **`gzserver`**（经 `gazebo_ros` 启动） | Gazebo 世界与物理仿真 |
| **`/<R>/simulate/robot_state_publisher`** | 订阅 Gazebo 发布的 `joint_states`，按 URDF 发布 **`/<R>/odom` → `base_footprint` → `base_link`** 等 TF |
| **`/<R>/simulate/spawn_entity`** | 将模型按 `robot_description` 生成到 Gazebo（通常很快结束） |
| **Gazebo 插件（随 URDF）** | 差速底盘（`cmd_vel`）、**`/<R>/scan_2d`**、IMU、相机等 |

### 5.2 心跳与管理器（`manager.launch.py`）

| 节点 | 作用 |
|------|------|
| **`/<R>/heartbeat`**（Lifecycle） | 周期发布 **`/<R>/robot_status`**（`RobotStatus`）；提供 **`/<R>/set_heartbeat_params`** 服务 |
| **`/<R>/health_monitor`** | 监视必要节点与可选定位话题，推进 **`robot_status`** |
| **`/<R>/task_manager`** | **`set_robot_task`**、**`localize_nav_command`** 等任务接口 |
| **`/<R>/slam/lifecycle_manager`** | **栈生命周期**：SLAM 模式切换；代理 **`set_stack_lifecycle_transition`** 驱动 heartbeat / Nav2 等 Lifecycle 节点；发布 **`/<R>/stack_lifecycle`** |

### 5.3 SLAM（由 `stack_lifecycle_manager` 管理）

| 节点 | 作用 |
|------|------|
| **`/<R>/slam/mapping`** | **建图**（`slam_gmapping`）：发布 **`/<R>/mapping`** 与 **`map`→`/<R>/odom` TF |
| **`/<R>/slam/localizing`** | **定位**（`nav2_amcl`）：发布 **`/<R>/map`** 与定位 TF；与 mapping **二选一** |

切换示例：

```bash
ros2 service call /robot2/slam/set_stack_lifecycle_transition \
  custom_msgs_srvs/srv/SetStackLifecycleTransition "{node_name: slam, transition: mapping}"
```

### 5.4 导航 `nav_bringup`（`stack.launch.py` → `navigation_namespaced.launch.py`）

均在 **`/<R>/`** 下；全局代价地图通过 remap 订阅 **`/<R>/map`** 或 **`/<R>/mapping`**（由 `grid_mode` 决定）。

| 节点 | 作用 |
|------|------|
| **`/<R>/controller_server`** | 局部规划 / DWB，输出 `cmd_vel` |
| **`/<R>/planner_server`** | 全局路径（NavFn 等） |
| **`/<R>/recoveries_server`** | 脱困（旋转、后退、等待等） |
| **`/<R>/bt_navigator`** | 行为树导航（NavigateToPose 等） |
| **`/<R>/waypoint_follower`** | 多点巡逻 / 航点跟随 |
| **`/<R>/lifecycle_manager_navigation`** | 统一管理上述 Nav2 节点的 lifecycle |

### 5.5 演示 `fake_pub`（`params/launch/fake/fake_pub.launch.py`）

| 节点 | 作用 |
|------|------|
| **`/<R>/fake`** | 轻量假数据（位姿/话题），便于无 Gazebo 时与 Web 联调 |

### 5.6 Web 后端桥（非 `src` 内 ROS 包）

| 组件 | 作用 |
|------|------|
| **`backend/ros_tf_bridge.py`（ROS 2 节点）** | 聚合多机 TF、`/*/robot_status` 发现、激光/路径/topdown 相机等供 **HTTP** 使用；与上表节点通过话题/服务协同 |

---

## 6. 构建（`colcon`）

在**工作区根** `openDelivery/`（与 `src/`、`install/` 同级）执行：

```bash
cd /path/to/openDelivery
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to simulate manager
# 含心跳、栈管理、Nav2 bringup、log_bag（依赖会一并解析）：
# colcon build --symlink-install --packages-up-to heartbeat nav_bringup log_bag
# GMapping 源码与 manager：
# colcon build --symlink-install --packages-up-to slam_gmapping manager
source install/setup.bash
```

定位依赖系统包 `ros-foxy-nav2-amcl`；仿真另需：`ros-foxy-gazebo-ros-pkgs`、`ros-foxy-xacro`、`ros-foxy-robot-state-publisher`。

---

## 7. 运行流程（简版）

### 7.0 一键：建图 + 心跳 + 导航（推荐）

在仓库根目录**已** `source install/setup.bash`，且 **终端 1** 已按与 `robot_name` 一致的命名空间起好仿真或真机（提供 `/clock`、`/<robot>/odom`、`/<robot>/scan_2d` 等）：

**建图模式（示例 `robot2`）：**

```bash
cd /path/to/openDelivery
./start_heartbeat_slam_nav.sh robot2 mapping
```

等价拉起：`heartbeat` + `manager`（**stack_lifecycle_manager** 按磁盘缓存设 `initial_slam_mode:=mapping`）+ `nav_bringup`（`grid_mode:=mapping`）。

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
ros2 launch manager manager.launch.py namespace:=robot2 initial_slam_mode:=mapping
```

保存地图（需安装 `nav2_map_server` 等）：

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/sim_map
```

### 7.2 定位模式（已有地图）

```bash
ros2 launch manager manager.launch.py namespace:=robot2 initial_slam_mode:=localize map_file:=/tmp/sim_map.yaml
```

### 7.3 真机/实车

在 `driver/`、`system/` 中逐步接入传感器与底盘 launch，再与 `stack_lifecycle_manager` / `navigation/` 串联（具体 launch 待后续补充）。

---

## 8. 仿真包 `simulate`（路径 `simulate/simulate/`）

### 8.1 目录说明

- **`src/simulate/`**：容器目录，只放仿真相关 ament 包，本层不直接放 `package.xml`。
- **包本体**：`src/simulate/simulate/`（包名 **`simulate`**）。新增仿真包时可在 `src/simulate/<包名>/` 下并列创建。

### 8.2 话题与 TF（Gazebo）

命名空间由 launch **`namespace`** 决定（默认随 **`robot_name`**）。插件话题在 **`/<namespace>/...`**（激光为 **`scan_2d`**，另有 `imu/data`、`cmd_vel`、`odom`、`camera/...`、前向下倾相机 `front_down_camera/...`），与 **`PushRosNamespace`** 一致；详见 `simulate/launch/simulate.launch.py` 与 `urdf/simple_2d_robot.urdf.xacro` 中 **`robot_namespace`**。

单机 Gazebo 差速模型在 URDF 内常见链：**`robotN/odom` → `robotN/base_footprint` → `robotN/base_link`**（帧名带 **`robotN/`** 前缀）；**不含 `map`**，与 §2「整栈」TF 图的区别见上节。

### 8.3 系统依赖（ROS 2 Foxy 示例）

```bash
sudo apt-get update
sudo apt-get install -y ros-foxy-gazebo-ros-pkgs ros-foxy-xacro ros-foxy-robot-state-publisher
```

保存栅格地图（可选）：

```bash
sudo apt-get install -y ros-foxy-nav2-map-server
```

GMapping 源码已包含在工作区；定位使用系统包 `ros-foxy-nav2-amcl`。

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

主入口 **`simulate.launch.py`**：`robot_state_publisher` 与 `spawn_entity` 在 **`PushRosNamespace`** 下。Web 仿真上线由后端单独托管全局 Gazebo/Xvfb 世界；每个机器人栈固定使用 `start_gazebo:=false`，只生成自己的唯一 entity，并使用持久化且互不重叠的出生位姿。机器人离线仅删除本实体，不停止共享 Gazebo。

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

### 8.6 联动 GMapping + AMCL 建图 / 定位

```bash
ros2 launch manager manager.launch.py namespace:=robot2 initial_slam_mode:=mapping
```

---

## 9. 与 Web 栈的关系

仓库根的 `web/`、`backend/` 通过 TF 与话题与机器人交互；演示可用 `fake/scripts/fake_pub.py`。详见 **`../README.md`** 与 **`start_web_stack.sh`**。

---

## 10. 其它文档

- 工作区总览：`../README.md`
- 上游 **GMapping + AMCL** 包内说明：`slam/slam_gmapping/README.md`（第三方）
