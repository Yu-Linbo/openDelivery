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

## 3. 已有 ROS 2 包（当前仓库内）

| 包名 | 路径 | 说明 |
|------|------|------|
| **simulate** | `simulate/simulate/` | Gazebo 无头仿真；`simulate/` 为容器目录，包在子目录中 |
| **slam_bringup** | `slam/slam_bringup/` | `slam_toolbox` 建图 / 定位 launch 与参数 |
| **slam_toolbox** | `slam/slam_toolbox/` | 上游源码（通常 **`foxy-devel`** 分支，与 ROS 2 Foxy 对齐） |

### 非 colcon 包（脚本）

| 内容 | 路径 | 说明 |
|------|------|------|
| **fake 演示** | `fake/scripts/` | 如 `fake_pub.py`，用于 Web 联调演示轨迹与话题 |

---

## 4. 构建（`colcon`）

在**工作区根** `openDelivery/`（与 `src/`、`install/` 同级）执行：

```bash
cd /path/to/openDelivery
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to simulate slam_bringup
# 若包含 slam_toolbox 源码：
# colcon build --symlink-install --packages-select slam_toolbox simulate slam_bringup
source install/setup.bash
```

**源码构建 `slam_toolbox` 时**常见系统依赖（Foxy 示例）：`libceres-dev`、`ros-foxy-bondcpp`；仿真另需：`ros-foxy-gazebo-ros-pkgs`、`ros-foxy-xacro`、`ros-foxy-robot-state-publisher`。

---

## 5. 运行流程（简版）

### 5.1 仿真 + SLAM 建图

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

### 5.2 定位模式（已有地图）

```bash
ros2 launch slam_bringup localization.launch.py map_file:=/tmp/sim_map.yaml
```

### 5.3 真机/实车

在 `driver/`、`system/` 中逐步接入传感器与底盘 launch，再与 `slam_bringup` 或 `navigation/` 串联（具体 launch 待后续补充）。

---

## 6. 仿真包 `simulate`（路径 `simulate/simulate/`）

### 6.1 目录说明

- **`src/simulate/`**：容器目录，只放仿真相关 ament 包，本层不直接放 `package.xml`。
- **包本体**：`src/simulate/simulate/`（包名 **`simulate`**）。新增仿真包时可在 `src/simulate/<包名>/` 下并列创建。

### 6.2 话题与 TF（Gazebo）

命名空间由 launch **`namespace`** 决定（默认随 **`robot_name`**）。插件话题在 **`/<namespace>/...`**（如 `scan`、`imu/data`、`cmd_vel`、`odom`、`camera/...`），与 **`PushRosNamespace`** 一致；详见 `simulate/launch/simulate.launch.py` 与 `urdf/simple_2d_robot.urdf.xacro` 中 **`robot_namespace`**。

单机 Gazebo 差速模型在 URDF 内常见链：**`robotN/odom` → `robotN/base_footprint` → `robotN/base_link`**（帧名带 **`robotN/`** 前缀）；**不含 `map`**，与 §1 中「整栈」图的区别见上节。

### 6.3 系统依赖（ROS 2 Foxy 示例）

```bash
sudo apt-get update
sudo apt-get install -y ros-foxy-gazebo-ros-pkgs ros-foxy-xacro ros-foxy-robot-state-publisher
```

保存栅格地图（可选）：

```bash
sudo apt-get install -y ros-foxy-nav2-map-server
```

从源码编译 `slam_toolbox` 时，请使用与发行版匹配的分支（如 Foxy：**`foxy-devel`**），并安装 `libceres-dev`、`ros-foxy-bondcpp` 等。

### 6.4 故障排除

- **`gzserver` 立刻退出（exit code 255）**  
  可先结束残留进程再启动：

  ```bash
  pkill -9 gzserver || true
  ros2 launch simulate simulate.launch.py
  ```

- **`robot_description` YAML 解析报错（Foxy）**  
  launch 已对 `robot_description` 使用 `ParameterValue(..., value_type=str)`；自行改写时请保留。

### 6.5 启动（无头）

主入口 **`simulate.launch.py`**：`robot_state_publisher` 与 `spawn_entity` 在 **`PushRosNamespace`** 下；**Gazebo** 进程全局；第二台车不重起 `gzserver` 时用 **`start_gazebo:=false namespace:=<其他>`**。

`params/launch/simulate/` 与包内 `launch/` 同步，由 `open_delivery_system` 安装到 **`bringup_launch/simulate/`**。

```bash
cd /path/to/openDelivery
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch simulate simulate.launch.py
# ros2 launch simulate simulate.launch.py namespace:=robot2
# ros2 launch open_delivery_system bringup_launch/simulate/simulate.launch.py
ros2 launch simulate test.launch.py
```

`fake_pub` 不属于本包：见 **`params/launch/fake/fake_pub.launch.py`**（`bringup_launch/fake/fake_pub.launch.py`）。

### 6.6 联动 slam_toolbox 建图 / 定位

```bash
ros2 launch slam_bringup mapping.launch.py
# ros2 run nav2_map_server map_saver_cli -f /tmp/sim_map
# ros2 launch slam_bringup localization.launch.py map_file:=/tmp/sim_map.yaml
```

---

## 7. 与 Web 栈的关系

仓库根的 `web/`、`backend/` 通过 TF 与话题与机器人交互；演示可用 `fake/scripts/fake_pub.py`。详见 **`../README.md`** 与 **`start_web_stack.sh`**。

---

## 8. 其它文档

- 工作区总览：`../README.md`
- 上游 **`slam_toolbox`** 包内说明：`slam/slam_toolbox/README.md`（第三方）
