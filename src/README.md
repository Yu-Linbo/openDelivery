# OpenDelivery `src` 功能包说明

本目录为 **ROS 2** 工作区源码根（与历史 **ROS 1 / catkin**、`ud_*` 命名工程无关）。当前路线：

- **SLAM**：`slam_toolbox`（建图 / 定位）
- **导航**：`nav2` + `nav2_map_server`（规划与加载地图，按需接入）
- **不再使用**：RTAB-Map

---

## 1. 顶层模块目录（规划占位）

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

## 2. 已有 ROS 2 包（当前仓库内）

| 包名 | 路径 | 说明 |
|------|------|------|
| **simulate** | `simulate/simulate/` | Gazebo 无头仿真：差速、`/scan`、`/imu/data`、`/odom`、前向相机；`simulate/` 为容器目录，包在子目录中 |
| **slam_bringup** | `slam/slam_bringup/` | `slam_toolbox` 建图 / 定位 launch 与参数 |
| **slam_toolbox** | `slam/slam_toolbox/` | 上游源码（通常 **`foxy-devel`** 分支，与 ROS 2 Foxy 对齐） |

### 非 colcon 包（脚本）

| 内容 | 路径 | 说明 |
|------|------|------|
| **fake 演示** | `fake/scripts/` | 如 `fake_pub.py`，用于 Web 联调演示轨迹与话题 |

---

## 3. 构建（`colcon`）

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

## 4. 运行流程（简版）

### 4.1 仿真 + SLAM 建图

```bash
# 终端 1：无头 Gazebo（若 gzserver 异常退出可先：pkill -9 gzserver）
ros2 launch simulate headless_sim.launch.py

# 终端 2（已 source install）
ros2 launch slam_bringup mapping.launch.py
```

保存地图（需安装 `nav2_map_server` 等）：

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/sim_map
```

### 4.2 定位模式（已有地图）

```bash
ros2 launch slam_bringup localization.launch.py map_file:=/tmp/sim_map.yaml
```

### 4.3 真机/实车

在 `driver/`、`system/` 中逐步接入传感器与底盘 launch，再与 `slam_bringup` 或 `navigation/` 串联（具体 launch 待后续补充）。

---

## 5. 与 Web 栈的关系

仓库根目录的 `web/`、`backend/` 通过 TF 与话题与机器人交互；演示可用 `fake/scripts/fake_pub.py`。详见仓库根 `README.md` 与 `start_web_stack.sh`。

---

## 6. 文档索引

- 工作区总览：`../README.md`
- 仿真目录说明：`simulate/README.md`
- 仿真包细节：`simulate/simulate/README.md`
