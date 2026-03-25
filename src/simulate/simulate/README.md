# simulate（功能包）

本包位于 **`src/simulate/simulate/`**：外层 `src/simulate/` 仅作仿真类包的目录容器，见 [`../README.md`](../README.md)。

最小 2D 仿真机器人（Gazebo 无头）：

- `/scan`：2D 雷达
- `/imu/data`：IMU
- `/odom`：差速里程计
- `/camera/image_raw`：前向相机
- `tf`：`odom -> base_footprint -> base_link`（差速插件）+ 各传感器相对 `base_link` 的静态外参（URDF）

## 系统依赖（ROS 2 Foxy 示例）

仿真与 xacro 处理需要已安装：

```bash
sudo apt-get update
sudo apt-get install -y ros-foxy-gazebo-ros-pkgs ros-foxy-xacro ros-foxy-robot-state-publisher
```

保存栅格地图（可选）：

```bash
sudo apt-get install -y ros-foxy-nav2-map-server
```

从源码编译 `slam_toolbox` 时，请使用与发行版匹配的分支，例如 Foxy 使用 **`foxy-devel`**，并安装 `libceres-dev`、`ros-foxy-bondcpp` 等构建依赖。

## 故障排除

- **`gzserver` 立刻退出（exit code 255）**  
  常见原因是上一次仿真的 `gzserver` 仍在运行或 master 端口被占用。可先结束残留进程再启动：

  ```bash
  pkill -9 gzserver || true
  ros2 launch simulate headless_sim.launch.py
  ```

- **`robot_description` YAML 解析报错（Foxy）**  
  本仓库的 launch 已对 `robot_description` 使用 `ParameterValue(..., value_type=str)`；若你自行改写 launch，请保留该写法。

## 启动（无头）

```bash
cd /home/ubuntu/project_openclaw/openDelivery
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch simulate headless_sim.launch.py
```

## 联动 slam_toolbox 建图

新终端：

```bash
cd /home/ubuntu/project_openclaw/openDelivery
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

ros2 launch slam_bringup mapping.launch.py
```

可选保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/sim_map
```

## 用已保存地图做定位

```bash
ros2 launch slam_bringup localization.launch.py map_file:=/tmp/sim_map.yaml
```
