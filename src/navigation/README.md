# Navigation（OpenDelivery）

## 本仓库内包

| 包名 | 说明 |
|------|------|
| `nav_bringup` | **仅导航**：Nav2 控制器、规划器、恢复、`bt_navigator`、`waypoint_follower`。栅格通过 `grid_mode` 直连 `/<robot>/map` 或 `/<robot>/mapping`；TF 由 **SLAM** 提供；无 `map_server` / `AMCL`。见 `nav_bringup/README.md`。 |

## 上游 Nav2 源码（可选）

系统包：`sudo apt install ros-$ROS_DISTRO-nav2-bringup`。

若需改 Nav2 本体：`scripts/clone_navigation2.sh`。

## 启动示例

先启动 `slam_bringup`（或其它发布 `/<robot>/map` 或 `/<robot>/mapping` 的栈），再：

```bash
cd /path/to/openDelivery
source install/setup.bash
ros2 launch nav_bringup stack.launch.py robot_name:=robot2 use_sim_time:=true grid_mode:=mapping
```
