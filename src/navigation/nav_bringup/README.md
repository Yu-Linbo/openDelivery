# nav_bringup

**仅导航**：规划、控制、恢复、`bt_navigator`、`waypoint_follower`。不包含 `map_server` / `AMCL`；**TF 由 SLAM** 提供。

## 栅格地图话题（不经过 `/map` 中转）

Nav2 全局代价地图的 **static_layer** 运行在独立节点 `/<robot>/global_costmap/global_costmap` 上，launch 里对 `planner_server` 的 **remap** 不会作用到该节点。此处通过参数 **`map_topic`**（模板占位符 `@@OCC_GRID_TOPIC@@`，由 `stack.launch.py` 按 `grid_mode` 写成 `/<robot>/map` 或 `/<robot>/mapping`）订阅栅格；`remap ("map", …)` 仍保留以便兼容仅含 `map` 相对名的组件。

| `stack.launch.py` 参数 `grid_mode` | 订阅的 OccupancyGrid |
|-------------------------------------|----------------------|
| `localize`（默认） | **`/<robot_name>/map`** |
| `mapping` | **`/<robot_name>/mapping`** |

示例：

```bash
ros2 launch nav_bringup stack.launch.py robot_name:=robot2 use_sim_time:=true grid_mode:=mapping
ros2 launch nav_bringup stack.launch.py robot_name:=robot2 use_sim_time:=true grid_mode:=localize
```

`start_heartbeat_slam_nav.sh` 已按模式传入对应 `grid_mode`。

## 依赖

Nav2 核心（Foxy 示例，与 `navigation_launch.py` 所需一致）：

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

`ros-foxy-nav2-behavior-tree` 多为 `bt_navigator` 的依赖；若缺省会导致行为树插件 `.so` 无法加载。

### 参数文件与命名空间

临时 `nav2` 参数文件必须是 **顶层为节点名**（`bt_navigator:`、`controller_server:`、…），由 `RewrittenYaml` 再包一层 `<namespace>/`。不要在生成该文件时额外再嵌套一层 `<robot_name>/`，否则参数落不到实际节点上（代价地图会退回默认层类型，`bt_navigator` 可能报缺少 BT 插件库）。

## 动作接口

- `/<robot_name>/navigate_to_pose`
- `/<robot_name>/follow_waypoints`
