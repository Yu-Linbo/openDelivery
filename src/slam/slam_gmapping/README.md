# OpenDelivery SLAM / 定位接口

建图和定位互斥运行，由 `/<robot>/slam/lifecycle_manager` 统一切换
`mapping|localize|inactive`。目录中的 GMapping 源码仅作历史兼容，默认建图后端已换为
slam_toolbox。

## 建图：slam_toolbox

| 方向 | 接口类别 | 接口 | 类型 | 用途 |
| --- | --- | --- | --- | --- |
| 输入 | Topic | `/robot2/scan_2d` | `sensor_msgs/msg/LaserScan` | 激光扫描 |
| 输入 | TF | `/tf`、`/tf_static` | `robot2/odom ← robot2/base_footprint ← laser` | 里程计与雷达外参 |
| 输出 | Topic | `/robot2/mapping` | `nav_msgs/msg/OccupancyGrid` | 实时建图栅格 |
| 输出 | TF | `/tf` | `map → robot2/odom` | Ceres pose-graph / scan-matching 校正 |
| 保存 | Service | `/robot2/slam/serialize_map` | `slam_toolbox/srv/SerializePoseGraph` | Web 保存地图时生成 `.posegraph` 与 `.data` |

## 定位方法

具体后端由 `backend/data/robot_status_last.json` 中每台机器人的
`localization_method` 选择。

### `slam_toolbox`

加载与地图 YAML 同基名的 `.posegraph` 和 `.data`，以滚动激光窗口进行弹性
pose-graph 定位；接收 `/robot2/initial`，发布 `map→robot2/odom`。辅助 TF 位姿桥
将结果同步为 `/robot2/amcl_pose`，供现有 `health_monitor` 判定 ready。

### `gazebo_ground_truth`

仅用于仿真，实现在相邻包 `src/slam/gazebo_ground_truth_localization`。节点读取
`/gazebo/model_states` 的绝对 `world→base`，依据地图 YAML/PGM 与 world 中楼层
模型位姿换算为地图坐标，再与 `odom→base` 合成严格一致的 `map→odom`；同时发布
近零协方差的 `/robot2/amcl_pose`。它订阅 `/robot2/initial`，姿态重定位会重设
`map→world` 对齐而不传送 Gazebo 模型。该模式不使用激光估计，不能部署到真机。

### `amcl`

兼容只有 `.pgm/.yaml`、尚未生成 slam_toolbox pose graph 的旧地图。接口保持原样：
订阅 `/robot2/scan_2d`、`/robot2/map`、`/robot2/initial`，发布
`/robot2/amcl_pose` 与 `map→robot2/odom`。

## 辅助重定位

`slam/relocalization` 提供 `/robot2/record_relocalization` 和
`/robot2/relocalize` 服务，并将匹配成功的修正位姿发布到 `/robot2/initial`。
记录按 `RobotStatus.current_map` 隔离保存。
