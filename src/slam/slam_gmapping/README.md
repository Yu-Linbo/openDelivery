# OpenDelivery SLAM 接口

建图和定位互斥运行。Topic 用于连续数据流；Service 只用于一次性命令或状态查询。

## 建图：GMapping

| 方向 | 接口类别 | 接口 | 类型 | 用途 |
| --- | --- | --- | --- | --- |
| 输入 | Topic | /robot2/scan_2d | sensor_msgs/msg/LaserScan | 激光扫描 |
| 输入 | TF | /tf、/tf_static | robot2/odom <- robot2/base_footprint <- laser_frame | 由扫描时间查询机器人和雷达位姿；不直接订阅 odom topic |
| 输出 | Topic | /robot2/mapping | nav_msgs/msg/OccupancyGrid | 实时建图栅格 |
| 输出 | Topic | /robot2/slam/map_metadata、/robot2/slam/entropy | MapMetaData、Float64 | 地图元数据、粒子熵 |
| 输出 | TF | /tf | map -> robot2/odom | SLAM 位姿校正 |
| 控制 | Service | 无自定义 service | - | 参数服务由 rclcpp 自动提供；建图控制由上层 lifecycle manager 负责 |

## 定位：AMCL

| 方向 | 接口类别 | 接口 | 类型 | 用途 |
| --- | --- | --- | --- | --- |
| 输入 | Topic | /robot2/scan_2d | sensor_msgs/msg/LaserScan | 用实时激光匹配静态地图 |
| 输入 | Topic | /robot2/map | nav_msgs/msg/OccupancyGrid | map_server 提供的已保存地图；订阅者应使用 Transient Local QoS |
| 输入 | Topic | /robot2/initial | geometry_msgs/msg/PoseWithCovarianceStamped | 重定位初始位姿；由 AMCL 的 initialpose 重映射而来 |
| 输入 | TF | /tf、/tf_static | robot2/odom <- robot2/base_footprint | 里程计位姿与机器人坐标关系 |
| 输出 | Topic | /robot2/amcl_pose | geometry_msgs/msg/PoseWithCovarianceStamped | 估计位姿与协方差；health_monitor 依据协方差更新 robot_status 为 ready |
| 输出 | Topic | /robot2/slam/particle_cloud、/robot2/slam/particlecloud | geometry_msgs/msg/PoseArray | 粒子滤波可视化 |
| 输出 | TF | /tf | map -> robot2/odom | 定位校正 |
| 控制 | Service | lifecycle、参数 service | - | configure、activate 等由 /robot2/slam/lifecycle_manager 统一管理 |

## 辅助重定位

`slam/relocalization` 提供 `/robot2/record_relocalization` 和 `/robot2/relocalize` 服务，
并将匹配成功的修正位姿发布到 `/robot2/initial`。记录按 `RobotStatus.current_map`
隔离保存；模式 1 优先修正传入点位，评分不足时自动回退到遍历历史 scan 的模式 0。
