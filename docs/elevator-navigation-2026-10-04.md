# 电梯通行与已有机器人复用（2026-10-04）

## 电梯门口停滞原因与改动

最近 robot2 的跨楼层任务在 test_101 进梯子任务失败：候梯和呼梯已完成，
随后报告 `Nav2 goal made no progress for 60.0s`。控制器日志同时出现
`PathDistCritic` 找不到局部 costmap 内可用路径点和无有效轨迹。

旧配置使用半径 0.28 m 的圆形，再加默认 0.01 m 外形余量，相当于 0.58 m
直径。test_101 地图门口的可通行宽度约 0.55 m，圆形把本来能通过的门口封死。
这与软膨胀成本的权重不同；仅降低膨胀半径不能修复过大的硬碰撞外形。

局部和全局 costmap 改用同一个矩形外形，边界为 x=-0.21～0.26 m、
y=-0.16～0.16 m，额外保留 0.01 m 余量。矩形覆盖车壳、轮子、前置激光与
摄像头碰撞体；几何测试实际展开 Xacro 并经过 Gazebo SDF 转换，逐项核对
全部碰撞体的变换后包围盒。

DWB 保留 `BaseObstacle` 并加入 `ObstacleFootprint`，同时检查中心和完整旋转
外形。障碍层、未知区域策略、0.55 m 软膨胀半径、速度限制和到达容差不变。
Foxy 的实现分别见 [BaseObstacle](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_critics/src/base_obstacle.cpp)
和 [ObstacleFootprint](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp)。

## OpenClaw 机器人选择

选择仍由 AI 根据实时快照决定，后端只校验和执行计划：

- 未点名机器人时优先使用在线 ready/idle 机器人。
- 没有合适的在线机器人时，选择快照内已有的离线机器人，先 `startup_sim` 再导航。
- 浏览器选中项只在适合的已有机器人之间作为偏好。
- 空快照需要澄清使用哪个已有机器人，不自动编造新编号。创建机器人需要明确请求。
- 明确指定的离线机器人仍先上线；回复遵循当前提问语言。

桥接提示与运行 skill 同步采用上述规则。真实模型的三个隔离规划请求拦截了
执行线程，未启动机器人：离线 robot1 的中文请求返回先上线 robot1 再执行两站导航；
空快照的英文请求返回澄清和空 actions；在线 robot2 与离线 robot1 同时存在时，
英文请求直接选择 robot2 导航，不包含上线步骤。原始模型输出中的 actions 已复核。

## 验证与生效范围

后端 unittest 198 项通过；导航执行器、配置、Gazebo 几何与仿真电梯 pytest
53 项通过。`nav_bringup` 已成功构建。

显式运行 `src/navigation/navigation_tasks/test/nav2_elevator_check.py`，在隔离
ROS domain 94 中使用真实 Nav2、四个项目地图和软件差速运动模型，逐层执行
候梯点到梯内点、梯内转向及返回候梯点。首次运行的 8 个任务均为 Finished，
满足原目标 0.10 m 位置和 0.15 rad 朝向容差，全程未检测到带余量矩形与障碍格相交。
输出保存在 `/tmp/od-elevator-check`，不提交包含运行时环境信息的原始日志。

首次运行在任务完成后的 ROS 清理阶段异常退出。检查脚本随后改为独立持有并
显式关闭 executor，且只向 launch 发送一次 SIGINT。复测能正常清理并返回
失败退出码，但 test_102 出梯在约 70 s 后报告 `Nav2 goal status=6`，未到达
目标；日志包含 `send_goal failed` 和无有效轨迹。因而当前证据证明过大的圆形
外形问题已改正，尚不能证明所有楼层的重复通行稳定性。该间歇失败仍需排查，
脚本保留严格的失败判定，不把任务失败当作通过。

该检查不注册机器人，也不向生产 ROS 域发送命令。它验证地图与 Nav2 闭环，
尚不代表完整 Gazebo 跨层流程或真实底盘测试。已有导航进程需重新启动才能
读取新的 footprint 和 DWB critics；后端进程需重载才能读取新的桥接提示。
