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
外形。未知区域仍按未知处理，0.55 m 软膨胀半径、速度限制和到达容差不变。
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

## 前置激光盲区、动作响应与进程清理

OP1 实际激光位于车体前方 x=0.23 m，只扫描前方 180°。原局部 costmap
只使用激光障碍层，车体后方的已知空地仍可能被当成未知；进梯后的转向因此
全部被 `ObstacleFootprint` 拒绝。局部 costmap 增加当前地图的 StaticLayer，
使用与全局相同的地图话题和 transient-local 开关。地图中的未知区域仍保留，
实时障碍层继续更新动态障碍。闭环检查同步改用实际安装位置、180°视场和
720 束激光，避免用全向激光掩盖这个问题。

Foxy 的 BT navigator 在黑板中硬编码 10 ms 的服务响应超时；YAML 中的
`default_server_timeout` 没有生效。新增项目自有 BT XML，通过受支持的
`server_timeout="2000"` 输入端口给每个动作和清除 costmap 服务设置超时。
保留原有重规划、旋转和等待恢复流程，两个 launch 入口均加载该 XML。
依据见 [Foxy BT navigator](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_bt_navigator/src/bt_navigator.cpp)
与 [BT action 输入端口](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_behavior_tree/include/nav2_behavior_tree/bt_action_node.hpp)。

原来的导航停止命令只匹配 launch 字符串，可能留下 task_executor 和恢复节点，
同一任务会被多个执行器消费并交错发布 Failed/Finished；还可能杀掉同机器人
但运行在独立 ROS domain 的检查进程。现在按实际可执行文件、完整机器人
namespace、ROS domain 和 PID 启动时间清理导航节点及孤儿子进程，不停止
共享的机器人进程组。仿真启动时的旧进程清理也限定当前 domain。使用 fake
manager 的单元测试显式屏蔽真实进程清理，避免测试影响正在运行的仿真。

## 验证与生效范围

后端 unittest 202 项通过；导航执行器、配置、Gazebo 几何与仿真电梯 pytest
54 项通过，共 256 项。`nav_bringup` 已成功构建。

显式运行 `src/navigation/navigation_tasks/test/nav2_elevator_check.py`，在隔离
ROS domain 94 中使用真实 Nav2、四个项目地图、实际前置半圈激光配置与软件
差速运动模型。test_101～test_104 每层各执行进梯、梯内转向和返回候梯点，
8 个任务全部 Finished，检查进程正常退出（exit 0）。位置误差 0.0956～0.0994 m，
朝向误差 0.1439～0.1477 rad，满足原有 0.10 m / 0.15 rad 容差。
全程逐帧核对带余量矩形与地图障碍格，没有检测到碰撞。
输出保存在 `/tmp/od-elevator-check`，不提交原始运行日志。

实际 Gazebo 的四楼到一楼返回任务已完成：六个子任务均为 Finished，
耗时 55.71 s，末端位置误差 0.0903 m，朝向误差 0.1375 rad。
验证报告为 `/tmp/od-live-elevator/return-report.json`。

完整 AI 执行验证也已通过：中文请求“去四楼电梯候梯点”，浏览器选中已有的
离线 robot2。真实模型自行返回 `startup_sim(robot2)` 和四楼候梯点导航，后端
等待 online + ready 后才开始导航。实际 Gazebo 中六个子任务全部 Finished，
助手 job 为 completed，执行耗时 113.91 s，位置误差 0.0983 m，
朝向误差 0.1411 rad。前后机器人编号均为 robot1、robot2，没有创建新机器人。
报告为 `/tmp/od-live-elevator/report.json`；使用独立验证会话，不重置用户对话。

该次完整验证前，旧 Web 监督进程下的 DDS 接收异常曾导致仿真 ready 但后端
离线；相同环境的独立桥接和备用后端能接收心跳。只重启后端未恢复，完整重启
Web 服务及其仿真进程后恢复。未把该问题归因于碰撞系数，也未采用会导致
本机 ROS 服务不可达的自定义传输配置。本机 Foxy 官方 SHM 清理器还存在
计数器 AttributeError；验证时仅在实例中补齐计数器后运行原算法，清理僵尸
段并保留活跃段。此系统安装问题未改写，不能声称仅后端热重载能恢复所有
历史 DDS 状态。

检查不注册新机器人，也不向真机发命令。仿真结果尚不代表真实底盘验证；
真实机器人仍需验证定位精度、动态障碍、制动距离和狭窄通道表现。
已有导航进程需重新启动才能读取新的地图层、footprint、DWB critics 和 BT XML；
后端进程需重载才能读取新的进程清理逻辑与桥接提示。
