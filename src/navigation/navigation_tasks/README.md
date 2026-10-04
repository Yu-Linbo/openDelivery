# Navigation tasks

`navigation_task_node` 由 `nav_bringup` 启动在 `/<robot>/navigation` 命名空间，提供：

- `task_info`（订阅 `TaskInfo`，可靠、非持久命令流）
- `task_command`（订阅 `TaskCommand`，可靠、非持久命令流）
- `task_status`（发布 `TaskStatus`，transient-local）

节点接受 `navigation`、`patrol` 和 `following`。点到点/巡逻按顺序通过 `NavigateToPose`
执行 poses；following 将 pose 数组组成一个 `nav_msgs/Path` 后调用 `FollowPath`。导航层对巡逻
每一点都报告 `Navigating`，根 task_manager 将索引 1 及之后汇总为 `Patrolling`；following
在导航层报告 `Following`，根层汇总为 `Cleaning`。

命令为 `Pause`、`Resume`、`Terminate`。Pause 取消当前 Nav2 goal 但不推进索引；Resume
重新提交当前工作项；Terminate 取消并进入终止状态。新任务会取消上一任务的活动 goal。

`end_action=back` 时，调用方必须把返回/等待位姿放在最后一个 pose。因此点到点任务在
`waiting` 模式只有一个目标，在 `back` 模式为目标加返回点。当前尚无楼层等待点自动查询和
导航执行器本身只处理同层任务。根 `task_manager` 会按连续 `floor_ids` 将跨楼层任务展开为
本层候梯导航、呼梯、本层进梯导航、乘梯换层、目标层出梯导航和原业务目标；导航执行器收到的
始终是单层子任务。

完整入口链路：

```text
/<robot>/task_info → task_manager → /<robot>/navigation/task_info
  → navigation/task_executor → /<robot>/navigation/navigate_to_pose 或 navigation/follow_path
  → /<robot>/navigation/task_status → task_manager → /<robot>/task_status
```

Nav2、任务执行器和 Web 直接目标接口统一使用 `/<robot>/navigation/navigate_to_pose`。
旧的根命名空间 `/<robot>/navigate_to_pose` 不再是本启动文件的目标接口。
启动或切图时 Nav2 lifecycle 激活可能晚于机器人状态更新。执行器通过参数
`action_server_wait_sec` 等待 action server，默认 15 秒；超时状态会包含实际等待的
action 名称。

ROS 2 Foxy 的 BT action 偶发在 controller 已接收路径后以 `send_goal failed` 中止外层
`NavigateToPose`。执行器仅对 `STATUS_ABORTED` 使用有上限的延迟重试，默认重试 2 次、
间隔 1 秒；Pause、Terminate 或新任务会取消等待中的重试。其他终态仍立即如实上报，避免
真正不可达的目标被无限重试。


## 到达判定与停滞处理

规划器不再用距离目标 0.5 m 内的替代终点：`GridBased.tolerance=0.0`。
DWB 和 Nav2 到达窗口统一为 0.10 m，朝向窗口为 0.15 rad；Nav2 在最终旋转期间
继续检查位置，避免进入窗口后漂出仍报告成功。末端进展检测半径缩小到 0.05 m，
朝向评分点偏移缩小到 0.05 m，软膨胀成本权重从 0.02 调到 0.005，
避免靠墙合法目标的接近收益被软障碍惩罚压过。硬障碍拒绝保留，碰撞外形改为覆盖
全部 Gazebo 碰撞部件的矩形：前端 x=0.26 m、后端 x=-0.21 m、两侧 y=±0.16 m，
另加 0.01 m 余量。局部和全局 costmap 使用同一外形，DWB 同时启用
`BaseObstacle` 与 `ObstacleFootprint`，检查中心及朝向对应的完整外形。
原来的 0.28 m 圆形加余量后直径 0.58 m，会封死约 0.55 m 的电梯门；
软膨胀半径仍为 0.55 m，未通过缩小真实车体或关闭障碍层来过门。
任务执行器在 Nav2 成功后，
通过 `map → <robot>/base_footprint` 的新鲜 TF 独立核对原始目标的位置和朝向。
TF 超过 1 s、缺失或误差超限时，最多等待 2 s 让定位更新，随后按原有有界策略重试，
仍不满足则报告 Failed；巡逻/返回任务只有核对通过才推进到下一个点。

执行器除了响应/反馈超时，还监控实际距离缩短以及目标附近的朝向调整：

| ROS 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `nav2_progress_timeout_sec` | 60 s | 收到反馈但距离/最终朝向长期无改善，取消并有界重试 |
| `nav2_goal_timeout_sec` | 600 s | 每次 goal 的最长执行时间，防止反复重规划无限运行 |
| `arrival_xy_tolerance` | 0.10 m | 对原始目标的位置复核容差 |
| `arrival_yaw_tolerance` | 0.15 rad | 对原始目标的朝向复核容差 |
| `arrival_tf_max_age_sec` | 1 s | 可接受的定位 TF 最大年龄 |
| `arrival_verification_timeout_sec` | 2 s | Nav2 成功后的定位复核等待时间 |

这些执行器参数在启动时读取；修改后需重启导航栈。超时计时及重试使用单调时钟，
不依赖仿真 `/clock` 是否还在推进。目标落在障碍物内时应明确失败，不能把附近的点当成到达。

回归检查（先 source ROS 及工作区）：

```bash
python3 -m pytest -q src/navigation/navigation_tasks/test
python3 src/navigation/navigation_tasks/test/nav2_closed_loop_check.py
python3 src/navigation/navigation_tasks/test/nav2_elevator_check.py
```

第二个是显式运行的 Nav2 软件闭环检查，使用隔离的 ROS domain 93、合成地图、
激光、TF、里程计和差速运动模型，检查直行、原地转向、绕障及障碍内目标失败。
它不会启动真实机器人；结果及日志保存在 `/tmp/od-navigation-check`。
第三个使用隔离的 ROS domain 94、现有 robot2 的身份和四个真实楼层地图，
验证候梯点到梯内点、梯内转向以及出梯。每次运动都核对带余量的矩形外形
与地图障碍格是否相交；结果及日志保存在 `/tmp/od-elevator-check`。
检查使用真实 Nav2 与软件运动模型，不注册机器人，也不向生产 ROS 域发命令。
真实机器人仍需验证定位精度、底盘制动及狭窄通道表现。
