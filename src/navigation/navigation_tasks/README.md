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

`/<robot>/navigate_to_pose` 作为兼容 action 保留，正常 Web 任务链路不直接使用。
启动或切图时 Nav2 lifecycle 激活可能晚于机器人状态更新。执行器通过参数
`action_server_wait_sec` 等待 action server，默认 15 秒；超时状态会包含实际等待的
action 名称。
