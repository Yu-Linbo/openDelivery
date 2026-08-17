# simulate

本包除 Gazebo 机器人、传感器和世界外，还包含临时换层节点 `fake_elevator`。

## fake_elevator

节点由 `simulate.launch.py` 随机器人实体启动，位于 `/<robot>/fake_elevator`。它保持一套可被
真实电梯管理器替换的模块接口：

| Topic | 消息 | 方向 |
|-------|------|------|
| `/<robot>/fake_elevator/info` | `ElevatorInfo` | task_manager → fake_elevator |
| `/<robot>/fake_elevator/status` | `ElevatorStatus` | fake_elevator → task_manager |
| `/<robot>/fake_elevator/command` | `ElevatorCommand` | task_manager → fake_elevator |

`ElevatorInfo.operation` 区分两类任务：

1. `call`：发布 `Calling`，等待 `call_delay_sec`（默认 3 秒）后完成，不移动模型也不切图。
2. `ride`：发布 `Riding` 并等待 `ride_delay_sec`，随后进入 `MovingModel`，调用
   `/gazebo/set_model_state` 将机器人实体移动至 `target_inside_pose`。
3. `SwitchingMap`：把 heartbeat 的 `current_map` 改为目标楼层、状态改为
   `localization_lost`，再调用 `/<robot>/map_server/load_map` 加载
   `<map_root>/<floor>/<floor>.yaml`。
4. `Relocalizing`：使用上一楼层梯内点调用 `/<robot>/relocalize` 模式 1。
5. `Finished` 或 `Failed`：将最终结果和匹配分数写入状态消息。

Pause、Resume、Terminate 对当前电梯子任务生效。状态话题使用 transient-local QoS，使后加入的
task_manager 或监控端也能读取最新阶段。

`fake_elevator` 不控制真实轿厢，也没有呼梯、门控、进出梯检测和安全联锁。它目前只验证
“任务拆分 → 换图 → 重定位 → 继续导航”的编排链路；接入真实电梯时应保持上述消息接口。
