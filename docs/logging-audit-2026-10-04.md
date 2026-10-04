# 2026-10-04 日志检查记录

检查范围：`backend/logs/platform.log`、三个 `managed_*.log` 文件，以及 `/tmp/openclaw/openclaw-2026-10-03.log`、`openclaw-2026-10-04.log`。以下为检查时的证据，不代表所有历史错误仍在发生。

## 发现与处理

| 发现 | 证据与处理 |
| --- | --- |
| 平台缺少请求链路与失败上下文 | 改动前平台文件只有 21 行，主要为启动、动作接受与 job 结束记录。已补充请求 ID、会话 / job / task ID、具体步骤和目标、耗时、失败阶段、原因与异常堆栈。 |
| Gazebo 网络异常淹没其他信息 | `managed_simulation_world.log` 约 96 MB，包含 1,448,011 行 `Exception sending a multicast message:No such device`、542 行 `Network is unreachable`。文件最后修改时间为 10 月 2 日，属于既有输出。新启动的托管进程启用完全相同行的周期汇总及轮转；保留首次错误与重复次数。此改动改善记录方式，网络异常仍需根据发生时的网卡 / 多播环境排查。 |
| 机器人历史启动错误难以归属 | `managed_robot2.log` 中有 Xvfb 启动失败、空 `map_file_name:=` 参数及 shell 语法错误等历史记录；该文件最后修改时间为 4 月 19 日。新日志记录 node ID、PID、启动命令、输出文件和退出码，可把一次启动与其错误对应起来。 |
| 飞书通道持续失败重试 | 10 月 3 日及 4 日 OpenClaw 原生日志反复出现 `Plugin "feishu" loaded with origin "global"; reason=record-missing`，并说明 channel ingress queue 仅允许 trusted plugins 使用。10 月 4 日最近记录至检查时仍在重试。后续处理应检查插件安装 / 注册记录和版本兼容性；不要将该错误归为机器人导航失败，也不要通过降低日志级别隐藏它。 |
| 后台 memory-core 认证错误 | 两天日志中另有 `lane=background:plugin:memory-core` 的 OpenAI Responses HTTP 401，提示缺少认证。应检查该后台插件使用的认证配置；它与飞书重试、机器人导航及助手前台模型请求分别定位。 |

## 验证证据

- 最终完整后端回归 197 项通过，包含日志专用测试 11 项和 ROS 队列测试 3 项；前端解析器 10 个用例通过。
- 临时测试进程连续输出 1,000 条相同行，保留首条并汇总 `repeated_lines=999`；不同后续行和输出结束记录均保留。轮转及超长行读取有验证。
- 独立收集器在父进程退出后继续记录子进程输出，避免后端重载切断机器人日志管道。
- 后端实际重载后，本地请求返回 `authenticated=true, user=linbo`；网站游客返回 `authenticated=false`。两处响应均包含 `X-Request-ID`，平台文件中可查询对应记录。
- 实际无效 JSON 请求返回 HTTP 400，日志记录请求 ID、来源、耗时及 `error="invalid JSON body"`；连续成功 GET 轮询不产生 INFO 请求刷屏。
- 浏览器中地图正常加载，助手新对话按钮可见，API 请求无失败，控制台无错误。

日志事件与查询方法见 [logging.md](logging.md)。后续出现未完成的助手任务时，从请求 ID 查询其会话、job 和导航 task，再检查 `assistant.navigation_observed` 与 `ros.task_status_changed`，区分未下发、正在等待、失败和已完成。
