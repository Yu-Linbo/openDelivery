# 日志规范与 Web 观测

在日志页选择 bag 并点击播放，播放器自动加载对应日志。地图、相机和状态位于上方，日志位于下方，共用播放进度；日志默认随播放时间高亮，点击时间可以跳转，取消“跟随播放时间”可自由翻页。支持级别、节点、文本 / 正则组合筛选，搜索高亮和错误数量统计；无结构文本归入“原文”，不会丢弃堆栈或命令输出。

## 统一格式

```text
[INFO] [1790730000.123456789] [robot_log_recorder]: opening bag
[WARN] [1790730001.123456789] [amcl]: laser scan delayed
```

字段依次为级别、秒时间戳、来源节点 / 模块、消息。级别使用 `DEBUG`、`INFO`、`WARN`、`ERROR`、`FATAL`，兼容旧 `WARNING`。平台与录制文本使用系统接收时间，与 bag 存储时间对齐；rosout 原始时间另以 `ros_time` 字段保留，兼容仿真时钟。bag 内自带 rosout 时，播放定位使用消息存储时间，消息自身时间仍单独保留。录制器保留 rosout 的文件、行号、函数信息为消息尾部 `source` 字段。

平台 Python 模块使用 `logging.getLogger("opendelivery.<模块>")`，正常事件用 INFO，可恢复异常用 WARN，操作失败用 ERROR，线程终止用 FATAL。`server.py` 启动时配置控制台和轮转文件。托管 ROS 子进程默认设置 `RCUTILS_CONSOLE_OUTPUT_FORMAT=[{severity}] [{time}] [{name}]: {message}`，关闭 ANSI 色彩；显式环境配置仍优先。

平台事件在兼容 ROS 的前缀后增加可读的 UTC 时间与 `key=value` 字段；字符串采用 JSON 转义，换行不会伪造下一条记录。例如：

```text
[ERROR] [1791104118.123000000] [opendelivery.assistant]: time=2026-10-04T08:55:18.123+00:00 event="assistant.navigation_failed" request_id="..." session_id="..." job_id="..." step=2 robot_id="robot1" floor_id="test_103" task_id="web_nav_..." error="navigation Failed"
```

## 请求、对话与任务的定位

每个 HTTP 请求由后端生成独立 `request_id`，响应头 `X-Request-ID` 返回该值。助手后台线程和 ROS 命令队列继承日志上下文，因此可按同一 ID 串起请求、AI 规划、任务步骤与 ROS 下发记录。机器人上报的任务状态通过 `robot_id`、`task_id` 对应下发记录，不因周期性进度更新重复输出。

| 事件 | 可定位的信息 |
| --- | --- |
| `http.request_completed` | 方法、路径（不含查询字符串）、HTTP 状态、耗时、来源地址、错误原因；500 异常保留堆栈 |
| `assistant.session_resolved` / `assistant.session_reset` | 登录身份、会话 ID、旧会话到新会话的对应关系 |
| `assistant.request_failed` | 失败阶段：参数验证、事实读取、模型调用、计划验证或执行；异常类型与原因 |
| `assistant.model_started` / `assistant.plan_validated` | agent、thinking、超时、prompt 字符数、实际模型、token / cache 用量、规划耗时与动作列表 |
| `assistant.job_queued` / `assistant.job_started` / `assistant.job_finished` | 请求与后台 job 的对应、计划步数、完成步数、总耗时、失败原因 |
| `assistant.action_started` / `assistant.action_result` / `assistant.action_completed` | 第几步、动作、机器人、楼层、点位、导航 task ID；`action_result` 表示 API 结果，导航须等 `action_completed` 才表示到达 |
| `assistant.navigation_observed` / `assistant.robot_observed` | 实际状态变化；长时间等待每 30 秒补充进展，轮询失败按首次及 30 秒间隔汇总 |
| `ros.command_queued` / `ros.command_dispatched` / `ros.command_failed` | 请求到 ROS 的 command ID、机器人、task ID 和下发失败原因；下发成功不等于任务完成 |
| `ros.task_status_changed` | 机器人任务状态的前后变化（包括 Finished、Failed、Terminated） |
| `managed.started` / `managed.exited` | 节点 ID、PID、日志路径、启动命令、退出码，以及是否主动停止 |

正常 GET 轮询、OPTIONS 和遥控续租成功记录为 DEBUG，默认 INFO 不刷屏；所有 HTTP 失败及耗时超过 2 秒的普通请求仍记录。助手模型请求以自身耗时事件记录，长连接不按普通慢请求处理。不主动记录完整对话正文、prompt、模型回复、认证头或环境变量；诊断字段限制长度，并遮盖常见 token、password、api_key、Bearer 值。

从浏览器 Network 面板获取 `X-Request-ID` 后，在项目根目录查询：

```bash
rg 'request_id="实际ID"' backend/logs/platform*.log
rg 'session_id="会话ID"|job_id="任务ID"|task_id="导航ID"' backend/logs/platform*.log
rg '\[ERROR\]|\[WARN\]' backend/logs/platform*.log
tail -n 100 backend/logs/managed_robot1.log
```

## 目录与保留

| 来源 | 位置 | 规则 |
| --- | --- | --- |
| 平台 | `backend/logs/platform.log` | 每文件 5 MiB，保留 5 个备份（`platform.log.1.log` 至 `.5.log`） |
| 托管进程 | `backend/logs/managed_<id>.log` | 新启动进程由独立收集器记录；每文件 5 MiB，保留 5 个 `.N.log` 备份；连续完全相同的行汇总重复次数 |
| ROS | `backend/logs/ros/` | ROS 创建的节点和 launch 日志 |
| 机器人录制 | `log_bag/<robot>/backup/logs/` | 沿用 recorder 的归档与保留策略，关联 bag 的删除仍由原接口处理 |
| 构建与测试 | `log/` | 保留 colcon 原始文件与目录结构 |

已有日志不批量重写或删除。已有超大托管文件在首次轮转时完整保留为 `.1.log`，后续遵循五个备份的保留策略。已经运行、直接写文件的进程沿用原记录方式；新启动的进程启用收集器，后端重载不会关闭其日志管道。

收集器保留第一条原始行；连续相同行以 `managed.output_repeated` 记录省略次数和样例，持续重复时每 10 秒汇总，内容变化或输出结束时补齐剩余次数。不同内容与堆栈行按原顺序保留。单行超过 64 KiB 会截断并记录 `managed.output_line_truncated`；其后的正常日志继续读取。该机制压缩重复信息，不把网络异常判为恢复或隐藏首次错误。

浏览器兼容 ROS 标准、扩展、launch 前缀，以及旧录制器 `[ISO时间] rosout level=<数字> name=... file=... line=... msg=...` 格式。其他日志和多行续行按原文呈现。页面语言切换不会翻译日志正文。

## 大文件与布局

关联文本读取限末尾 2 MiB，日志列表每页渲染 250 行。表格在播放器内部滚动，窄屏依次展示画面、状态与日志；bag 列表、下载和离线回放保持原有流程。多 bag 按各自 segment 的源时间转换为连续播放时间，不将录制间隔误算为播放时间。无时间的原文保留在列表末尾，缺失或损坏的关联日志不影响 bag 播放。若 bag 自带 rosout，则优先使用该时间线；超过每主题 12000 条时按间隔采样并提示。原始日志仍可随 bag 下载。

## 验证

```bash
python3 -m unittest discover -s backend/tests -p 'test_diagnostic_logging.py'
python3 -m unittest discover -s backend/tests -p 'test_log*.py'
python3 -m unittest discover -s backend/tests -p 'test_web*.py'
node web/tests/log_parser.test.cjs
colcon build --packages-select log_bag --symlink-install
ctest --test-dir build/log_bag --output-on-failure
```
