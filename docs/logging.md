# 日志规范与 Web 观测

在日志页选择 bag 并点击播放，播放器自动加载对应日志。地图、相机和状态位于上方，日志位于下方，共用播放进度；日志默认随播放时间高亮，点击时间可以跳转，取消“跟随播放时间”可自由翻页。支持级别、节点、文本 / 正则组合筛选，搜索高亮和错误数量统计；无结构文本归入“原文”，不会丢弃堆栈或命令输出。

## 统一格式

```text
[INFO] [1790730000.123456789] [robot_log_recorder]: opening bag
[WARN] [1790730001.123456789] [amcl]: laser scan delayed
```

字段依次为级别、秒时间戳、来源节点 / 模块、消息。级别使用 `DEBUG`、`INFO`、`WARN`、`ERROR`、`FATAL`，兼容旧 `WARNING`。平台与录制文本使用系统接收时间，与 bag 存储时间对齐；rosout 原始时间另以 `ros_time` 字段保留，兼容仿真时钟。bag 内自带 rosout 时，播放定位使用消息存储时间，消息自身时间仍单独保留。录制器保留 rosout 的文件、行号、函数信息为消息尾部 `source` 字段。

平台 Python 模块使用 `logging.getLogger("opendelivery.<模块>")`，正常事件用 INFO，可恢复异常用 WARN，操作失败用 ERROR，线程终止用 FATAL。`server.py` 启动时配置控制台和轮转文件。托管 ROS 子进程默认设置 `RCUTILS_CONSOLE_OUTPUT_FORMAT=[{severity}] [{time}] [{name}]: {message}`，关闭 ANSI 色彩；显式环境配置仍优先。

## 目录与保留

| 来源 | 位置 | 规则 |
| --- | --- | --- |
| 平台 | `backend/logs/platform.log` | 每文件 5 MiB，保留 5 个备份（`platform.log.1.log` 至 `.5.log`） |
| 托管进程 | `backend/logs/managed_<id>.log` | 保留原有进程输出；启动记录采用标准格式 |
| ROS | `backend/logs/ros/` | ROS 创建的节点和 launch 日志 |
| 机器人录制 | `log_bag/<robot>/backup/logs/` | 沿用 recorder 的归档与保留策略，关联 bag 的删除仍由原接口处理 |
| 构建与测试 | `log/` | 保留 colcon 原始文件与目录结构 |

历史日志不重写、不迁移、不批量删除。浏览器兼容 ROS 标准、扩展、launch 前缀，以及旧录制器 `[ISO时间] rosout level=<数字> name=... file=... line=... msg=...` 格式。其他日志和多行续行按原文呈现。页面语言切换不会翻译日志正文。

## 大文件与布局

关联文本读取限末尾 2 MiB，日志列表每页渲染 250 行。表格在播放器内部滚动，窄屏依次展示画面、状态与日志；bag 列表、下载和离线回放保持原有流程。多 bag 按各自 segment 的源时间转换为连续播放时间，不将录制间隔误算为播放时间。无时间的原文保留在列表末尾，缺失或损坏的关联日志不影响 bag 播放。若 bag 自带 rosout，则优先使用该时间线；超过每主题 12000 条时按间隔采样并提示。原始日志仍可随 bag 下载。

## 验证

```bash
python3 -m unittest discover -s backend/tests -p 'test_log*.py'
python3 -m unittest discover -s backend/tests -p 'test_web*.py'
node web/tests/log_parser.test.cjs
colcon build --packages-select log_bag --symlink-install
ctest --test-dir build/log_bag --output-on-failure
```
