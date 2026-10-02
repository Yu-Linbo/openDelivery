# OpenClaw 成本与指令精简（2026-10-01）

## 模型状态

切换前真实请求使用 `openai/gpt-5.6-sol`、`medium`。用户指定的目标是 `openai/gpt-6.1-sol`、`low`（light 对应官方 low）。本机目录尚无该模型，已添加官方规格及 Codex / ChatGPT 路由条目；配置校验通过，但真实请求 HTTP 400：

> The 'gpt-6.1-sol' model is not supported when using Codex with a ChatGPT account.

因此当前默认恢复为已验证的 `openai/gpt-5.6-sol`、`low`，避免中断助手。`active.patch.json` 是可用配置，`config.patch.json` 是待账户开放后使用的目标配置。若改用 OpenAI API，需先在 OpenClaw 安全配置 API 认证，将目标模型条目的 `api` 改为 `openai-responses`，再验证真实请求；不能把订阅路由当作 API 路由。不要在聊天、仓库或日志写密钥。

官方参数来源：[GPT-6.1 Sol](https://developers.openai.com/api/docs/models/gpt-6.1-sol)。

## 已应用的精简

- 工作区 `AGENTS.md` 从 8716 bytes 的通用模板改为配送相关的简短规则；保留会话隔离、语言匹配、实时事实和隐私约束，删除日记必读、群聊表情、语音和无关主动检查。
- 重写 operations skill，移除旧的“全部请求共用 main session”要求。Codex 的通用技能在此 agent 的独立配置中禁用，operations 作为可显式调用的参考保留；OpenClaw 的自动技能清单设为空，Codex skill 的 `allow_implicit_invocation: false` 避免常规请求已经携带规则时再次读取 skill。
- main 的动态工具配置由 messaging 改为 minimal，仅提供 session_status。MCP 当前为空，配送由 bridge 白名单执行；不新增与配送无关的 MCP。原有频道配置仍保留。
- 每轮提示精简重复规则；浏览器只提供 view/floor/robot_id，不注入网址或浏览器声称的在线列表。在线状态仅来自后端实时快照。
- 点位目录完整序列化，取消 6000 字符截断，避免切坏 JSON 或丢失候选点。后端继续仅校验参数与执行 AI 计划，不使用关键词选意图、机器人或点位。
- bridge 显式使用 low 推理，覆盖旧会话的 minimal/medium；模型随 Gateway 的已验证默认值，部署可用 OPENCLAW_MODEL 显式覆盖。
- Codex 配置自动压缩阈值为 24000 tokens，工具结果预算 1200 tokens。历史会话不 reset；管理员保存的旧消息保持完整。
- 平台日志记录实际模型、prompt_tokens、cache_read、output_tokens。这些是 runtime 报告值；不要把包含缓存的上下文总量当作未缓存输入计费。

## 安装与恢复

安装前已备份到 `~/.openclaw/backups/model-cost-20261001-192915/`，含 OpenClaw 配置、原 AGENTS、原 skill 和 Codex 配置。备份含认证信息，应仅在原机器保护保存，不复制到仓库。

```sh
openclaw config patch --file ops/openclaw/active.patch.json --dry-run --json
openclaw config patch --file ops/openclaw/active.patch.json
openclaw config validate
```

`AGENTS.md`、skills 目录同步到 `~/.openclaw/workspace/`。operations skill 同步到该 agent 的 `codex-home/skills/`；`codex.settings.toml` 安装到该 agent 的 config.toml 时须保留已有 projects trust 配置。只重新加载 Gateway 和 Python 后端，不停止 ROS/Gazebo。

模型更换后必须用无执行的真实计划请求核实 `agentMeta.model`、`requestShaping.thinking` 和原始 turn_context，不能只检查配置。验证应覆盖在线机器人、全部离线需新编号、英中文回复、否定请求，以及多站顺序计划。

## 实测结果

最近真实用户会话 `opendelivery-4a76d395403440dd850d42f8b38ffab8` 的英文配送计划包含两次导航，平台日志记录 job `023c78aab7f14b0ba4dc1613b60aa999` 已 completed。此前一轮模型输入 22231 tokens，其中 19968 命中缓存。另一个旧 main 会话报告约 79925 tokens 上下文，历史累积也是成本来源。

最终 3 个隔离规划测试均通过，机器人执行线程被拦截：在线 robot1 的英文取货送货；仅离线 robot1 时中文计划选择新 robot2 并先上线；英文否定执行请求返回 chat/空 actions。原始轨迹均确认 gpt-5.6-sol / low，每轮只有一次模型采样、零工具调用，输入分别 12298 / 12303 / 12279 tokens。测试使用固定机器人快照和两点目录；该数值不是线上费用承诺，真实输入取决于地图规模、历史和缓存。之前类似验证约 19350 tokens 输入。

现有相关 Python 检查合计 56 项通过（11 bridge、18 assistant、19 web monitor、8 i18n），skill 校验和 git diff --check 通过；后端重载后游客 session 接口正常。目标 6.1 Sol 的真实验证仍被账户权限阻止。
