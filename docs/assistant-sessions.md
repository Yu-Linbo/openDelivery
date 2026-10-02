# OpenClaw 对话与管理页

登录状态复用 linbo.lol 的共享认证。登录用户可以开启新对话；当前与历史对话保存在后端 SQLite，管理入口是 `/opendelivery/assistant-admin.html`。首次登录打开助手会导入浏览器原有的最近 50 条消息，之后的消息与异步操作结果由后端保存。

原有对话与游客继续使用 OpenClaw 的 main 会话，新对话使用独立的 `agent:main:opendelivery-...` 会话键。因此 reset 不会删除旧对话，也不会清空游客仍在使用的原有上下文。旧版浏览器未保存的消息无法从浏览器恢复。

操作意图、机器人选择、点位匹配和执行步骤均由 AI 根据对话与实时状态决定。后端向 AI 提供在线机器人和点位目录，不使用关键词判断意图、不替换机器人、不自动插入上线步骤。提示要求 AI：未指定机器人时优先选择在线、空闲且 ready 的机器人；没有在线机器人时选择未占用编号，并在导航计划前明确加入仿真上线步骤。显式指定的机器人保持不变。

AI 返回 `decision`（`execute` / `query` / `clarify` / `chat`）及完整 `actions`。后端仅校验结构、动作白名单、参数和权限：查询不能包含写操作，澄清/闲聊不能包含动作，执行计划不能为空。通过校验后按 AI 给出的顺序执行，等待上线 ready 及每一步导航完成，取货返回也需等返程完成。

回复及执行结果使用本轮提问的语言；英文、中文使用后端模板，其他语言使用模型提供的状态模板。会话消息不参与工作台界面翻译，切换界面语言不会改写回复。请求是否授权执行由 AI 结合语义和上下文判断，支持不同语言和表达方式。是否展示原始 API 数据也由 AI 的 `include_result` 字段决定。

## 反向代理

在现有 `/etc/nginx/sites-available/vps-stack-web` 的 `/opendelivery/api/` location 中加入：

```nginx
proxy_set_header X-Auth-User $linbo_user;
```

`$linbo_user` 已由 `/etc/nginx/snippets/linbo-auth.conf` 的 `auth_request_set` 获取。该指令覆盖客户端自行提交的身份头。后端只信任来自现有 Tailscale 代理 `100.64.0.2` 的身份；本机或直接访问后端的请求按游客处理。代理地址迁移时通过 `OPEN_DELIVERY_AUTH_PROXIES` 配置准确的新地址。

旧的 `https://linbo.lol/openclaw/` 在 443 端口没有管理代理。兼容旧书签的两个精确路由可导向新的会话管理页：

```nginx
location = /openclaw { return 302 /opendelivery/assistant-admin.html; }
location = /openclaw/ { return 302 /opendelivery/assistant-admin.html; }
```

原生 OpenClaw Control UI 仍使用既有的 `https://linbo.lol:18789/openclaw/` 独立入口。新会话管理页不需要在浏览器中提供 Gateway token。

修改前备份配置，运行 `nginx -t`，通过后 reload nginx。后端由 `start_web_stack.sh` 的 supervisor 管理，仅重启后端进程即可加载新代码；不要停止整个工作台或 ROS/Gazebo。

## 验证

```sh
python3 -m unittest discover -s backend/tests -p 'test_assistant_sessions.py'
python3 -m unittest discover -s backend/tests -p 'test_openclaw_chat.py'
```

验证范围包括：游客与伪造身份不能 reset/读取历史、跨用户隔离、旧消息持久化、重复 reset 返回冲突、新 OpenClaw 会话键、后台操作结果写回原会话。
