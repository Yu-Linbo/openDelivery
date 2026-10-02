(function () {
  "use strict";
  const port = Number(location.port || 0);
  const base = window.API_BASE_URL || (location.pathname.startsWith("/opendelivery/")
    ? location.origin + "/opendelivery"
    : `${location.protocol}//${location.hostname}:${port ? port + 1 : 8001}`);
  const status = document.getElementById("status");
  const list = document.getElementById("sessions");
  const messages = document.getElementById("messages");
  const title = document.getElementById("title");
  const info = document.getElementById("session-info");
  const login = document.getElementById("login");
  let selected = "";
  let requestId = 0;
  const date = (epoch) => new Date(epoch * 1000).toLocaleString();
  async function read(path) {
    const response = await fetch(base + path, { credentials: "same-origin", cache: "no-store" });
    if (response.status === 403 || response.redirected) {
      login.hidden = false;
      list.replaceChildren();
      messages.replaceChildren();
      title.textContent = "请登录后查看会话管理";
      info.textContent = "";
      throw new Error("请登录后查看会话管理");
    }
    const payload = await response.json();
    if (!response.ok) throw new Error(payload.error || "读取失败");
    login.hidden = true;
    return payload;
  }
  async function select(id) {
    selected = id;
    const generation = ++requestId;
    status.textContent = "正在读取…";
    try {
      const { session } = await read(`/api/assistant/sessions/${encodeURIComponent(id)}`);
      if (generation !== requestId) return;
      for (const button of list.children) button.setAttribute("aria-pressed", button.dataset.id === id ? "true" : "false");
      title.textContent = session.title;
      info.textContent = `${session.active ? "当前对话" : "历史对话"} · ${date(session.created)} · ${session.id}`;
      messages.replaceChildren();
      for (const item of session.messages) {
        const row = document.createElement("div");
        row.className = `message ${item.role === "user" ? "user" : "assistant"}`;
        const label = document.createElement("small");
        label.textContent = `${item.role === "user" ? "你" : "OpenClaw"} · ${date(item.created)}`;
        row.append(label, document.createTextNode(item.text));
        messages.append(row);
      }
      if (!session.messages.length) messages.textContent = "这个对话还没有消息。";
      status.textContent = "";
    } catch (error) { if (generation === requestId) status.textContent = error.message; }
  }
  async function refresh() {
    status.textContent = "正在读取…";
    try {
      const { sessions } = await read("/api/assistant/sessions");
      list.replaceChildren();
      for (const session of sessions) {
        const button = document.createElement("button");
        button.type = "button";
        button.className = "session";
        button.dataset.id = session.id;
        button.setAttribute("aria-pressed", "false");
        const name = document.createElement("span");
        name.textContent = session.title;
        const detail = document.createElement("small");
        detail.textContent = `${session.active ? "当前对话" : "历史对话"} · ${session.message_count} 条消息 · ${date(session.updated)}`;
        button.append(name, detail);
        button.addEventListener("click", () => select(session.id));
        list.append(button);
      }
      if (!sessions.length) {
        status.textContent = "暂无对话，请先在工作台打开 OpenClaw 助手。";
        messages.replaceChildren();
        title.textContent = "暂无对话";
        info.textContent = "";
        return;
      }
      await select(sessions.some((item) => item.id === selected) ? selected : sessions[0].id);
    } catch (error) { status.textContent = error.message; }
  }
  document.getElementById("refresh").addEventListener("click", refresh);
  refresh();
})();
