let API_BASE_URL =
  window.API_BASE_URL || `${window.location.protocol}//${window.location.hostname}:8001`;

const menuButtons = Array.from(document.querySelectorAll(".menu-item"));
const layoutEl = document.querySelector(".layout");
const btnSidebarCollapse = document.getElementById("btn-sidebar-collapse");
const btnSidebarReveal = document.getElementById("btn-sidebar-reveal");
const robotPresenceAnchor = document.getElementById("robot-presence-anchor");
const btnRobotPresence = document.getElementById("btn-robot-presence");
const robotPresencePanel = document.getElementById("robot-presence-panel");
const robotPresenceList = document.getElementById("robot-presence-list");
const robotPresenceEmpty = document.getElementById("robot-presence-empty");
const robotPresenceSummary = document.getElementById("robot-presence-summary");
const views = {
  monitor: document.getElementById("view-monitor"),
  gazebo: document.getElementById("view-gazebo"),
  ros: document.getElementById("view-ros"),
  settings: document.getElementById("view-settings"),
  logs: document.getElementById("view-logs"),
};

function setSidebarHidden(hidden) {
  if (!layoutEl) {
    return;
  }
  layoutEl.classList.toggle("sidebar-hidden", hidden);
  try {
    localStorage.setItem("openDelivery_sidebar_hidden_v1", hidden ? "1" : "0");
  } catch {
    /* ignore */
  }
}

if (layoutEl) {
  const saved = localStorage.getItem("openDelivery_sidebar_hidden_v1") === "1";
  layoutEl.classList.toggle("sidebar-hidden", saved);
}

if (btnSidebarCollapse) {
  btnSidebarCollapse.addEventListener("click", () => setSidebarHidden(true));
}
if (btnSidebarReveal) {
  btnSidebarReveal.addEventListener("click", () => setSidebarHidden(false));
}

const floorSelect = document.getElementById("floor-select");
const gridToggle = document.getElementById("grid-toggle");
const mapStatus = document.getElementById("map-status");
const mapMeta = document.getElementById("map-meta");
const robotStatus = document.getElementById("robot-status");
const mapWrapper = document.querySelector(".map-wrapper");
const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");

const settingsForm = document.getElementById("settings-form");
const settingsMessage = document.getElementById("settings-message");
const logList = document.getElementById("log-list");
const addLogBtn = document.getElementById("btn-add-log");
const clearLogBtn = document.getElementById("btn-clear-log");
const btnResetView = document.getElementById("btn-reset-view");
const scan2dToggle = document.getElementById("scan-2d-toggle");
const plannedPathToggle = document.getElementById("planned-path-toggle");
const relocRobotId = document.getElementById("reloc-robot-id");
const relocX = document.getElementById("reloc-x");
const relocY = document.getElementById("reloc-y");
const relocYaw = document.getElementById("reloc-yaw");
const relocMessage = document.getElementById("reloc-message");
const btnRelocMapOnly = document.getElementById("btn-reloc-map-only");
const btnRelocPoseOnly = document.getElementById("btn-reloc-pose-only");
const btnRelocBoth = document.getElementById("btn-reloc-both");
const btnRelocFillPose = document.getElementById("btn-reloc-fill-pose");
const relocPickToggle = document.getElementById("reloc-pick-toggle");
const btnRelocSkipHeading = document.getElementById("btn-reloc-skip-heading");
const btnRelocClearPick = document.getElementById("btn-reloc-clear-pick");
const mapNameInput = document.getElementById("map-name-input");
const btnSaveMap = document.getElementById("btn-save-map");
const rosNodesSummary = document.getElementById("ros-nodes-summary");
const rosNodesError = document.getElementById("ros-nodes-error");
const rosNodesPersistentList = document.getElementById("ros-nodes-persistent");
const rosNodesCreatedList = document.getElementById("ros-nodes-created");
const rosNodesDiscoveredList = document.getElementById("ros-nodes-discovered");
const btnRosNodesRefresh = document.getElementById("btn-ros-nodes-refresh");
const rosNodeCreateForm = document.getElementById("ros-node-create-form");
const rosNodeCreateTypeSelect = document.getElementById("ros-node-create-type");
const rosNodeFieldsFakePub = document.getElementById("ros-node-fields-fake_pub");
const rosNodeFieldsSlamMapping = document.getElementById(
  "ros-node-fields-slam_bringup_mapping"
);
const rosNodeRobotName = document.getElementById("ros-node-robot-name");
const rosNodeSlamRobotName = document.getElementById("ros-node-slam-robot-name");
const rosNodeCurrentMap = document.getElementById("ros-node-current-map");
const btnRosNodeCreate = document.getElementById("btn-ros-node-create");

const SETTINGS_KEY = "robotSettings";
const LOGS_KEY = "robotLogs";
const MONITOR_CHECKBOXES_KEY = "openDelivery_monitor_checkboxes_v1";
const FLOOR_PREF_KEY = "openDelivery_active_floor_v1";
const MAP_NAME_PREF_KEY = "openDelivery_map_name_v1";
const ACTIVE_VIEW_KEY = "openDelivery_active_view_v1";
const MAPPING_SUFFIX = "_mapping";
const ROBOT_ICON_PATH = "./icons/robot.svg";

const ZOOM_MIN = 0.15;
const ZOOM_MAX = 8;

let floors = [];
let activeFloor = null;
let activePgm = null;
let activeMeta = null;
/** @type {HTMLCanvasElement | null} */
let mapBitmap = null;

/** Latest snapshot from backend: { timestamp, source, robots: [...] } */
let latestSnapshot = null;

/** Rows from `GET /api/robot/status/cache` (merged with pose for 离线 robots). */
let robotStatusCacheItems = [];
let robotPresencePanelOpen = false;
let robotPresenceCacheTimer = null;

/** Per-robot overlays from REST (map frame) */
const latestScanByRobot = {};
const latestPathByRobot = {};
let sensorPollTimer = null;

/** Live OccupancyGrid polling for floor like robot1_mapping → GET /api/mapping/live?robot_id= */
let mapLiveTimer = null;
let mapLiveInitializedView = false;
/** @type {string | null} */
let activeMappingRobotId = null;
let rosNodesPollTimer = null;

let robotIconLoaded = false;
const robotIcon = new Image();
robotIcon.onload = () => {
  robotIconLoaded = true;
  renderScene();
};
robotIcon.onerror = () => {
  robotIconLoaded = false;
};
robotIcon.src = ROBOT_ICON_PATH;

/** View in CSS pixels: uniform scale (px per map pixel), top-left of map on canvas */
let viewScale = 1;
let viewPanX = 0;
let viewPanY = 0;

let isDragging = false;
let dragLastX = 0;
let dragLastY = 0;

/** 地图选位姿：0 定点，1 定朝向 */
let relocPickStep = 0;
/** @type {{ x: number, y: number } | null} */
let relocPickAnchorWorld = null;
let relocPickHoverSx = null;
let relocPickHoverSy = null;

function activateView(target, persist = true) {
  const next = views[target] ? target : "monitor";
  menuButtons.forEach((b) => b.classList.toggle("active", b.dataset.view === next));
  Object.entries(views).forEach(([key, pane]) => {
    if (pane) {
      pane.classList.toggle("active", key === next);
    }
  });
  if (persist) {
    try {
      localStorage.setItem(ACTIVE_VIEW_KEY, next);
    } catch {
      /* ignore */
    }
  }
  if (window.location.hash !== `#${next}`) {
    history.replaceState(null, "", `#${next}`);
  }
}

function getInitialView() {
  const hashView = window.location.hash.replace(/^#/, "").trim();
  if (views[hashView]) {
    return hashView;
  }
  try {
    const savedView = localStorage.getItem(ACTIVE_VIEW_KEY);
    if (savedView && views[savedView]) {
      return savedView;
    }
  } catch {
    /* ignore */
  }
  return "monitor";
}

menuButtons.forEach((btn) => {
  btn.addEventListener("click", () => {
    activateView(btn.dataset.view);
  });
});

window.addEventListener("hashchange", () => {
  activateView(window.location.hash.replace(/^#/, "").trim(), false);
});

activateView(getInitialView(), false);

function isMappingFloor(f) {
  return typeof f === "string" && f.endsWith(MAPPING_SUFFIX);
}

function robotIdFromMappingFloor(f) {
  if (!isMappingFloor(f)) {
    return "";
  }
  return f.slice(0, -MAPPING_SUFFIX.length);
}

function updateMappingToolbar() {
  const row = document.getElementById("mapping-toolbar-row");
  if (!row) {
    return;
  }
  row.classList.toggle("is-active", isMappingFloor(activeFloor));
}

function addFloorOptions() {
  floorSelect.innerHTML = "";
  floors.forEach((floor) => {
    const option = document.createElement("option");
    option.value = floor;
    option.textContent = isMappingFloor(floor) ? `${floor} · 建图中` : floor;
    floorSelect.appendChild(option);
  });
}

async function fetchJson(path) {
  const res = await fetch(path);
  if (!res.ok) {
    throw new Error(`请求失败: ${path} (${res.status})`);
  }
  return res.json();
}

async function canReachApi(baseUrl, timeoutMs = 1200) {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), timeoutMs);
  try {
    const res = await fetch(`${baseUrl}/api/floors`, { signal: ctrl.signal });
    return res.ok;
  } catch {
    return false;
  } finally {
    clearTimeout(timer);
  }
}

async function resolveApiBaseUrl() {
  if (window.API_BASE_URL) {
    API_BASE_URL = window.API_BASE_URL;
    return;
  }
  if (await canReachApi(API_BASE_URL)) {
    return;
  }

  const protoHost = `${window.location.protocol}//${window.location.hostname}`;
  const originPort = Number(window.location.port || 0);
  const candidatePorts = [8001, 8002, 8003, originPort + 1, originPort + 2].filter(
    (p) => Number.isInteger(p) && p > 0
  );
  const seen = new Set();
  for (const p of candidatePorts) {
    if (seen.has(p)) {
      continue;
    }
    seen.add(p);
    const base = `${protoHost}:${p}`;
    if (await canReachApi(base)) {
      API_BASE_URL = base;
      return;
    }
  }
}

async function postRobotCommand(payload) {
  const res = await fetch(`${API_BASE_URL}/api/robot/command`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  let data = {};
  try {
    data = await res.json();
  } catch {
    /* ignore */
  }
  if (!res.ok) {
    throw new Error(data.error || `请求失败 (${res.status})`);
  }
  return data;
}

async function postGazeboSetModelState(payload) {
  const res = await fetch(`${API_BASE_URL}/api/gazebo/set_model_state`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  let data = {};
  try {
    data = await res.json();
  } catch {
    /* ignore */
  }
  if (!res.ok) {
    throw new Error(data.error || `请求失败 (${res.status})`);
  }
  return data;
}

async function fetchJsonOptional(path) {
  const res = await fetch(path);
  if (res.status === 404) {
    return null;
  }
  if (!res.ok) {
    return null;
  }
  return res.json();
}

async function fetchFloors() {
  const data = await fetchJson(`${API_BASE_URL}/api/floors`);
  floors = Array.isArray(data.floors) ? data.floors : [];
}

function parseYaml(text) {
  const lines = text.split("\n");
  const map = {};
  lines.forEach((line) => {
    const idx = line.indexOf(":");
    if (idx <= 0) {
      return;
    }
    const key = line.slice(0, idx).trim();
    const raw = line.slice(idx + 1).trim();
    map[key] = raw;
  });
  return map;
}

function parseOrigin(originText) {
  if (!originText) {
    return [0, 0, 0];
  }
  const normalized = originText.replace(/\[|\]/g, "");
  const items = normalized.split(",").map((s) => Number(s.trim()));
  if (items.length < 2 || Number.isNaN(items[0]) || Number.isNaN(items[1])) {
    return [0, 0, 0];
  }
  return [items[0], items[1], items[2] || 0];
}

function getCanvasCssSize() {
  const w = canvas.clientWidth || mapWrapper.clientWidth || 900;
  const h = canvas.clientHeight || 560;
  return { w, h };
}

function resizeCanvasToDisplay() {
  if (!mapWrapper) {
    return;
  }
  const { w, h } = (() => {
    const rect = mapWrapper.getBoundingClientRect();
    const width = Math.max(320, Math.floor(rect.width));
    const height = Math.max(400, Math.floor(rect.height));
    return { w: width, h: height };
  })();

  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(w * dpr);
  canvas.height = Math.floor(h * dpr);
  canvas.style.width = `${w}px`;
  canvas.style.height = `${h}px`;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  renderScene();
}

/**
 * Build grayscale bitmap from PGM once per floor (avoids per-frame distortion / heavy work).
 */
function buildMapBitmap(pgm) {
  const { width, height, maxVal, data } = pgm;
  const c = document.createElement("canvas");
  c.width = width;
  c.height = height;
  const cctx = c.getContext("2d");
  const imageData = cctx.createImageData(width, height);
  for (let i = 0; i < width * height; i += 1) {
    const v = Math.round((data[i] / maxVal) * 255);
    const p = i * 4;
    imageData.data[p] = v;
    imageData.data[p + 1] = v;
    imageData.data[p + 2] = v;
    imageData.data[p + 3] = 255;
  }
  cctx.putImageData(imageData, 0, 0);
  return c;
}

function resetViewToFit() {
  if (!activePgm || !mapBitmap) {
    return;
  }
  const { w, h } = getCanvasCssSize();
  const mw = activePgm.width;
  const mh = activePgm.height;
  const margin = 0.98;
  viewScale = Math.min((w / mw) * margin, (h / mh) * margin);
  viewPanX = (w - mw * viewScale) / 2;
  viewPanY = (h - mh * viewScale) / 2;
}

function clampViewScale() {
  viewScale = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, viewScale));
}

/** Map image pixel (origin top-left, y down) from world pose */
function worldToMapPixels(pose) {
  if (!activePgm || !activeMeta || !pose) {
    return null;
  }
  const resolution = Number(activeMeta.resolution);
  if (!resolution || Number.isNaN(resolution)) {
    return null;
  }
  const [ox, oy] = parseOrigin(activeMeta.origin);
  const mapX = (pose.x - ox) / resolution;
  const mapY = activePgm.height - (pose.y - oy) / resolution;
  return { mapX, mapY };
}

/** 地图像素 (左上原点、y 向下) → 世界坐标 (与 worldToMapPixels 互逆) */
function mapPixelsToWorld(mapX, mapY) {
  if (!activePgm || !activeMeta) {
    return null;
  }
  const resolution = Number(activeMeta.resolution);
  if (!resolution || Number.isNaN(resolution)) {
    return null;
  }
  const [ox, oy] = parseOrigin(activeMeta.origin);
  const x = ox + mapX * resolution;
  const y = oy + (activePgm.height - mapY) * resolution;
  return { x, y };
}

function mapPixelToScreen(mapX, mapY) {
  return {
    sx: viewPanX + mapX * viewScale,
    sy: viewPanY + mapY * viewScale,
  };
}

function screenToMapPixel(sx, sy) {
  return {
    mapX: (sx - viewPanX) / viewScale,
    mapY: (sy - viewPanY) / viewScale,
  };
}

function drawGridScreen() {
  if (!gridToggle.checked || !activePgm) {
    return;
  }
  const mw = activePgm.width;
  const mh = activePgm.height;
  const { w, h } = getCanvasCssSize();

  ctx.save();
  ctx.strokeStyle = "rgba(34, 197, 94, 0.22)";
  ctx.lineWidth = 1;
  const gridMapPx = Math.max(8, Math.floor(mw / 80));

  const x0 = Math.max(0, viewPanX);
  const y0 = Math.max(0, viewPanY);
  const x1 = Math.min(w, viewPanX + mw * viewScale);
  const y1 = Math.min(h, viewPanY + mh * viewScale);

  const startMx = Math.floor(screenToMapPixel(x0, y0).mapX / gridMapPx) * gridMapPx;
  const startMy = Math.floor(screenToMapPixel(x0, y0).mapY / gridMapPx) * gridMapPx;

  for (let mx = startMx; mx <= mw; mx += gridMapPx) {
    const { sx } = mapPixelToScreen(mx, 0);
    if (sx < x0 - 1 || sx > x1 + 1) {
      continue;
    }
    ctx.beginPath();
    ctx.moveTo(sx, y0);
    ctx.lineTo(sx, y1);
    ctx.stroke();
  }
  for (let my = startMy; my <= mh; my += gridMapPx) {
    const { sy } = mapPixelToScreen(0, my);
    if (sy < y0 - 1 || sy > y1 + 1) {
      continue;
    }
    ctx.beginPath();
    ctx.moveTo(x0, sy);
    ctx.lineTo(x1, sy);
    ctx.stroke();
  }
  ctx.restore();
}

function drawRobotAtScreen(sx, sy, yaw, name, localization) {
  const isLost = localization === "lost";
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-(yaw || 0));
  // robot.svg 美术坐标与地图 yaw 差 90°，顺时针补正（canvas 正角为顺时针）
  ctx.rotate(Math.PI / 2);
  if (isLost) {
    ctx.globalAlpha = 0.55;
  }
  if (robotIconLoaded) {
    const size = 28;
    ctx.drawImage(robotIcon, -size / 2, -size / 2, size, size);
  } else {
    ctx.fillStyle = "#ef4444";
    ctx.strokeStyle = "#fecaca";
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(10, 0);
    ctx.lineTo(-8, -6);
    ctx.lineTo(-4, 0);
    ctx.lineTo(-8, 6);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
  ctx.globalAlpha = 1;

  if (name) {
    ctx.save();
    ctx.font = "12px 'Fira Code', 'Noto Sans SC', sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "bottom";
    const labelY = sy - 20;
    const text = isLost ? `${String(name)} · 丢失` : String(name);
    const metrics = ctx.measureText(text);
    const pad = 4;
    const tw = metrics.width + pad * 2;
    const th = 18;
    const lx = sx - tw / 2;
    const ly = labelY - th + 4;
    ctx.fillStyle = "rgba(15, 23, 42, 0.88)";
    ctx.strokeStyle = isLost ? "rgba(251, 191, 36, 0.75)" : "rgba(34, 197, 94, 0.5)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    if (typeof ctx.roundRect === "function") {
      ctx.roundRect(lx, ly, tw, th, 4);
    } else {
      ctx.rect(lx, ly, tw, th);
    }
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#e5e7eb";
    ctx.fillText(text, sx, labelY - 2);
    ctx.restore();
  }
}

function saveMonitorCheckboxPrefs() {
  try {
    localStorage.setItem(
      MONITOR_CHECKBOXES_KEY,
      JSON.stringify({
        grid: !!(gridToggle && gridToggle.checked),
        scan2d: !!(scan2dToggle && scan2dToggle.checked),
        plannedPath: !!(plannedPathToggle && plannedPathToggle.checked),
        relocPick: !!(relocPickToggle && relocPickToggle.checked),
      })
    );
  } catch {
    /* quota / private mode */
  }
}

function loadAndApplyMonitorCheckboxPrefs() {
  try {
    const raw = localStorage.getItem(MONITOR_CHECKBOXES_KEY);
    if (!raw) {
      return;
    }
    const p = JSON.parse(raw);
    if (!p || typeof p !== "object") {
      return;
    }
    if (gridToggle && typeof p.grid === "boolean") {
      gridToggle.checked = p.grid;
    }
    if (scan2dToggle && typeof p.scan2d === "boolean") {
      scan2dToggle.checked = p.scan2d;
    }
    if (plannedPathToggle && typeof p.plannedPath === "boolean") {
      plannedPathToggle.checked = p.plannedPath;
    }
    if (relocPickToggle && typeof p.relocPick === "boolean") {
      relocPickToggle.checked = p.relocPick;
    }
  } catch {
    /* ignore */
  }
  syncRelocPickCursorClass();
}

function saveFloorPreference(floor) {
  if (!floor) {
    return;
  }
  try {
    localStorage.setItem(FLOOR_PREF_KEY, String(floor));
  } catch {
    /* ignore */
  }
}

function loadMapNamePreference() {
  try {
    return localStorage.getItem(MAP_NAME_PREF_KEY) || "";
  } catch {
    return "";
  }
}

function saveMapNamePreference(v) {
  try {
    localStorage.setItem(MAP_NAME_PREF_KEY, v);
  } catch {
    /* ignore */
  }
}

async function fetchRosNodesStatus() {
  // Avoid hanging fetch when ROS discovery blocks.
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), 900);
  try {
    const res = await fetch(`${API_BASE_URL}/api/ros/nodes/status`, { signal: ctrl.signal });
    if (res.status === 404) {
      return null;
    }
    if (!res.ok) {
      return null;
    }
    return await res.json();
  } catch {
    return null;
  } finally {
    clearTimeout(timer);
  }
}

async function controlRosNode(nodeId, action) {
  const res = await fetch(`${API_BASE_URL}/api/ros/nodes/control`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ node_id: nodeId, action }),
  });
  let payload = {};
  try {
    payload = await res.json();
  } catch {
    /* ignore */
  }
  if (!res.ok) {
    throw new Error(payload.error || `请求失败 (${res.status})`);
  }
  return payload;
}

async function createRosNode(nodeType, params) {
  const res = await fetch(`${API_BASE_URL}/api/ros/nodes/create`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ type: nodeType, ...(params || {}) }),
  });
  let payload = {};
  try {
    payload = await res.json();
  } catch {
    /* ignore */
  }
  if (!res.ok) {
    throw new Error(payload.error || `请求失败 (${res.status})`);
  }
  return payload;
}

async function killDiscoveredRosNode(nodeName) {
  const res = await fetch(`${API_BASE_URL}/api/ros/nodes/discovered/kill`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ node_name: nodeName }),
  });
  let payload = {};
  try {
    payload = await res.json();
  } catch {
    /* ignore */
  }
  if (!res.ok) {
    throw new Error(payload.error || `请求失败 (${res.status})`);
  }
  return payload;
}

function renderRosNodesStatus(data) {
  if (!rosNodesPersistentList || !rosNodesCreatedList || !rosNodesDiscoveredList || !rosNodesSummary) {
    return;
  }

  if (!data) {
    rosNodesSummary.textContent = "状态: 无法连接后端";
    rosNodesPersistentList.innerHTML = "";
    rosNodesCreatedList.innerHTML = "";
    rosNodesDiscoveredList.innerHTML = "";
    return;
  }

  const managed = Array.isArray(data.managed_nodes) ? data.managed_nodes : [];
  const persistentNodes = managed.filter((n) => !!n.persistent);
  const createdNodes = managed.filter((n) => !n.persistent);
  const discovered = Array.isArray(data.discovered_nodes) ? data.discovered_nodes : [];
  const running = managed.filter((n) => n.running).length;

  rosNodesSummary.textContent = `受管节点: ${running}/${managed.length} 运行中 · ROS发现节点: ${discovered.length}`;

  const renderInto = (listEl, nodes) => {
    listEl.innerHTML = "";
    nodes.forEach((node) => {
      const card = document.createElement("article");
      card.className = "ros-node-card";
      const statusClass = node.running ? "running" : "stopped";
      card.innerHTML = `
        <div class="ros-node-head">
          <strong>${node.name || node.id}</strong>
          <span class="ros-node-badge ${statusClass}">${
            node.running ? "运行中" : "已停止"
          }</span>
        </div>
        <div class="ros-node-meta">ID: ${
          node.id
        } · 进程: ${node.process_count || 0}${
        node.note ? ` · ${node.note}` : ""
      }</div>
        <div class="ros-node-actions">
          <button type="button" data-node-action="pause" data-node-id="${
            node.id
          }">暂停</button>
          <button type="button" data-node-action="restart" data-node-id="${
            node.id
          }">重启</button>
        </div>
      `;
      listEl.appendChild(card);
    });
  };

  renderInto(rosNodesPersistentList, persistentNodes);
  renderInto(rosNodesCreatedList, createdNodes);

  rosNodesDiscoveredList.innerHTML = "";
  discovered
    .map((node) => {
      if (typeof node === "string") {
        return { name: node, running: true };
      }
      return node || {};
    })
    .forEach((node) => {
    const card = document.createElement("article");
    card.className = "ros-node-card";
    const nodeName = String(node && node.name ? node.name : "");
    const running = node && node.running !== false;
    const statusClass = running ? "running" : "stopped";
    card.innerHTML = `
      <div class="ros-node-head">
        <strong>${nodeName || "(unknown)"}</strong>
        <span class="ros-node-badge ${statusClass}">${running ? "发现中" : "离线"}</span>
      </div>
      <div class="ros-node-meta">来源: ros2 node list（只读，非受管）</div>
      <div class="ros-node-actions">
        <button
          type="button"
          data-discovered-kill="1"
          data-node-name="${nodeName}"
          ${nodeName ? "" : "disabled"}
        >Kill</button>
      </div>
    `;
    rosNodesDiscoveredList.appendChild(card);
    });
}

function setRosNodeButtonsDisabled(disabled) {
  const lists = [rosNodesPersistentList, rosNodesCreatedList].filter(Boolean);
  lists.forEach((listEl) => {
    listEl.querySelectorAll("button[data-node-action]").forEach((b) => {
      b.disabled = disabled;
    });
  });
  if (rosNodesDiscoveredList) {
    rosNodesDiscoveredList.querySelectorAll("button[data-discovered-kill]").forEach((b) => {
      b.disabled = disabled;
    });
  }
  if (btnRosNodesRefresh) {
    btnRosNodesRefresh.disabled = disabled;
  }
}

async function refreshRosNodesStatus() {
  const data = await fetchRosNodesStatus();
  renderRosNodesStatus(data);
  if (rosNodesError) {
    rosNodesError.textContent = data && data.last_error ? String(data.last_error) : "";
  }
}

function initRosNodesPage() {
  if (!rosNodesPersistentList || !rosNodesCreatedList || !rosNodesDiscoveredList) {
    return;
  }
  if (btnRosNodesRefresh) {
    btnRosNodesRefresh.addEventListener("click", () => {
      refreshRosNodesStatus().catch((err) => {
        if (rosNodesError) {
          rosNodesError.textContent = err.message || String(err);
        }
      });
    });
  }

  const bindNodeActions = (listEl) => {
    if (!listEl) {
      return;
    }
    listEl.addEventListener("click", async (ev) => {
      const btn = ev.target.closest("button[data-node-action]");
      if (!btn) {
        return;
      }
      const nodeId = btn.dataset.nodeId;
      const action = btn.dataset.nodeAction;
      if (!nodeId || !action) {
        return;
      }
      setRosNodeButtonsDisabled(true);
      if (rosNodesError) {
        rosNodesError.textContent = "";
      }
      try {
        await controlRosNode(nodeId, action);
        appendLog(`ROS节点 ${nodeId} ${action} 成功`);
      } catch (err) {
        if (rosNodesError) {
          rosNodesError.textContent = err.message || String(err);
        }
        appendLog(`ROS节点 ${nodeId} ${action} 失败: ${err.message || err}`);
      } finally {
        setRosNodeButtonsDisabled(false);
        refreshRosNodesStatus().catch(() => {});
      }
    });
  };
  bindNodeActions(rosNodesPersistentList);
  bindNodeActions(rosNodesCreatedList);
  rosNodesDiscoveredList.addEventListener("click", async (ev) => {
    const btn = ev.target.closest("button[data-discovered-kill]");
    if (!btn) {
      return;
    }
    const nodeName = String(btn.dataset.nodeName || "").trim();
    if (!nodeName) {
      return;
    }
    const yes = window.confirm(`确认 Kill 节点 ${nodeName} ?`);
    if (!yes) {
      return;
    }
    setRosNodeButtonsDisabled(true);
    if (rosNodesError) {
      rosNodesError.textContent = "";
    }
    try {
      await killDiscoveredRosNode(nodeName);
      appendLog(`Kill discovered 节点 ${nodeName} 请求已发送`);
    } catch (err) {
      if (rosNodesError) {
        rosNodesError.textContent = err.message || String(err);
      }
      appendLog(`Kill discovered 节点失败: ${err.message || err}`);
    } finally {
      setRosNodeButtonsDisabled(false);
      refreshRosNodesStatus().catch(() => {});
    }
  });

  refreshRosNodesStatus().catch(() => {});
  if (rosNodeCreateForm) {
    rosNodeCreateForm.addEventListener("submit", async (e) => {
      e.preventDefault();

      const nodeType = rosNodeCreateTypeSelect ? rosNodeCreateTypeSelect.value : "";
      if (!nodeType) {
        if (rosNodesError) {
          rosNodesError.textContent = "请选择节点类型";
        }
        return;
      }

      let params = {};
      if (nodeType === "fake_pub") {
        const rn = (rosNodeRobotName && rosNodeRobotName.value ? rosNodeRobotName.value : "")
          .trim();
        const cm = (rosNodeCurrentMap && rosNodeCurrentMap.value ? rosNodeCurrentMap.value : "")
          .trim();
        if (!rn || !cm) {
          if (rosNodesError) {
            rosNodesError.textContent = "请填写 robot_name 与初始楼层 id";
          }
          return;
        }
        params = { robot_name: rn, initial_floor: cm };
      } else if (nodeType === "slam_bringup_mapping") {
        const rn = (
          rosNodeSlamRobotName && rosNodeSlamRobotName.value
            ? rosNodeSlamRobotName.value
            : ""
        ).trim();
        params = { robot_name: rn || "sim_robot" };
      }

      if (btnRosNodeCreate) {
        btnRosNodeCreate.disabled = true;
      }
      if (rosNodesError) {
        rosNodesError.textContent = "";
      }

      try {
        const payload = await createRosNode(nodeType, params);
        appendLog(`创建节点 ${payload.node_id || ""} 并启动`);
        await refreshRosNodesStatus();
      } catch (err) {
        if (rosNodesError) {
          rosNodesError.textContent = err.message || String(err);
        }
        appendLog(`创建节点失败: ${err.message || err}`);
      } finally {
        if (btnRosNodeCreate) {
          btnRosNodeCreate.disabled = false;
        }
      }
    });

    const syncFields = () => {
      const nodeType = rosNodeCreateTypeSelect ? rosNodeCreateTypeSelect.value : "";
      const showFake = nodeType === "fake_pub";
      if (rosNodeFieldsFakePub) {
        rosNodeFieldsFakePub.style.display = showFake ? "" : "none";
      }
      if (rosNodeFieldsSlamMapping) {
        rosNodeFieldsSlamMapping.style.display = showFake ? "none" : "";
      }
    };

    if (rosNodeFieldsFakePub && rosNodeFieldsSlamMapping) {
      syncFields();
      if (rosNodeCreateTypeSelect) {
        rosNodeCreateTypeSelect.addEventListener("change", () => syncFields());
      }
    }
  }
  if (rosNodesPollTimer) {
    clearInterval(rosNodesPollTimer);
  }
  rosNodesPollTimer = setInterval(() => {
    refreshRosNodesStatus().catch(() => {});
  }, 2500);
}

function loadFloorPreference() {
  try {
    const v = localStorage.getItem(FLOOR_PREF_KEY);
    return v ? String(v).trim() : "";
  } catch {
    return "";
  }
}

function syncRelocPickCursorClass() {
  if (!mapWrapper || !relocPickToggle) {
    return;
  }
  mapWrapper.classList.toggle("pick-mode", relocPickToggle.checked);
}

/** 一次选点流程结束：清状态、取消「地图选位姿」勾选 */
function exitRelocPickModeAfterDone(message) {
  relocPickStep = 0;
  relocPickAnchorWorld = null;
  relocPickHoverSx = null;
  relocPickHoverSy = null;
  if (relocMessage && message) {
    relocMessage.textContent = message;
  }
  if (relocPickToggle) {
    relocPickToggle.checked = false;
    syncRelocPickCursorClass();
  }
  saveMonitorCheckboxPrefs();
  renderScene();
}

function resetRelocPickState(hint) {
  relocPickStep = 0;
  relocPickAnchorWorld = null;
  relocPickHoverSx = null;
  relocPickHoverSy = null;
  if (relocMessage && hint) {
    relocMessage.textContent = hint;
  }
  renderScene();
}

/**
 * @returns {boolean} 是否已处理（阻止地图拖拽）
 */
function handleRelocMapClick(ev) {
  if (!relocPickToggle || !relocPickToggle.checked || !activePgm || !activeMeta) {
    return false;
  }
  const rect = canvas.getBoundingClientRect();
  const sx = ev.clientX - rect.left;
  const sy = ev.clientY - rect.top;
  const { w, h } = getCanvasCssSize();
  if (sx < 0 || sy < 0 || sx > w || sy > h) {
    return false;
  }
  const { mapX, mapY } = screenToMapPixel(sx, sy);
  const mw = activePgm.width;
  const mh = activePgm.height;
  if (mapX < 0 || mapX > mw || mapY < 0 || mapY > mh) {
    return false;
  }
  const world = mapPixelsToWorld(mapX, mapY);
  if (!world) {
    return false;
  }

  if (relocPickStep === 0) {
    relocPickAnchorWorld = { x: world.x, y: world.y };
    relocPickStep = 1;
    if (relocX) {
      relocX.value = String(Number(world.x.toFixed(3)));
    }
    if (relocY) {
      relocY.value = String(Number(world.y.toFixed(3)));
    }
    if (relocMessage) {
      relocMessage.textContent = "已定点，再点击地图设置朝向（或「跳过朝向」）";
    }
    renderScene();
    return true;
  }

  const yaw = Math.atan2(world.y - relocPickAnchorWorld.y, world.x - relocPickAnchorWorld.x);
  if (relocYaw) {
    relocYaw.value = String(Number(yaw.toFixed(4)));
  }
  exitRelocPickModeAfterDone("已设置朝向，可下发重定位");
  return true;
}

function drawRelocPickOverlay() {
  if (!activePgm || relocPickStep !== 1 || !relocPickAnchorWorld) {
    return;
  }
  const pix = worldToMapPixels({ x: relocPickAnchorWorld.x, y: relocPickAnchorWorld.y });
  if (!pix) {
    return;
  }
  const { sx, sy } = mapPixelToScreen(pix.mapX, pix.mapY);
  ctx.save();
  ctx.fillStyle = "rgba(250, 204, 21, 0.95)";
  ctx.strokeStyle = "#f59e0b";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(sx, sy, 7, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  if (relocPickHoverSx != null && relocPickHoverSy != null) {
    ctx.strokeStyle = "rgba(251, 191, 36, 0.95)";
    ctx.lineWidth = 2.5;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(relocPickHoverSx, relocPickHoverSy);
    ctx.stroke();
    ctx.setLineDash([]);
  }
  ctx.restore();
}

function drawPlannedPathOnMap(points) {
  if (!activePgm || !points || points.length < 2) {
    return;
  }
  ctx.save();
  ctx.strokeStyle = "rgba(96, 165, 250, 0.9)";
  ctx.lineWidth = 2;
  ctx.lineJoin = "round";
  ctx.beginPath();
  let started = false;
  points.forEach((pt) => {
    if (!pt || pt.length < 2) {
      return;
    }
    const pix = worldToMapPixels({ x: pt[0], y: pt[1] });
    if (!pix) {
      return;
    }
    const { sx, sy } = mapPixelToScreen(pix.mapX, pix.mapY);
    if (!started) {
      ctx.moveTo(sx, sy);
      started = true;
    } else {
      ctx.lineTo(sx, sy);
    }
  });
  ctx.stroke();
  ctx.restore();
}

function drawScan2dOnMap(data) {
  if (!activePgm || !data || !data.hits || data.hits.length === 0) {
    return;
  }
  ctx.save();
  // scan_2d：地图坐标系下的命中点，红色圆点（半径随缩放略变）
  const ptRadius = Math.max(1.25, Math.min(3.5, viewScale * 0.2));
  ctx.fillStyle = "#ef4444";
  data.hits.forEach((hit) => {
    if (!hit || hit.length < 2) {
      return;
    }
    const pix = worldToMapPixels({ x: hit[0], y: hit[1] });
    if (!pix) {
      return;
    }
    const { sx, sy } = mapPixelToScreen(pix.mapX, pix.mapY);
    ctx.beginPath();
    ctx.arc(sx, sy, ptRadius, 0, Math.PI * 2);
    ctx.fill();
  });
  ctx.restore();
}

/** Floor id from pose snapshot (`active_floor`); bridge merges RobotStatus topics like /robot1/robot_status. */
function snapshotRobotFloor(r) {
  if (!r) return "";
  const a = r.active_floor;
  if (a != null && String(a).trim() !== "") return String(a).trim();
  const legacy = r.current_map;
  if (legacy != null && String(legacy).trim() !== "") return String(legacy).trim();
  return "";
}

function escapeHtml(s) {
  return String(s)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

async function fetchRobotStatusCache() {
  try {
    const data = await fetchJson(`${API_BASE_URL}/api/robot/status/cache`);
    const items = data && Array.isArray(data.items) ? data.items : [];
    robotStatusCacheItems = items;
  } catch {
    robotStatusCacheItems = [];
  }
}

function mergePresenceRows() {
  const live =
    latestSnapshot && Array.isArray(latestSnapshot.robots) ? latestSnapshot.robots : [];
  const liveById = new Map(live.map((r) => [String(r.id), r]));
  const ids = new Set(liveById.keys());
  robotStatusCacheItems.forEach((it) => {
    const id = String(it.robot_id || "").trim();
    if (id) ids.add(id);
  });
  const rows = [];
  ids.forEach((id) => {
    const liveR = liveById.get(id);
    const cache = robotStatusCacheItems.find((x) => String(x.robot_id) === id);
    let online = false;
    if (liveR) {
      online = liveR.heartbeat_online !== false;
    }
    const name = (liveR && (liveR.name || liveR.id)) || (cache && cache.robot_name) || id;
    const floor =
      (liveR && snapshotRobotFloor(liveR)) ||
      (cache && String(cache.current_map || "").trim()) ||
      "";
    rows.push({ id, name, floor, online });
  });
  rows.sort((a, b) => a.id.localeCompare(b.id));
  return rows;
}

function updateRobotPresenceTriggerSummary() {
  if (!robotPresenceSummary) {
    return;
  }
  const rows = mergePresenceRows();
  const onlineN = rows.filter((r) => r.online).length;
  const total = rows.length;
  if (total === 0) {
    robotPresenceSummary.textContent = "暂无数据 · 点击展开";
    return;
  }
  robotPresenceSummary.textContent = `${onlineN} 在线 · ${total - onlineN} 离线`;
}

function renderRobotPresencePanel() {
  if (!robotPresenceList || !robotPresenceEmpty) {
    return;
  }
  const rows = mergePresenceRows();
  const onlineN = rows.filter((r) => r.online).length;
  const total = rows.length;
  if (robotPresenceSummary) {
    robotPresenceSummary.textContent =
      total === 0 ? "无数据" : `${onlineN} 在线 · ${total - onlineN} 离线`;
  }

  if (rows.length === 0) {
    robotPresenceList.innerHTML = "";
    robotPresenceEmpty.hidden = false;
    return;
  }
  robotPresenceEmpty.hidden = true;
  robotPresenceList.innerHTML = rows
    .map((r) => {
      const badge = r.online
        ? '<span class="robot-presence-badge robot-presence-badge--online">在线</span>'
        : '<span class="robot-presence-badge robot-presence-badge--offline">离线</span>';
      const floorLine = r.floor
        ? `<div class="robot-presence-list__meta">${escapeHtml(r.floor)}</div>`
        : "";
      return `<li>
        <div>
          <div class="robot-presence-list__id">${escapeHtml(r.name)}</div>
          <div class="robot-presence-list__meta">${escapeHtml(r.id)}</div>
          ${floorLine}
        </div>
        ${badge}
      </li>`;
    })
    .join("");
}

function setRobotPresenceOpen(open) {
  robotPresencePanelOpen = open;
  if (!robotPresencePanel || !btnRobotPresence) {
    return;
  }
  robotPresencePanel.hidden = !open;
  btnRobotPresence.setAttribute("aria-expanded", open ? "true" : "false");
  if (robotPresenceCacheTimer) {
    clearInterval(robotPresenceCacheTimer);
    robotPresenceCacheTimer = null;
  }
  if (open) {
    fetchRobotStatusCache()
      .then(() => {
        renderRobotPresencePanel();
      })
      .catch(() => {
        renderRobotPresencePanel();
      });
    robotPresenceCacheTimer = setInterval(() => {
      fetchRobotStatusCache()
        .then(renderRobotPresencePanel)
        .catch(() => {});
    }, 4000);
  } else {
    updateRobotPresenceTriggerSummary();
  }
}

function initRobotPresenceUi() {
  if (btnRobotPresence && robotPresencePanel) {
    btnRobotPresence.addEventListener("click", (ev) => {
      ev.stopPropagation();
      setRobotPresenceOpen(!robotPresencePanelOpen);
    });
  }
  document.addEventListener("click", (ev) => {
    if (!robotPresencePanelOpen || !robotPresenceAnchor) {
      return;
    }
    if (robotPresenceAnchor.contains(ev.target)) {
      return;
    }
    setRobotPresenceOpen(false);
  });
  document.addEventListener("keydown", (ev) => {
    if (ev.key === "Escape" && robotPresencePanelOpen) {
      setRobotPresenceOpen(false);
    }
  });
  updateRobotPresenceTriggerSummary();
}

function getRobotsOnCurrentMap() {
  if (!latestSnapshot || !Array.isArray(latestSnapshot.robots) || !activeFloor) {
    return [];
  }
  if (isMappingFloor(activeFloor)) {
    const rid = robotIdFromMappingFloor(activeFloor);
    if (!rid) {
      return [];
    }
    return latestSnapshot.robots.filter((r) => r && r.id === rid);
  }
  return latestSnapshot.robots.filter((r) => snapshotRobotFloor(r) === activeFloor);
}

function renderScene() {
  const { w, h } = getCanvasCssSize();
  ctx.fillStyle = "#020617";
  ctx.fillRect(0, 0, w, h);

  if (!mapBitmap || !activePgm) {
    return;
  }

  const mw = activePgm.width;
  const mh = activePgm.height;

  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapBitmap, viewPanX, viewPanY, mw * viewScale, mh * viewScale);

  drawGridScreen();

  const robotsHere = getRobotsOnCurrentMap();
  if (plannedPathToggle && plannedPathToggle.checked) {
    robotsHere.forEach((r) => {
      const pathData = latestPathByRobot[r.id];
      if (pathData && pathData.points && pathData.points.length) {
        drawPlannedPathOnMap(pathData.points);
      }
    });
  }
  if (scan2dToggle && scan2dToggle.checked) {
    robotsHere.forEach((r) => {
      const sd = latestScanByRobot[r.id];
      if (sd && sd.hits && sd.hits.length) {
        drawScan2dOnMap(sd);
      }
    });
  }

  robotsHere.forEach((r) => {
    const pixels = worldToMapPixels(r.pose);
    if (!pixels) {
      return;
    }
    const { mapX, mapY } = pixels;
    if (mapX < 0 || mapX > mw || mapY < 0 || mapY > mh) {
      return;
    }
    const { sx, sy } = mapPixelToScreen(mapX, mapY);
    drawRobotAtScreen(
      sx,
      sy,
      r.pose && r.pose.yaw,
      r.name || r.id || "robot",
      r.localization
    );
  });

  drawRelocPickOverlay();
}

function updateRobotStatus() {
  try {
    if (!robotStatus) {
      return;
    }
    const here = getRobotsOnCurrentMap();
    if (!activeFloor) {
      robotStatus.textContent = "定位: 未选择地图";
      return;
    }
    if (isMappingFloor(activeFloor)) {
      const rid = robotIdFromMappingFloor(activeFloor);
      const topic = rid ? `/${rid}/mapping` : "";
      if (here.length === 0) {
        robotStatus.textContent = `建图 ${activeFloor} · ${topic} · 无该机器人位姿`;
      } else {
        const parts = here.map((r) => {
          const p = r.pose || {};
          const nm = r.name || r.id;
          const loc = r.localization === "lost" ? "丢失" : "OK";
          return `${nm} [${loc}] (${p.x?.toFixed?.(2) ?? "?"},${p.y?.toFixed?.(2) ?? "?"})`;
        });
        robotStatus.textContent = `建图 (${here.length}): ${parts.join(" · ")}`;
      }
      return;
    }
    if (!latestSnapshot) {
      robotStatus.textContent = `地图 ${activeFloor} · 无实时数据`;
      return;
    }
    if (here.length === 0) {
      robotStatus.textContent = `地图 ${activeFloor} · 当前无机器人定位`;
      return;
    }
    const parts = here.map((r) => {
      const p = r.pose || {};
      const nm = r.name || r.id;
      const loc = r.localization === "lost" ? "丢失" : "OK";
      return `${nm} [${loc}] (${p.x?.toFixed?.(2) ?? "?"},${p.y?.toFixed?.(2) ?? "?"})`;
    });
    robotStatus.textContent = `本图定位 (${here.length}): ${parts.join(" · ")}`;
    if (relocRobotId && relocRobotId.dataset.userEdited !== "1") {
      const pick = here[0] || (latestSnapshot?.robots || [])[0];
      if (pick?.id) {
        relocRobotId.value = pick.id;
      }
    }
  } finally {
    updateRobotPresenceTriggerSummary();
    if (robotPresencePanelOpen) {
      renderRobotPresencePanel();
    }
  }
}

function fillRelocPoseFromScreenRobot() {
  if (!relocX || !relocY || !relocYaw) {
    return;
  }
  const here = getRobotsOnCurrentMap();
  const rid = relocRobotId?.value?.trim();
  const r =
    here.find((bot) => bot.id === rid) ||
    here[0] ||
    (latestSnapshot?.robots || []).find((bot) => bot.id === rid) ||
    (latestSnapshot?.robots || [])[0];
  if (!r?.pose) {
    if (relocMessage) {
      relocMessage.textContent = "无可用位姿，请手填 x/y/yaw";
    }
    return;
  }
  const p = r.pose;
  relocX.value = String(p.x ?? 0);
  relocY.value = String(p.y ?? 0);
  relocYaw.value = String(p.yaw ?? 0);
  if (relocMessage) {
    relocMessage.textContent = `已填入 ${r.name || r.id} 位姿`;
  }
}

function refreshMetaPanel() {
  if (!activePgm || !activeMeta) {
    return;
  }
  const here = getRobotsOnCurrentMap();
  mapMeta.textContent = JSON.stringify(
    {
      floor: activeFloor,
      source: isMappingFloor(activeFloor)
        ? `mapping-live-/${robotIdFromMappingFloor(activeFloor) || "?"}/mapping`
        : "backend-api",
      width: activePgm.width,
      height: activePgm.height,
      resolution: activeMeta.resolution,
      origin: activeMeta.origin,
      occupied_thresh: activeMeta.occupied_thresh,
      free_thresh: activeMeta.free_thresh,
      robots_on_this_map: here.map((r) => ({
        id: r.id,
        name: r.name,
      })),
      view: { scale: Number(viewScale.toFixed(4)), panX: viewPanX, panY: viewPanY },
    },
    null,
    2
  );
}

async function loadFloorMap(floor) {
  stopMapLivePolling();
  activeMappingRobotId = null;
  updateMappingToolbar();
  if (isMappingFloor(floor)) {
    const rid = robotIdFromMappingFloor(floor);
    if (!rid) {
      mapStatus.textContent = "无效的建图楼层名";
      return;
    }
    mapStatus.textContent = `正在连接 /${rid}/mapping …`;
    activePgm = null;
    mapBitmap = null;
    activeFloor = floor;
    activeMappingRobotId = rid;
    if (relocRobotId && !relocRobotId.dataset.userEdited) {
      relocRobotId.value = rid;
    }
    startMapLivePolling();
    updateMappingToolbar();
    updateRobotStatus();
    renderScene();
    appendLog(`建图视图: ${floor} → /${rid}/mapping`);
    return;
  }
  mapStatus.textContent = `正在加载 ${floor}...`;
  try {
    const mapData = await fetchJson(`${API_BASE_URL}/api/maps/${encodeURIComponent(floor)}`);
    const pgm = mapData.pgm;
    const yamlText = mapData.yaml;
    if (!pgm || typeof yamlText !== "string") {
      throw new Error("后端返回的地图数据格式不正确");
    }
    activeFloor = floor;
    activePgm = pgm;
    activeMeta = parseYaml(yamlText);
    mapBitmap = buildMapBitmap(pgm);
    resetViewToFit();
    renderScene();
    updateRobotStatus();
    refreshMetaPanel();
    updateMappingToolbar();
    mapStatus.textContent = `${floor} 加载完成 · 滚轮缩放 · 拖拽平移`;
  } catch (err) {
    mapStatus.textContent = `加载失败: ${err.message}`;
    mapBitmap = null;
    activePgm = null;
    const { w, h } = getCanvasCssSize();
    ctx.fillStyle = "#020617";
    ctx.fillRect(0, 0, w, h);
    updateMappingToolbar();
  }
}

async function fetchPoseOnce() {
  try {
    latestSnapshot = await fetchJson(`${API_BASE_URL}/api/robot/pose`);
    updateRobotStatus();
    renderScene();
    refreshMetaPanel();
  } catch (err) {
    latestSnapshot = null;
    updateRobotStatus();
  }
}

function startPoseStream() {
  const eventSource = new EventSource(`${API_BASE_URL}/api/robot/pose/stream`);
  eventSource.onmessage = (event) => {
    latestSnapshot = JSON.parse(event.data);
    updateRobotStatus();
    renderScene();
    refreshMetaPanel();
  };
  eventSource.onerror = () => {
    eventSource.close();
    setInterval(fetchPoseOnce, 1000);
  };
}

function onWheel(ev) {
  ev.preventDefault();
  if (!activePgm || !mapBitmap) {
    return;
  }
  const rect = canvas.getBoundingClientRect();
  const sx = ev.clientX - rect.left;
  const sy = ev.clientY - rect.top;
  const factor = ev.deltaY < 0 ? 1.12 : 1 / 1.12;
  const oldScale = viewScale;
  viewScale *= factor;
  clampViewScale();
  const { mapX, mapY } = screenToMapPixel(sx, sy);
  viewPanX = sx - mapX * viewScale;
  viewPanY = sy - mapY * viewScale;
  if (oldScale !== viewScale) {
    renderScene();
    refreshMetaPanel();
  }
}

function onMouseDown(ev) {
  if (ev.button !== 0) {
    return;
  }
  const pickOn = relocPickToggle && relocPickToggle.checked;
  if (pickOn && activePgm && !ev.shiftKey && handleRelocMapClick(ev)) {
    return;
  }
  isDragging = true;
  dragLastX = ev.clientX;
  dragLastY = ev.clientY;
  canvas.style.cursor = "grabbing";
}

function onMouseMove(ev) {
  if (
    relocPickToggle &&
    relocPickToggle.checked &&
    relocPickStep === 1 &&
    relocPickAnchorWorld
  ) {
    const rect = canvas.getBoundingClientRect();
    const sx = ev.clientX - rect.left;
    const sy = ev.clientY - rect.top;
    const { w, h } = getCanvasCssSize();
    if (sx >= 0 && sy >= 0 && sx <= w && sy <= h) {
      if (relocPickHoverSx !== sx || relocPickHoverSy !== sy) {
        relocPickHoverSx = sx;
        relocPickHoverSy = sy;
        renderScene();
      }
    }
  }
  if (!isDragging) {
    return;
  }
  const dx = ev.clientX - dragLastX;
  const dy = ev.clientY - dragLastY;
  dragLastX = ev.clientX;
  dragLastY = ev.clientY;
  viewPanX += dx;
  viewPanY += dy;
  renderScene();
  refreshMetaPanel();
}

function onMouseUp() {
  isDragging = false;
  canvas.style.cursor = "";
}

function stopSensorPolling() {
  if (sensorPollTimer) {
    clearInterval(sensorPollTimer);
    sensorPollTimer = null;
  }
}

function stopMapLivePolling() {
  if (mapLiveTimer) {
    clearInterval(mapLiveTimer);
    mapLiveTimer = null;
  }
  mapLiveInitializedView = false;
}

async function applyLiveMappingFrame() {
  const rid = activeMappingRobotId;
  if (!rid) {
    return;
  }
  const url = `${API_BASE_URL}/api/mapping/live?robot_id=${encodeURIComponent(rid)}`;
  const data = await fetchJsonOptional(url);
  if (!data || !data.available) {
    mapStatus.textContent =
      (data && data.reason) ||
      `等待 /${rid}/mapping（需向该 topic 发布 OccupancyGrid，且 ROS 桥运行中）…`;
    return;
  }
  const raw = atob(data.data_b64);
  const bytes = new Uint8Array(raw.length);
  for (let i = 0; i < raw.length; i += 1) {
    bytes[i] = raw.charCodeAt(i);
  }
  const pgm = {
    width: data.width,
    height: data.height,
    maxVal: 255,
    data: Array.from(bytes),
  };
  activePgm = pgm;
  activeMeta = {
    resolution: String(data.resolution),
    origin: `[${data.origin[0]}, ${data.origin[1]}, ${data.origin[2]}]`,
    occupied_thresh: "0.65",
    free_thresh: "0.196",
  };
  mapBitmap = buildMapBitmap(pgm);
  if (!mapLiveInitializedView) {
    resetViewToFit();
    mapLiveInitializedView = true;
  }
  renderScene();
  refreshMetaPanel();
  mapStatus.textContent = `建图 ${rid} ${data.width}×${data.height} · res ${data.resolution} m/cell`;
}

function startMapLivePolling() {
  stopMapLivePolling();
  mapLiveInitializedView = false;
  applyLiveMappingFrame().catch(() => {});
  mapLiveTimer = setInterval(() => {
    applyLiveMappingFrame().catch(() => {});
  }, 400);
}

async function postSaveMap(mapName) {
  const res = await fetch(`${API_BASE_URL}/api/mapping/save`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ map_name: mapName }),
  });
  let payload = {};
  try {
    payload = await res.json();
  } catch {
    /* ignore */
  }
  if (!res.ok) {
    throw new Error(payload.error || `保存失败 (${res.status})`);
  }
  return payload;
}

function startSensorPolling() {
  stopSensorPolling();
  const wantScan = scan2dToggle && scan2dToggle.checked;
  const wantPath = plannedPathToggle && plannedPathToggle.checked;
  if (!wantScan && !wantPath) {
    return;
  }
  sensorPollTimer = setInterval(async () => {
    const here = getRobotsOnCurrentMap();
    if (here.length === 0) {
      return;
    }
    const ws = scan2dToggle && scan2dToggle.checked;
    const wp = plannedPathToggle && plannedPathToggle.checked;
    await Promise.all(
      here.map(async (r) => {
        const id = encodeURIComponent(r.id);
        if (ws) {
          const d = await fetchJsonOptional(`${API_BASE_URL}/api/robot/${id}/scan_2d`);
          if (d) {
            latestScanByRobot[r.id] = d;
          } else {
            delete latestScanByRobot[r.id];
          }
        }
        if (wp) {
          const d = await fetchJsonOptional(`${API_BASE_URL}/api/robot/${id}/planned_path`);
          if (d) {
            latestPathByRobot[r.id] = d;
          } else {
            delete latestPathByRobot[r.id];
          }
        }
      })
    );
    renderScene();
  }, 250);
}

function bindMapInteractions() {
  canvas.addEventListener("wheel", onWheel, { passive: false });
  canvas.addEventListener("mousedown", onMouseDown);
  window.addEventListener("mousemove", onMouseMove);
  window.addEventListener("mouseup", onMouseUp);
  canvas.addEventListener("mouseleave", onMouseUp);
}

function initSettings() {
  const defaults = {
    maxSpeed: 1.2,
    angularSpeed: 0.8,
    safetyDistance: 0.6,
    refreshInterval: 500,
  };
  const saved = JSON.parse(localStorage.getItem(SETTINGS_KEY) || "null") || defaults;

  Object.keys(defaults).forEach((key) => {
    settingsForm.elements[key].value = saved[key];
  });

  settingsForm.addEventListener("submit", (e) => {
    e.preventDefault();
    const payload = {
      maxSpeed: Number(settingsForm.elements.maxSpeed.value),
      angularSpeed: Number(settingsForm.elements.angularSpeed.value),
      safetyDistance: Number(settingsForm.elements.safetyDistance.value),
      refreshInterval: Number(settingsForm.elements.refreshInterval.value),
    };
    localStorage.setItem(SETTINGS_KEY, JSON.stringify(payload));
    settingsMessage.textContent = `已保存: ${new Date().toLocaleString()}`;
    appendLog(`参数已更新 ${JSON.stringify(payload)}`);
  });
}

function getLogs() {
  return JSON.parse(localStorage.getItem(LOGS_KEY) || "[]");
}

function setLogs(logs) {
  localStorage.setItem(LOGS_KEY, JSON.stringify(logs));
}

function appendLog(message) {
  const logs = getLogs();
  logs.unshift(`[${new Date().toLocaleTimeString()}] ${message}`);
  setLogs(logs.slice(0, 200));
  renderLogs();
}

function renderLogs() {
  const logs = getLogs();
  logList.innerHTML = "";
  logs.forEach((item) => {
    const li = document.createElement("li");
    li.textContent = item;
    logList.appendChild(li);
  });
}

function initLogs() {
  if (getLogs().length === 0) {
    appendLog("系统启动");
    appendLog("监控模块就绪");
  } else {
    renderLogs();
  }

  addLogBtn.addEventListener("click", () => {
    appendLog("收到机器人心跳，状态正常");
  });

  clearLogBtn.addEventListener("click", () => {
    setLogs([]);
    renderLogs();
  });
}

async function initMonitor() {
  await resolveApiBaseUrl();
  loadAndApplyMonitorCheckboxPrefs();

  bindMapInteractions();

  if (btnResetView) {
    btnResetView.addEventListener("click", () => {
      resetViewToFit();
      renderScene();
      refreshMetaPanel();
      appendLog("地图视图已重置");
    });
  }

  if (relocRobotId) {
    relocRobotId.addEventListener("input", () => {
      relocRobotId.dataset.userEdited = "1";
    });
  }
  if (btnRelocFillPose) {
    btnRelocFillPose.addEventListener("click", () => {
      fillRelocPoseFromScreenRobot();
    });
  }

  syncRelocPickCursorClass();
  if (relocPickToggle) {
    relocPickToggle.addEventListener("change", () => {
      syncRelocPickCursorClass();
      if (!relocPickToggle.checked) {
        resetRelocPickState("");
      }
      saveMonitorCheckboxPrefs();
    });
  }
  if (btnRelocSkipHeading) {
    btnRelocSkipHeading.addEventListener("click", () => {
      if (relocPickStep !== 1) {
        return;
      }
      exitRelocPickModeAfterDone("已定点，朝向请手填或保持原值");
    });
  }
  if (btnRelocClearPick) {
    btnRelocClearPick.addEventListener("click", () => {
      exitRelocPickModeAfterDone("已清除地图选点");
    });
  }

  const targetMapName = () => {
    const fv = floorSelect && floorSelect.value ? floorSelect.value.trim() : "";
    if (isMappingFloor(fv)) {
      return mapNameInput && mapNameInput.value ? mapNameInput.value.trim() : "";
    }
    if (fv) {
      return fv;
    }
    return mapNameInput && mapNameInput.value ? mapNameInput.value.trim() : "";
  };

  async function runRelocCommand(mode) {
    if (!relocMessage) {
      return;
    }
    const rid = relocRobotId && relocRobotId.value ? relocRobotId.value.trim() : "";
    if (!rid) {
      relocMessage.textContent = "请填写机器人 ID";
      return;
    }
    const payload = { mode, robot_id: rid };
    if (mode === "map_only" || mode === "both") {
      const mn = targetMapName();
      if (!mn) {
        relocMessage.textContent =
          isMappingFloor(floorSelect && floorSelect.value)
            ? "建图模式下请填写「保存为」地图名（切图经 robot_status 的 current_map 字段下发）"
            : "请填写「保存为」地图名或选择已保存楼层";
        return;
      }
      payload.map_name = mn;
    }
    if (mode === "pose_only" || mode === "both") {
      const x = parseFloat(relocX && relocX.value);
      const y = parseFloat(relocY && relocY.value);
      const yaw = parseFloat((relocYaw && relocYaw.value) || "0");
      if (Number.isNaN(x) || Number.isNaN(y)) {
        relocMessage.textContent = "请填写 x、y（米），可用「填入位姿」";
        return;
      }
      payload.x = x;
      payload.y = y;
      payload.yaw = Number.isNaN(yaw) ? 0 : yaw;
    }
    relocMessage.textContent = "发送中…";
    try {
      await postRobotCommand(payload);
      relocMessage.textContent = "已下发到 ROS";
      appendLog(`下发 ${mode} → ${rid}`);
    } catch (err) {
      relocMessage.textContent = err.message || String(err);
    }
  }

  if (btnRelocMapOnly) {
    btnRelocMapOnly.addEventListener("click", () => runRelocCommand("map_only"));
  }
  if (btnRelocPoseOnly) {
    btnRelocPoseOnly.addEventListener("click", () => runRelocCommand("pose_only"));
  }
  if (btnRelocBoth) {
    btnRelocBoth.addEventListener("click", () => runRelocCommand("both"));
  }

  if (typeof ResizeObserver !== "undefined" && mapWrapper) {
    const ro = new ResizeObserver(() => {
      resizeCanvasToDisplay();
    });
    ro.observe(mapWrapper);
  } else {
    window.addEventListener("resize", resizeCanvasToDisplay);
  }

  await fetchFloors();
  addFloorOptions();
  const preferredFloor = loadFloorPreference();
  const initialFloor = preferredFloor && floors.includes(preferredFloor) ? preferredFloor : floors[0];
  floorSelect.value = initialFloor;
  if (mapNameInput) {
    const sn = loadMapNamePreference();
    if (sn) {
      mapNameInput.value = sn;
    }
    mapNameInput.addEventListener("change", () => {
      saveMapNamePreference(mapNameInput.value.trim());
      updateRobotStatus();
      renderScene();
    });
    if (isMappingFloor(initialFloor)) {
      updateMappingToolbar();
    }
  }
  if (btnSaveMap) {
    btnSaveMap.addEventListener("click", async () => {
      const name = mapNameInput && mapNameInput.value ? mapNameInput.value.trim() : "";
      if (!name) {
        mapStatus.textContent = "保存失败: 请先填写地图名";
        return;
      }
      btnSaveMap.disabled = true;
      mapStatus.textContent = "正在保存地图到 map/ …";
      try {
        await postSaveMap(name);
        saveMapNamePreference(name);
        mapStatus.textContent = `已保存 map/${name}/`;
        appendLog(`地图已保存: ${name}`);
        await fetchFloors();
        addFloorOptions();
        if (floors.includes(name)) {
          floorSelect.value = name;
          await loadFloorMap(name);
          saveFloorPreference(name);
        }
      } catch (err) {
        mapStatus.textContent = `保存失败: ${err.message || err}`;
        appendLog(`保存地图失败: ${err.message || err}`);
      } finally {
        btnSaveMap.disabled = false;
      }
    });
  }
  resizeCanvasToDisplay();
  await loadFloorMap(initialFloor);
  saveFloorPreference(initialFloor);
  await fetchPoseOnce();
  startPoseStream();

  floorSelect.addEventListener("change", async (e) => {
    const floor = e.target.value;
    await loadFloorMap(floor);
    saveFloorPreference(floor);
    appendLog(`切换楼层到 ${floor}`);
  });

  gridToggle.addEventListener("change", () => {
    saveMonitorCheckboxPrefs();
    renderScene();
  });

  if (scan2dToggle) {
    scan2dToggle.addEventListener("change", () => {
      if (!scan2dToggle.checked) {
        Object.keys(latestScanByRobot).forEach((k) => {
          delete latestScanByRobot[k];
        });
      }
      saveMonitorCheckboxPrefs();
      startSensorPolling();
      renderScene();
    });
  }
  if (plannedPathToggle) {
    plannedPathToggle.addEventListener("change", () => {
      if (!plannedPathToggle.checked) {
        Object.keys(latestPathByRobot).forEach((k) => {
          delete latestPathByRobot[k];
        });
      }
      saveMonitorCheckboxPrefs();
      startSensorPolling();
      renderScene();
    });
  }
  startSensorPolling();
}

function initGazeboPage() {
  const modelNameEl = document.getElementById("gazebo-model-name");
  const gazeboX = document.getElementById("gazebo-x");
  const gazeboY = document.getElementById("gazebo-y");
  const gazeboYaw = document.getElementById("gazebo-yaw");
  const btnGazeboResetCamera = document.getElementById("btn-gazebo-reset-camera");
  const btnGazeboCamHome = document.getElementById("btn-gazebo-cam-home");
  const btnGazeboCamUp = document.getElementById("btn-gazebo-cam-up");
  const btnGazeboCamDown = document.getElementById("btn-gazebo-cam-down");
  const btnGazeboCamLeft = document.getElementById("btn-gazebo-cam-left");
  const btnGazeboCamRight = document.getElementById("btn-gazebo-cam-right");
  const btnGazeboCamZoomIn = document.getElementById("btn-gazebo-cam-zoom-in");
  const btnGazeboCamZoomOut = document.getElementById("btn-gazebo-cam-zoom-out");
  const btnGazeboZone1 = document.getElementById("btn-gazebo-zone-1");
  const btnGazeboZone2 = document.getElementById("btn-gazebo-zone-2");
  const btnGazeboZone3 = document.getElementById("btn-gazebo-zone-3");
  const btnGazeboZone4 = document.getElementById("btn-gazebo-zone-4");
  const topCameraCanvas = document.getElementById("gazebo-top-camera");
  const topCameraCtx = topCameraCanvas ? topCameraCanvas.getContext("2d") : null;
  const gazeboMessage = document.getElementById("gazebo-message");
  const btnGazeboTeleport = document.getElementById("btn-gazebo-set-model-state");
  // World-frame position; orientation matches drawn_model.world topdown_camera
  // (look down, then rotate counterclockwise 90 deg in the ground plane).
  const cameraDefaultPose = {
    x: 0.0,
    y: 0.0,
    z: 32.0,
    hfov: 1.3962634,
    qx: -0.5,
    qy: 0.5,
    qz: 0.5,
    qw: 0.5,
  };
  const cameraModel = { ...cameraDefaultPose };
  const cameraFrame = {
    width: 0,
    height: 0,
  };
  const cameraModelName = "topdown_camera";
  const cameraDriveSpeedEl = document.getElementById("camera-drive-speed");
  const camDriveKey = { up: false, down: false, left: false, right: false };
  let lastCameraDriveSendMs = 0;
  let cameraDriveTimer = null;
  let topCameraRefreshTimer = null;
  let topCameraRefreshInFlight = false;
  let camZoomDir = 0;

  const CAMERA_Z_MIN = 6.0;
  const CAMERA_Z_MAX = 80.0;
  const CAMERA_FRAME_SIZE = 20.0;
  const CAMERA_REFRESH_MS = 40;
  const CAMERA_ZONE_CENTERS = {
    1: { x: -10.0, y: 10.0 },
    2: { x: 10.0, y: 10.0 },
    3: { x: -10.0, y: -10.0 },
    4: { x: 10.0, y: -10.0 },
  };

  function gazeboViewActive() {
    return !!(views.gazebo && views.gazebo.classList.contains("active"));
  }

  function cameraOrientationPayload() {
    return {
      x: cameraModel.qx,
      y: cameraModel.qy,
      z: cameraModel.qz,
      w: cameraModel.qw,
    };
  }

  function buildTopdownCameraSetStateBody() {
    return {
      model_name: cameraModelName,
      x: cameraModel.x,
      y: cameraModel.y,
      z: cameraModel.z,
      yaw: 0,
      reference_frame: "world",
      orientation: cameraOrientationPayload(),
    };
  }

  function fitCameraZForArea(areaWidth, areaHeight, padding = 1.1) {
    const w = Number(cameraFrame.width || 640);
    const h = Number(cameraFrame.height || 480);
    const hfov = Number(cameraModel.hfov || 0);
    if (!hfov || !w || !h) {
      return cameraDefaultPose.z;
    }
    const vfov = 2 * Math.atan(Math.tan(hfov / 2) * (h / w));
    const zx = areaWidth / (2 * Math.tan(hfov / 2));
    const zy = areaHeight / (2 * Math.tan(vfov / 2));
    return Math.min(CAMERA_Z_MAX, Math.max(CAMERA_Z_MIN, Math.max(zx, zy) * padding));
  }

  function setTopCameraPose(pose, message, useQuiet = false) {
    cameraModel.x = Number.isFinite(pose.x) ? pose.x : cameraModel.x;
    cameraModel.y = Number.isFinite(pose.y) ? pose.y : cameraModel.y;
    cameraModel.z = Math.min(
      CAMERA_Z_MAX,
      Math.max(CAMERA_Z_MIN, Number.isFinite(pose.z) ? pose.z : cameraModel.z)
    );
    cameraModel.qx = Number.isFinite(pose.qx) ? pose.qx : cameraModel.qx;
    cameraModel.qy = Number.isFinite(pose.qy) ? pose.qy : cameraModel.qy;
    cameraModel.qz = Number.isFinite(pose.qz) ? pose.qz : cameraModel.qz;
    cameraModel.qw = Number.isFinite(pose.qw) ? pose.qw : cameraModel.qw;
    if (useQuiet) {
      postTopdownCameraPoseQuiet();
      if (gazeboMessage && message) {
        gazeboMessage.textContent = message;
      }
      return Promise.resolve();
    }
    return postGazeboSetModelState(buildTopdownCameraSetStateBody()).then(() => {
      if (gazeboMessage && message) {
        gazeboMessage.textContent = message;
      }
    });
  }

  function zoomTopCameraToZone(zoneId, useQuiet = false) {
    const zone = CAMERA_ZONE_CENTERS[String(zoneId)];
    if (!zone) {
      return Promise.resolve();
    }
    return setTopCameraPose(
      {
        x: zone.x,
        y: zone.y,
        z: fitCameraZForArea(CAMERA_FRAME_SIZE, CAMERA_FRAME_SIZE),
      },
      `已聚焦区域 ${zoneId}`,
      useQuiet
    );
  }

  function cameraDriveSpeedMps() {
    const v = cameraDriveSpeedEl && Number(cameraDriveSpeedEl.value);
    return Number.isFinite(v) && v > 0 ? v : 4;
  }

  function postTopdownCameraPoseQuiet() {
    const now = performance.now();
    if (now - lastCameraDriveSendMs < 26) {
      return;
    }
    lastCameraDriveSendMs = now;
    fetch(`${API_BASE_URL}/api/gazebo/set_model_state`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(buildTopdownCameraSetStateBody()),
      keepalive: true,
    }).catch(() => {});
  }

  function setCamDriveKey(code, down) {
    if (code === "KeyW" || code === "ArrowUp") {
      camDriveKey.up = down;
    } else if (code === "KeyS" || code === "ArrowDown") {
      camDriveKey.down = down;
    } else if (code === "KeyA" || code === "ArrowLeft") {
      camDriveKey.left = down;
    } else if (code === "KeyD" || code === "ArrowRight") {
      camDriveKey.right = down;
    }
  }

  function clearCamDriveKeys() {
    camDriveKey.up = false;
    camDriveKey.down = false;
    camDriveKey.left = false;
    camDriveKey.right = false;
    camZoomDir = 0;
  }

  function tickTopdownCameraDrive() {
    if (!gazeboViewActive()) {
      return;
    }
    let ix = 0;
    let iy = 0;
    if (camDriveKey.up) {
      ix += 1;
    }
    if (camDriveKey.down) {
      ix -= 1;
    }
    if (camDriveKey.left) {
      iy += 1;
    }
    if (camDriveKey.right) {
      iy -= 1;
    }
    if (ix === 0 && iy === 0) {
      if (camZoomDir === 0) {
        return;
      }
    }
    const sp = cameraDriveSpeedMps();
    const dt = 1 / 30;
    cameraModel.x += ix * sp * dt;
    cameraModel.y += iy * sp * dt;
    if (camZoomDir !== 0) {
      const zoomSpeed = Math.max(6.0, cameraModel.z * 1.15);
      cameraModel.z = Math.min(
        CAMERA_Z_MAX,
        Math.max(CAMERA_Z_MIN, cameraModel.z + camZoomDir * zoomSpeed * dt)
      );
    }
    postTopdownCameraPoseQuiet();
  }

  window.addEventListener("keydown", (ev) => {
    if (!gazeboViewActive()) {
      return;
    }
    const t = ev.target;
    if (
      t &&
      (t.tagName === "INPUT" ||
        t.tagName === "TEXTAREA" ||
        t.tagName === "SELECT" ||
        t.isContentEditable)
    ) {
      return;
    }
    if (
      !["KeyW", "KeyA", "KeyS", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(
        ev.code
      )
    ) {
      return;
    }
    ev.preventDefault();
    setCamDriveKey(ev.code, true);
  });
  window.addEventListener("keyup", (ev) => {
    if (
      !["KeyW", "KeyA", "KeyS", "KeyD", "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(
        ev.code
      )
    ) {
      return;
    }
    setCamDriveKey(ev.code, false);
  });
  window.addEventListener("blur", () => {
    clearCamDriveKeys();
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      clearCamDriveKeys();
    }
  });

  async function fillTopdownCameraPoseFromGazebo() {
    if (!gazeboMessage) {
      return;
    }
    gazeboMessage.textContent = "读取 topdown_camera 位姿中…";
    try {
      const data = await fetchJson(`${API_BASE_URL}/api/gazebo/topdown/state`);
      const models = Array.isArray(data && data.models) ? data.models : [];
      const cam = models.find((m) => m && m.name === cameraModelName);
      if (!cam || !cam.pose) {
        gazeboMessage.textContent = "未找到 topdown_camera 模型，请确认 world 中模型名";
        return;
      }
      const px = Number(cam.pose.x);
      const py = Number(cam.pose.y);
      const pz = Number(cam.pose.z);
      const pq = cam.pose;
      if (
        pq.qx != null &&
        pq.qy != null &&
        pq.qz != null &&
        pq.qw != null &&
        [pq.qx, pq.qy, pq.qz, pq.qw].every((v) => Number.isFinite(Number(v)))
      ) {
        cameraModel.qx = Number(pq.qx);
        cameraModel.qy = Number(pq.qy);
        cameraModel.qz = Number(pq.qz);
        cameraModel.qw = Number(pq.qw);
      }
      if (!Number.isNaN(px)) cameraModel.x = px;
      if (!Number.isNaN(py)) cameraModel.y = py;
      if (!Number.isNaN(pz)) cameraModel.z = pz;
      gazeboMessage.textContent = `已读取 topdown_camera: (${cameraModel.x.toFixed(2)}, ${cameraModel.y.toFixed(2)}, ${cameraModel.z.toFixed(2)})`;
    } catch (err) {
      gazeboMessage.textContent = err.message || String(err);
    }
  }

  function fitTopCameraCanvas(w, h) {
    if (!topCameraCanvas || !topCameraCtx) {
      return;
    }
    const rect = topCameraCanvas.getBoundingClientRect();
    const vw = Math.max(280, Math.floor(rect.width));
    const vh = Math.max(180, Math.floor(rect.height));
    const dpr = window.devicePixelRatio || 1;
    topCameraCanvas.width = Math.floor(vw * dpr);
    topCameraCanvas.height = Math.floor(vh * dpr);
    topCameraCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
    topCameraCtx.fillStyle = "#ffffff";
    topCameraCtx.fillRect(0, 0, vw, vh);
    if (w && h) {
      topCameraCtx.fillStyle = "#475569";
      topCameraCtx.font = "12px sans-serif";
      topCameraCtx.fillText(`${w}x${h}`, 8, 16);
    }
  }

  function rotateVectorByQuaternion(vec, quat) {
    const x = Number(quat.x || 0);
    const y = Number(quat.y || 0);
    const z = Number(quat.z || 0);
    const w = Number(quat.w || 1);
    const xx = x * x;
    const yy = y * y;
    const zz = z * z;
    const xy = x * y;
    const xz = x * z;
    const yz = y * z;
    const wx = w * x;
    const wy = w * y;
    const wz = w * z;
    return {
      x:
        (1 - 2 * (yy + zz)) * vec.x +
        2 * (xy - wz) * vec.y +
        2 * (xz + wy) * vec.z,
      y:
        2 * (xy + wz) * vec.x +
        (1 - 2 * (xx + zz)) * vec.y +
        2 * (yz - wx) * vec.z,
      z:
        2 * (xz - wy) * vec.x +
        2 * (yz + wx) * vec.y +
        (1 - 2 * (xx + yy)) * vec.z,
    };
  }

  function cameraPixelToWorld(px, py) {
    const width = cameraFrame.width;
    const height = cameraFrame.height;
    if (!width || !height) {
      return null;
    }
    const hfov = Number(cameraModel.hfov || 0);
    const camZ = Number(cameraModel.z || 0);
    if (!hfov || !camZ) {
      return null;
    }
    const fx = width / (2 * Math.tan(hfov / 2));
    const vfov = 2 * Math.atan(Math.tan(hfov / 2) * (height / width));
    const fy = height / (2 * Math.tan(vfov / 2));
    const cx = width / 2;
    const cy = height / 2;
    // Gazebo camera optical axis is +X in camera frame.
    // For the rendered image, screen-right maps to -Y and screen-down maps to -Z.
    const rayCamera = {
      x: 1,
      y: -(px - cx) / fx,
      z: -(py - cy) / fy,
    };
    const rayWorld = rotateVectorByQuaternion(rayCamera, cameraOrientationPayload());
    if (!Number.isFinite(rayWorld.z) || Math.abs(rayWorld.z) < 1e-6) {
      return null;
    }
    const t = -camZ / rayWorld.z;
    if (!Number.isFinite(t) || t <= 0) {
      return null;
    }
    return {
      x: cameraModel.x + rayWorld.x * t,
      y: cameraModel.y + rayWorld.y * t,
    };
  }

  function fillGazeboTargetFromWorld(world) {
    if (!gazeboX || !gazeboY || !gazeboMessage || !world) {
      return;
    }
    gazeboX.value = String(Number(world.x.toFixed(3)));
    gazeboY.value = String(Number(world.y.toFixed(3)));
    gazeboMessage.textContent =
      `已从画面取点: (${world.x.toFixed(2)}, ${world.y.toFixed(2)})，可调整 yaw 后移动模型`;
  }

  function normalizeDegrees360(value) {
    if (!Number.isFinite(value)) {
      return 0;
    }
    const wrapped = ((value % 360) + 360) % 360;
    return wrapped === 0 && value > 0 ? 360 : wrapped;
  }

  function correctTopCameraPickWorld(world) {
    if (!world) {
      return null;
    }
    return world;
  }

  function handleTopCameraPick(ev) {
    if (!topCameraCanvas || !cameraFrame.width || !cameraFrame.height) {
      return;
    }
    const rect = topCameraCanvas.getBoundingClientRect();
    const sx = ev.clientX - rect.left;
    const sy = ev.clientY - rect.top;
    if (sx < 0 || sy < 0 || sx > rect.width || sy > rect.height) {
      return;
    }
    const px = (sx / rect.width) * cameraFrame.width;
    const py = (sy / rect.height) * cameraFrame.height;
    const world = correctTopCameraPickWorld(cameraPixelToWorld(px, py));
    if (!world) {
      if (gazeboMessage) {
        gazeboMessage.textContent = "当前相机参数不足，无法从画面换算 world 坐标";
      }
      return;
    }
    fillGazeboTargetFromWorld(world);
  }

  async function refreshTopCameraFrame() {
    if (!topCameraCanvas || !topCameraCtx) {
      return;
    }
    if (topCameraRefreshInFlight) {
      return;
    }
    topCameraRefreshInFlight = true;
    try {
      const data = await fetchJson(`${API_BASE_URL}/api/gazebo/top_camera`);
      if (data && data.available && data.data_b64) {
        const width = Number(data.width || 0);
        const height = Number(data.height || 0);
        if (!width || !height) {
          return;
        }
        cameraFrame.width = width;
        cameraFrame.height = height;
        const raw = atob(data.data_b64);
        const rgb = new Uint8ClampedArray(raw.length);
        for (let i = 0; i < raw.length; i += 1) {
          rgb[i] = raw.charCodeAt(i);
        }
        const rgba = new Uint8ClampedArray(width * height * 4);
        for (let i = 0, j = 0; i < rgb.length; i += 3, j += 4) {
          rgba[j] = rgb[i];
          rgba[j + 1] = rgb[i + 1];
          rgba[j + 2] = rgb[i + 2];
          rgba[j + 3] = 255;
        }
        fitTopCameraCanvas(width, height);
        const vw = topCameraCanvas.clientWidth || width;
        const vh = topCameraCanvas.clientHeight || height;
        const off = document.createElement("canvas");
        off.width = width;
        off.height = height;
        const offCtx = off.getContext("2d");
        offCtx.putImageData(new ImageData(rgba, width, height), 0, 0);
        topCameraCtx.clearRect(0, 0, vw, vh);
        topCameraCtx.drawImage(off, 0, 0, vw, vh);
      }
    } catch {
      /* ignore */
    } finally {
      topCameraRefreshInFlight = false;
    }
  }

  if (btnGazeboTeleport) {
    btnGazeboTeleport.addEventListener("click", async () => {
      if (!gazeboMessage) {
        return;
      }
      const modelName = modelNameEl?.value?.trim();
      if (!modelName) {
        gazeboMessage.textContent = "请填写 Gazebo 模型名（与 spawn -entity 一致，如 robot2）";
        return;
      }
      const x = parseFloat((gazeboX && gazeboX.value) || "");
      const y = parseFloat((gazeboY && gazeboY.value) || "");
      const yawDegRaw = parseFloat((gazeboYaw && gazeboYaw.value) || "0");
      const yawDeg = normalizeDegrees360(Number.isNaN(yawDegRaw) ? 0 : yawDegRaw);
      const yaw = (yawDeg * Math.PI) / 180;
      if (gazeboYaw) {
        gazeboYaw.value = String(yawDeg);
      }
      if (Number.isNaN(x) || Number.isNaN(y)) {
        gazeboMessage.textContent = "请填写 x、y 坐标";
        return;
      }
      gazeboMessage.textContent = "Gazebo 瞬移中…";
      try {
        await postGazeboSetModelState({
          model_name: modelName,
          x,
          y,
          yaw,
          z: 0.05,
          reference_frame: "world",
        });
        gazeboMessage.textContent = `Gazebo 已瞬移: ${modelName}`;
        appendLog(`Gazebo set_model_state: ${modelName} -> (${x.toFixed(2)}, ${y.toFixed(2)})`);
      } catch (err) {
        gazeboMessage.textContent = `Gazebo 瞬移失败: ${err.message || err}`;
      }
    });
  }

  function bindHoldButton(btn, onStart, onStop) {
    if (!btn) {
      return;
    }
    const stop = () => {
      if (onStop) {
        onStop();
      }
    };
    btn.addEventListener("pointerdown", (ev) => {
      ev.preventDefault();
      btn.setPointerCapture?.(ev.pointerId);
      onStart();
    });
    btn.addEventListener("pointerup", stop);
    btn.addEventListener("pointercancel", stop);
    btn.addEventListener("pointerleave", stop);
  }

  bindHoldButton(btnGazeboCamUp, () => {
    camDriveKey.up = true;
  }, () => {
    camDriveKey.up = false;
  });
  bindHoldButton(btnGazeboCamDown, () => {
    camDriveKey.down = true;
  }, () => {
    camDriveKey.down = false;
  });
  bindHoldButton(btnGazeboCamLeft, () => {
    camDriveKey.left = true;
  }, () => {
    camDriveKey.left = false;
  });
  bindHoldButton(btnGazeboCamRight, () => {
    camDriveKey.right = true;
  }, () => {
    camDriveKey.right = false;
  });
  bindHoldButton(btnGazeboCamZoomIn, () => {
    camZoomDir = -1;
  }, () => {
    camZoomDir = 0;
  });
  bindHoldButton(btnGazeboCamZoomOut, () => {
    camZoomDir = 1;
  }, () => {
    camZoomDir = 0;
  });

  if (btnGazeboCamHome) {
    btnGazeboCamHome.addEventListener("click", async () => {
      if (!gazeboMessage) {
        return;
      }
      try {
        await setTopCameraPose(cameraDefaultPose, "已切换到全图视角");
      } catch (err) {
        gazeboMessage.textContent = `切换全图失败: ${err.message || err}`;
      }
    });
  }

  [btnGazeboZone1, btnGazeboZone2, btnGazeboZone3, btnGazeboZone4].forEach((btn, idx) => {
    if (!btn) {
      return;
    }
    const zoneId = idx + 1;
    btn.addEventListener("click", async () => {
      if (!gazeboMessage) {
        return;
      }
      try {
        await zoomTopCameraToZone(zoneId);
      } catch (err) {
        gazeboMessage.textContent = `切换区域 ${zoneId} 失败: ${err.message || err}`;
      }
    });
  });

  if (btnGazeboResetCamera) {
    btnGazeboResetCamera.addEventListener("click", async () => {
      if (!gazeboMessage) {
        return;
      }
      gazeboMessage.textContent = "复原俯视相机中…";
      try {
        await setTopCameraPose(cameraDefaultPose, "俯视相机已复原");
        gazeboMessage.textContent = "俯视相机已复原";
      } catch (err) {
        gazeboMessage.textContent = `复原俯视相机失败: ${err.message || err}`;
      }
    });
  }

  if (topCameraCanvas) {
    fillTopdownCameraPoseFromGazebo().catch(() => {});
    fitTopCameraCanvas();
    topCameraCanvas.addEventListener("click", handleTopCameraPick);
    cameraDriveTimer = window.setInterval(() => {
      tickTopdownCameraDrive();
    }, 33);
    window.addEventListener("resize", () => {
      fitTopCameraCanvas();
    });
    refreshTopCameraFrame().catch(() => {});
    topCameraRefreshTimer = window.setInterval(() => {
      refreshTopCameraFrame().catch(() => {});
    }, CAMERA_REFRESH_MS);
  }
}

async function bootstrap() {
  initSettings();
  initLogs();
  initRosNodesPage();
  initRobotPresenceUi();
  fetchRobotStatusCache()
    .then(() => updateRobotPresenceTriggerSummary())
    .catch(() => {});
  await initMonitor();
  initGazeboPage();
}

bootstrap();
