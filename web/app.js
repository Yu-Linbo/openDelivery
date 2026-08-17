const initialFrontendPort = Number(window.location.port || 0);
let API_BASE_URL =
  window.API_BASE_URL ||
  window.location.protocol +
    "//" +
    window.location.hostname +
    ":" +
    (initialFrontendPort > 0 ? initialFrontendPort + 1 : 8001);

const menuButtons = Array.from(document.querySelectorAll(".menu-item"));
const layoutEl = document.querySelector(".layout");
const contentEl = document.querySelector(".content");
const btnSidebarCollapse = document.getElementById("btn-sidebar-collapse");
const btnSidebarReveal = document.getElementById("btn-sidebar-reveal");
const robotPresenceAnchor = document.getElementById("robot-presence-anchor");
const btnRobotPresence = document.getElementById("btn-robot-presence");
const robotPresencePanel = document.getElementById("robot-presence-panel");
const robotPresenceList = document.getElementById("robot-presence-list");
const robotPresenceEmpty = document.getElementById("robot-presence-empty");
const robotPresenceSummary = document.getElementById("robot-presence-summary");
const robotPresenceNewId = document.getElementById("robot-presence-new-id");
const btnRobotPresenceBringupNew = document.getElementById("btn-robot-presence-bringup-new");
const robotPresenceNewMsg = document.getElementById("robot-presence-new-msg");
const robotQuickDock = document.getElementById("robot-quick-dock");
const robotDetailPanel = document.getElementById("robot-detail-panel");
const robotDetailName = document.getElementById("robot-detail-name");
const robotDetailSubtitle = document.getElementById("robot-detail-subtitle");
const robotDetailTabs = document.getElementById("robot-detail-tabs");
const robotDetailBody = document.getElementById("robot-detail-body");
const btnRobotDetailClose = document.getElementById("btn-robot-detail-close");
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

/** Coalesce high-frequency pose/sensor updates to one paint per animation frame (reduces map flicker). */
let mapPaintRaf = 0;
function scheduleMapPaint() {
  if (mapPaintRaf) {
    return;
  }
  mapPaintRaf = requestAnimationFrame(() => {
    mapPaintRaf = 0;
    renderScene();
    refreshMetaPanel();
  });
}

const settingsForm = document.getElementById("settings-form");
const settingsMessage = document.getElementById("settings-message");
const logBagList = document.getElementById("log-bag-list");
const logBagFileList = document.getElementById("log-bag-file-list");
const logBagFileHint = document.getElementById("log-bag-file-hint");
const logBagStatus = document.getElementById("log-bag-status");
const logBagSelectionSummary = document.getElementById("log-bag-selection-summary");
const logBagRobotSelect = document.getElementById("log-bag-robot-select");
const btnRefreshLogBags = document.getElementById("btn-refresh-log-bags");
const btnDownloadLogBag = document.getElementById("btn-download-log-bag");
const btnResetView = document.getElementById("btn-reset-view");
const scan2dToggle = document.getElementById("scan-2d-toggle");
const plannedPathToggle = document.getElementById("planned-path-toggle");
const semanticMapToggle = document.getElementById("semantic-map-toggle");
const customPointsToggle = document.getElementById("custom-points-toggle");
const waypointName = document.getElementById("waypoint-name");
const waypointType = document.getElementById("waypoint-type");
const btnWaypointPick = document.getElementById("btn-waypoint-pick");
const btnWaypointSave = document.getElementById("btn-waypoint-save");
const mapWaypointList = document.getElementById("map-waypoint-list");
const waypointMessage = document.getElementById("waypoint-message");
const waypointX = document.getElementById("waypoint-x");
const waypointY = document.getElementById("waypoint-y");
const waypointYaw = document.getElementById("waypoint-yaw");
const mapEditorLayer = document.getElementById("map-editor-layer");
const mapEditorTool = document.getElementById("map-editor-tool");
const mapEditorBrush = document.getElementById("map-editor-brush");
const mapEditorRasterValue = document.getElementById("map-editor-raster-value");
const mapEditorSemanticLabel = document.getElementById("map-editor-semantic-label");
const mapEditorSemanticSwatch = document.getElementById("map-editor-semantic-swatch");
const mapEditorSemanticName = document.getElementById("map-editor-semantic-name");
const btnMapEditorOpen = document.getElementById("btn-map-editor-open");
const mapEditorDialog = document.getElementById("map-editor-dialog");
const mapEditorBackdrop = document.getElementById("map-editor-backdrop");
const btnMapEditorClose = document.getElementById("btn-map-editor-close");
const mapEditorFloor = document.getElementById("map-editor-floor");
const btnMapEditorUndo = document.getElementById("btn-map-editor-undo");
const btnMapEditorCancel = document.getElementById("btn-map-editor-cancel");
const btnMapEditorToggle = document.getElementById("btn-map-editor-toggle");
const btnMapEditorSave = document.getElementById("btn-map-editor-save");
const mapEditorMessage = document.getElementById("map-editor-message");
const teleopLinear = document.getElementById("teleop-linear");
const teleopAngular = document.getElementById("teleop-angular");
const teleopMessage = document.getElementById("teleop-message");
const relocRobotId = document.getElementById("reloc-robot-id");
const relocX = document.getElementById("reloc-x");
const relocY = document.getElementById("reloc-y");
const relocYaw = document.getElementById("reloc-yaw");
const relocMessage = document.getElementById("reloc-message");
const btnRelocMapOnly = document.getElementById("btn-reloc-map-only");
const btnRelocPoseOnly = document.getElementById("btn-reloc-pose-only");
const btnRelocBoth = document.getElementById("btn-reloc-both");
const btnRelocRecord = document.getElementById("btn-reloc-record");
const btnRelocFillPose = document.getElementById("btn-reloc-fill-pose");
const relocPickToggle = document.getElementById("reloc-pick-toggle");
const btnRelocPickGoal = document.getElementById("btn-reloc-pick-goal");
const btnRelocSkipHeading = document.getElementById("btn-reloc-skip-heading");
const btnRelocClearPick = document.getElementById("btn-reloc-clear-pick");
const mapNameInput = document.getElementById("map-name-input");
const btnSaveMap = document.getElementById("btn-save-map");
const rosNodesSummary = document.getElementById("ros-nodes-summary");
const rosNodesError = document.getElementById("ros-nodes-error");
const rosRobotGroups = document.getElementById("ros-robot-groups");
const btnRosNodesRefresh = document.getElementById("btn-ros-nodes-refresh");

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
let semanticBitmap = null;
let semanticEditCanvas = null;
let mapPoints = [];
let savedMapPoints = [];
let semanticLegend = [];
let mapEditorActive = false;
let mapEditorDirty = false;
const mapEditorDirtyLayers = new Set();
const mapEditorUndoStack = [];
const MAP_EDITOR_UNDO_LIMIT = 12;
const MAP_EDITOR_UNDO_MAX_BYTES = 64 * 1024 * 1024;
let mapEditorUndoBytes = 0;
let mapEditorPainting = false;
let mapEditorResumeAfterPick = false;
let mapEditorMovedPointId = "";
let relocRobotOptionsSignature = "";

/** Latest snapshot from backend: { timestamp, source, robots: [...] } */
let latestSnapshot = null;
let webBootstrapData = null;

/** Rows from `GET /api/robot/status/cache` (merged with pose for 离线 robots). */
let robotStatusCacheItems = [];
let robotPresencePanelOpen = false;
let robotPresenceCacheTimer = null;
let robotPresenceListMarkup = "";
let robotQuickDockMarkup = "";
let selectedPresenceRobotId = "";
let selectedDetailRobotId = "";
let robotDetailActiveTab = "overview";
let robotDetailPayload = null;
let robotDetailPollTimer = null;
let robotDetailRefreshInFlight = false;

/** Per-robot: idle | starting | sim_online（仿真上线流程） */
const simBringupPhaseByRobot = {};
/** 刷新后短暂保留 sim_online；每次渲染会 prune：无 robot_status 心跳则清为 idle，避免离线仍显示「仿真离线」 */
const SIM_BRINGUP_PHASE_STORAGE_KEY = "openDelivery_sim_bringup_phase_v1";

function loadSimBringupPhasesFromSession() {
  try {
    const raw = sessionStorage.getItem(SIM_BRINGUP_PHASE_STORAGE_KEY);
    const o = raw ? JSON.parse(raw) : {};
    if (!o || typeof o !== "object") {
      return;
    }
    Object.keys(o).forEach((id) => {
      if (o[id] === "sim_online") {
        simBringupPhaseByRobot[id] = "sim_online";
      }
    });
  } catch {
    /* ignore */
  }
}

function persistSimBringupPhasesToSession() {
  try {
    const o = {};
    Object.keys(simBringupPhaseByRobot).forEach((id) => {
      if (simBringupPhaseByRobot[id] === "sim_online") {
        o[id] = "sim_online";
      }
    });
    sessionStorage.setItem(SIM_BRINGUP_PHASE_STORAGE_KEY, JSON.stringify(o));
  } catch {
    /* ignore */
  }
}

/** 已无 robot_status 心跳时清掉 sim_online（含 session 残留），避免离线仍显示「仿真离线」 */
function pruneStaleSimOnlinePhaseIfNeeded(rid) {
  if (!rid || simShutdownBusyByRobot[rid]) {
    return false;
  }
  if (simBringupPhaseByRobot[rid] !== "sim_online") {
    return false;
  }
  const rows = mergePresenceRows();
  const row = rows.find((x) => x.id === rid);
  if (!row || !row.online) {
    simBringupPhaseByRobot[rid] = "idle";
    return true;
  }
  return false;
}

/** 全局仅允许一台机器人处于「上线中」 */
let simBringupGlobalStartingId = "";
const simShutdownBusyByRobot = {};
/** 防止重复点击导致并发 startup 请求 */
const simBringupInFlightByRobot = {};
/** 仿真离线成功后，按钮短暂显示「离线」直到该时间戳（ms） */
const simPostOfflineButtonUntilByRobot = {};

/** Per-robot overlays from REST (map frame) */
const latestScanByRobot = {};
const latestPathByRobot = {};
let sensorPollTimer = null;
let sensorPollInFlight = false;
let scanStream = null;
let scanStreamActive = false;
let scanStreamRetryAt = 0;

/** Live OccupancyGrid polling for floor like robot1_mapping → GET /api/mapping/live?robot_id= */
let mapLiveTimer = null;
let mapLiveInitializedView = false;
let mapLiveInFlight = false;
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
/** 当前地图选点用途：重定位或导航目标。 */
let relocPickAction = "relocalize";
/** @type {{ x: number, y: number } | null} */
let relocPickAnchorWorld = null;
let relocPickHoverSx = null;
let relocPickHoverSy = null;

function activateView(target, persist = true) {
  const next = views[target] ? target : "monitor";
  if (contentEl) {
    contentEl.classList.toggle("content--monitor", next === "monitor");
  }
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

async function fetchJson(path, options = undefined) {
  const res = await fetch(path, options);
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

  const hosts = [
    window.location.hostname,
    "127.0.0.1",
    "localhost",
  ].filter(Boolean);
  const originPort = Number(window.location.port || 0);
  const candidatePorts = [8001, 8002, 8003, originPort + 1, originPort + 2].filter(
    (p) => Number.isInteger(p) && p > 0
  );
  const seen = new Set();
  for (const host of hosts) {
    for (const p of candidatePorts) {
      const base = `${window.location.protocol}//${host}:${p}`;
      if (seen.has(base)) {
        continue;
      }
      seen.add(base);
      if (await canReachApi(base)) {
        API_BASE_URL = base;
        return;
      }
    }
  }
}

async function fetchWebBootstrap() {
  webBootstrapData = await fetchJson(`${API_BASE_URL}/api/web/bootstrap`, {
    cache: "no-store",
  });
  if (Array.isArray(webBootstrapData.floors)) {
    floors = webBootstrapData.floors;
  }
  const cache = webBootstrapData.robot_status_cache;
  if (cache && Array.isArray(cache.items)) {
    robotStatusCacheItems = cache.items;
  }
  if (webBootstrapData.pose && typeof webBootstrapData.pose === "object") {
    latestSnapshot = webBootstrapData.pose;
  }
  return webBootstrapData;
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

async function postRobotGoto(robotId, x, y, yaw, floorId = "") {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), 30000);
  let res;
  let data = {};
  try {
    res = await fetch(`${API_BASE_URL}/api/robot/motion/goto`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ robot_id: robotId, x, y, yaw, floor_id: floorId }),
      signal: ctrl.signal,
    });
    try {
      data = await res.json();
    } catch {
      /* ignore */
    }
  } catch (err) {
    if (err && err.name === "AbortError") {
      throw new Error("导航请求超时，请检查导航 action server");
    }
    throw new Error("无法连接 Web 后端，请检查 Web 栈");
  } finally {
    clearTimeout(timer);
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
  const payload = await res.json();
  return payload && payload.available === false ? null : payload;
}

async function fetchFloors(force = false) {
  if (!force && Array.isArray(floors) && floors.length > 0) {
    return;
  }
  const ts = Date.now();
  const data = await fetchJson(`${API_BASE_URL}/api/floors?_=${ts}`, {
    cache: "no-store",
  });
  floors = Array.isArray(data.floors) ? data.floors : [];
}

async function refreshFloorOptionsPreserveSelection() {
  const prev = floorSelect ? String(floorSelect.value || "") : "";
  await fetchFloors(true);
  addFloorOptions();
  if (floorSelect) {
    if (prev && floors.includes(prev)) {
      floorSelect.value = prev;
    } else if (activeFloor && floors.includes(activeFloor)) {
      floorSelect.value = activeFloor;
    }
  }
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

function getMapOriginPose() {
  if (!activeMeta) {
    return { x: 0, y: 0, yaw: 0 };
  }
  const [ox, oy, legacyThird] = parseOrigin(activeMeta.origin);
  // YAML origin[2] is yaw; live mapping stores yaw in origin_yaw.
  const explicitYaw = Number(activeMeta.origin_yaw);
  const yaw =
    Number.isFinite(explicitYaw) && !Number.isNaN(explicitYaw) ? explicitYaw : Number(legacyThird || 0);
  return { x: ox, y: oy, yaw: Number.isFinite(yaw) ? yaw : 0 };
}

function getCanvasCssSize() {
  const w = canvas.clientWidth || mapWrapper.clientWidth || 900;
  const h = canvas.clientHeight || 560;
  return { w, h };
}

let mapResizeDebounceTimer = null;
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
  const nextW = Math.floor(w * dpr);
  const nextH = Math.floor(h * dpr);
  // Changing canvas width/height clears the buffer; skip if layout size unchanged to avoid visible flash.
  if (
    canvas.width === nextW &&
    canvas.height === nextH &&
    canvas.style.width === `${w}px` &&
    canvas.style.height === `${h}px`
  ) {
    return;
  }
  canvas.width = nextW;
  canvas.height = nextH;
  canvas.style.width = `${w}px`;
  canvas.style.height = `${h}px`;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  renderScene();
}

function scheduleResizeCanvasToDisplay() {
  if (mapResizeDebounceTimer) {
    clearTimeout(mapResizeDebounceTimer);
  }
  mapResizeDebounceTimer = setTimeout(() => {
    mapResizeDebounceTimer = null;
    resizeCanvasToDisplay();
  }, 80);
}

/**
 * Build grayscale bitmap from PGM once per floor (avoids per-frame distortion / heavy work).
 */
function buildMapBitmap(pgm, options = {}) {
  const { flipY = false } = options;
  const { width, height, maxVal, data } = pgm;
  const c = document.createElement("canvas");
  c.width = width;
  c.height = height;
  const cctx = c.getContext("2d");
  const imageData = cctx.createImageData(width, height);
  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const srcY = flipY ? height - 1 - y : y;
      const src = srcY * width + x;
      const v = Math.round((data[src] / maxVal) * 255);
      const p = (y * width + x) * 4;
      imageData.data[p] = v;
      imageData.data[p + 1] = v;
      imageData.data[p + 2] = v;
      imageData.data[p + 3] = 255;
    }
  }
  cctx.putImageData(imageData, 0, 0);
  return c;
}

function buildSemanticBitmapFromDataUrl(dataUrl) {
  return new Promise((resolve, reject) => {
    if (!dataUrl) { resolve(null); return; }
    const image = new Image();
    image.onload = () => {
      const c = document.createElement("canvas");
      c.width = image.naturalWidth;
      c.height = image.naturalHeight;
      c.getContext("2d").drawImage(image, 0, 0);
      resolve(c);
    };
    image.onerror = () => reject(new Error("语义地图图片加载失败"));
    image.src = dataUrl;
  });
}

function createBlankSemanticCanvas() {
  if (!activePgm) return null;
  const c = document.createElement("canvas");
  c.width = activePgm.width;
  c.height = activePgm.height;
  const cctx = c.getContext("2d");
  cctx.fillStyle = "#ffffff";
  cctx.fillRect(0, 0, c.width, c.height);
  return c;
}

function cloneMapPointRows(rows) {
  return (Array.isArray(rows) ? rows : []).map((point) => ({ ...point }));
}

function cloneCanvasElement(source) {
  if (!source) return null;
  const clone = document.createElement("canvas");
  clone.width = source.width;
  clone.height = source.height;
  clone.getContext("2d").drawImage(source, 0, 0);
  return clone;
}

function captureCanvasSnapshot(source) {
  if (!source) return null;
  const image = source.getContext("2d").getImageData(0, 0, source.width, source.height);
  return { width: source.width, height: source.height, data: new Uint8ClampedArray(image.data) };
}

function restoreCanvasSnapshot(target, snapshot) {
  if (!snapshot) return target;
  const canvasTarget = target && target.width === snapshot.width && target.height === snapshot.height
    ? target
    : document.createElement("canvas");
  canvasTarget.width = snapshot.width;
  canvasTarget.height = snapshot.height;
  const targetCtx = canvasTarget.getContext("2d");
  const image = targetCtx.createImageData(snapshot.width, snapshot.height);
  image.data.set(snapshot.data);
  targetCtx.putImageData(image, 0, 0);
  return canvasTarget;
}

function clearMapEditorUndoHistory() {
  mapEditorUndoStack.length = 0;
  mapEditorUndoBytes = 0;
  syncMapEditorUi();
}

function pushMapEditorUndoSnapshot(layer) {
  if (!layer) return;
  const snapshot = { layer, dirtyLayers: Array.from(mapEditorDirtyLayers), byteSize: 0 };
  if (layer === "points") {
    snapshot.points = cloneMapPointRows(mapPoints);
    snapshot.byteSize = Math.max(1024, JSON.stringify(snapshot.points).length * 2);
  } else {
    const source = layer === "semantic" ? semanticEditCanvas : mapBitmap;
    snapshot.canvas = captureCanvasSnapshot(source);
    if (!snapshot.canvas) return;
    snapshot.byteSize = snapshot.canvas.data.byteLength;
  }
  mapEditorUndoStack.push(snapshot);
  mapEditorUndoBytes += snapshot.byteSize;
  while (mapEditorUndoStack.length > 1 && (mapEditorUndoStack.length > MAP_EDITOR_UNDO_LIMIT || mapEditorUndoBytes > MAP_EDITOR_UNDO_MAX_BYTES)) {
    const removed = mapEditorUndoStack.shift();
    mapEditorUndoBytes -= removed.byteSize || 0;
  }
  syncMapEditorUi();
}

function undoMapEditorChange() {
  const snapshot = mapEditorUndoStack.pop();
  if (!snapshot) {
    if (mapEditorMessage) mapEditorMessage.textContent = "没有可撤销的修改";
    return;
  }
  mapEditorUndoBytes = Math.max(0, mapEditorUndoBytes - (snapshot.byteSize || 0));
  if (snapshot.layer === "points") {
    mapPoints = cloneMapPointRows(snapshot.points);
    renderMapWaypointList();
  } else if (snapshot.layer === "semantic") {
    semanticEditCanvas = restoreCanvasSnapshot(semanticEditCanvas, snapshot.canvas);
  } else {
    mapBitmap = restoreCanvasSnapshot(mapBitmap, snapshot.canvas);
  }
  mapEditorDirtyLayers.clear();
  snapshot.dirtyLayers.forEach((layer) => mapEditorDirtyLayers.add(layer));
  mapEditorDirty = mapEditorDirtyLayers.size > 0;
  syncMapEditorUi();
  renderScene();
  if (mapEditorMessage) mapEditorMessage.textContent = "已撤销上一步修改";
}

function discardMapEditorChanges() {
  if (!mapEditorDirty || !activePgm) return;
  if (!window.confirm("确定放弃当前地图的全部未保存修改吗？")) return;
  mapBitmap = buildMapBitmap(activePgm);
  semanticEditCanvas = semanticBitmap ? cloneCanvasElement(semanticBitmap) : createBlankSemanticCanvas();
  mapPoints = cloneMapPointRows(savedMapPoints);
  mapEditorDirtyLayers.clear();
  mapEditorDirty = false;
  clearMapEditorUndoHistory();
  renderMapWaypointList();
  renderScene();
  if (mapEditorMessage) mapEditorMessage.textContent = "已取消全部未保存修改";
}

async function loadActiveMapAssets() {
  semanticBitmap = null;
  semanticEditCanvas = null;
  mapPoints = [];
  savedMapPoints = [];
  clearMapEditorUndoHistory();
  semanticLegend = [];
  if (!activeFloor || isMappingFloor(activeFloor)) {
    renderMapWaypointList();
    return;
  }
  const assets = await fetchJson(`${API_BASE_URL}/api/maps/${encodeURIComponent(activeFloor)}/assets`, { cache: "no-store" });
  mapPoints = cloneMapPointRows(Array.isArray(assets.points) ? assets.points : []);
  savedMapPoints = cloneMapPointRows(mapPoints);
  const labels = assets.semantic_legend && Array.isArray(assets.semantic_legend.labels)
    ? assets.semantic_legend.labels : [];
  semanticLegend = labels;
  semanticBitmap = await buildSemanticBitmapFromDataUrl(assets.semantic_png || "");
  semanticEditCanvas = semanticBitmap ? cloneCanvasElement(semanticBitmap) : createBlankSemanticCanvas();
  clearMapEditorUndoHistory();
  renderSemanticLabelOptions();
  renderMapWaypointList();
}

function normalizeSemanticColor(value) {
  const color = String(value || "").trim();
  return color.length === 7 && /^#[0-9a-fA-F]{6}/.test(color) ? color.toLowerCase() : "#ffffff";
}

function semanticTextColor(color) {
  const normalized = normalizeSemanticColor(color).slice(1);
  const r = parseInt(normalized.slice(0, 2), 16);
  const g = parseInt(normalized.slice(2, 4), 16);
  const b = parseInt(normalized.slice(4, 6), 16);
  return (r * 299 + g * 587 + b * 114) / 1000 > 150 ? "#0f172a" : "#f8fafc";
}

function updateSemanticLabelPreview() {
  if (!mapEditorSemanticLabel) return;
  const option = mapEditorSemanticLabel.selectedOptions && mapEditorSemanticLabel.selectedOptions[0];
  const color = normalizeSemanticColor(mapEditorSemanticLabel.value);
  const name = option && option.dataset.semanticName ? option.dataset.semanticName : "未命名语义";
  if (mapEditorSemanticSwatch) mapEditorSemanticSwatch.style.backgroundColor = color;
  if (mapEditorSemanticName) mapEditorSemanticName.textContent = name + " · " + color;
  mapEditorSemanticLabel.style.borderColor = color;
}

function renderSemanticLabelOptions() {
  if (!mapEditorSemanticLabel) return;
  const fallback = [
    { id: "background", name: "背景/未标注", color: "#ffffff" },
    { id: "obstacle", name: "障碍/墙体", color: "#000000" },
    { id: "elevator", name: "电梯区域", color: "#6c5ce7" },
    { id: "corridor", name: "走廊", color: "#f39c12" },
  ];
  const labels = semanticLegend.length ? semanticLegend : fallback;
  mapEditorSemanticLabel.innerHTML = "";
  labels.forEach((label, index) => {
    const option = document.createElement("option");
    const color = normalizeSemanticColor(label.color);
    const name = String(label.name || label.id || ("语义标签 " + (index + 1)));
    option.value = color;
    option.textContent = name + " · " + color;
    option.dataset.semanticName = name;
    option.style.backgroundColor = color;
    option.style.color = semanticTextColor(color);
    mapEditorSemanticLabel.appendChild(option);
  });
  updateSemanticLabelPreview();
}

function pointTypeLabel(type) {
  return type === "elevator" || type === "elevator_inside" ? "电梯内点" : type === "elevator_waiting" ? "电梯等待点" : type === "standby" ? "待机点" : type === "relocalization" ? "重定位点" : "自定义点位";
}

function visibleMapPoints() {
  const editingPoints = mapEditorActive && mapEditorLayer && mapEditorLayer.value === "points";
  const showAllPoints = customPointsToggle && customPointsToggle.checked;
  return editingPoints || showAllPoints ? mapPoints : [];
}

function syncMapEditorUi() {
  const dialogOpen = Boolean(mapEditorDialog && !mapEditorDialog.hidden);
  if (mapEditorBackdrop) mapEditorBackdrop.hidden = !dialogOpen;
  if (btnMapEditorOpen) {
    btnMapEditorOpen.textContent = mapEditorDirty ? "地图编辑 · 未保存" : "地图编辑";
    btnMapEditorOpen.classList.toggle("has-unsaved", mapEditorDirty);
    btnMapEditorOpen.setAttribute("aria-expanded", String(dialogOpen));
  }
  if (mapEditorFloor) mapEditorFloor.textContent = activeFloor && !isMappingFloor(activeFloor) ? activeFloor : "未选择已保存地图";
  if (mapWrapper) mapWrapper.classList.toggle("map-edit-mode", mapEditorActive);
  if (btnMapEditorToggle) btnMapEditorToggle.textContent = mapEditorActive ? "暂停编辑" : "继续编辑";
  if (btnMapEditorUndo) btnMapEditorUndo.disabled = mapEditorUndoStack.length === 0;
  if (btnMapEditorCancel) btnMapEditorCancel.disabled = !mapEditorDirty;
  if (btnMapEditorSave) btnMapEditorSave.disabled = !mapEditorDirty;
}

function setMapEditorActive(active) {
  mapEditorActive = Boolean(active && activePgm && !isMappingFloor(activeFloor));
  mapEditorPainting = false;
  mapEditorMovedPointId = "";
  syncMapEditorUi();
  renderScene();
}

function setMapEditorDialogOpen(open) {
  if (!mapEditorDialog) return false;
  if (open && (!activePgm || isMappingFloor(activeFloor))) {
    if (mapStatus) mapStatus.textContent = "地图编辑仅支持已保存地图，请先选择地图";
    return false;
  }
  mapEditorDialog.hidden = !open;
  if (!open && relocPickAction === "waypoint") {
    mapEditorResumeAfterPick = false;
    relocPickAction = "relocalize";
    resetRelocPickState("");
    if (relocPickToggle) {
      relocPickToggle.checked = false;
      syncRelocPickCursorClass();
    }
  }
  setMapEditorActive(open);
  syncMapEditorUi();
  if (open && mapEditorLayer) mapEditorLayer.focus();
  return true;
}

function markMapEditorDirty(layer) {
  if (layer) mapEditorDirtyLayers.add(layer);
  mapEditorDirty = mapEditorDirtyLayers.size > 0;
  syncMapEditorUi();
}

function clearMapEditorDirty(layer) {
  if (layer) mapEditorDirtyLayers.delete(layer);
  else mapEditorDirtyLayers.clear();
  mapEditorDirty = mapEditorDirtyLayers.size > 0;
  syncMapEditorUi();
}

function drawMapPointsOverlay() {
  visibleMapPoints().forEach((point) => {
    const pix = worldToMapPixels(point);
    if (!pix) return;
    const { sx, sy } = mapPixelToScreen(pix.mapX, pix.mapY);
    const color = point.type === "elevator" || point.type === "elevator_inside" ? "#a78bfa" : point.type === "elevator_waiting" ? "#f472b6" : point.type === "standby" ? "#22c55e" : point.type === "relocalization" ? "#38bdf8" : "#fb923c";
    ctx.save();
    ctx.fillStyle = color;
    ctx.strokeStyle = "#0f172a";
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(sx, sy, point.type === "custom" || point.type === "relocalization" ? 5 : 7, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    const label = String(point.name || point.id);
    const labelX = sx + 9;
    const labelBaseline = sy - 7;
    ctx.font = '11px "Fira Code", monospace';
    const labelWidth = Math.ceil(ctx.measureText(label).width) + 10;
    const labelTop = labelBaseline - 12;
    ctx.fillStyle = "rgba(15, 23, 42, 0.88)";
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.beginPath();
    if (typeof ctx.roundRect === "function") ctx.roundRect(labelX - 5, labelTop, labelWidth, 17, 5);
    else ctx.rect(labelX - 5, labelTop, labelWidth, 17);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#f8fafc";
    ctx.fillText(label, labelX, labelBaseline);
    ctx.restore();
  });
}

function renderMapWaypointList() {
  if (!mapWaypointList) return;
  if (!mapPoints.length) { mapWaypointList.innerHTML = '<span class="reloc-message-block">本地图暂无点位</span>'; return; }
  mapWaypointList.innerHTML = mapPoints.map((point) => `<div class="map-waypoint-item"><span><strong>${escapeHtml(point.name || point.id)}</strong>${escapeHtml(pointTypeLabel(point.type))} · ${Number(point.x).toFixed(2)}, ${Number(point.y).toFixed(2)}</span><button type="button" data-delete-map-point="${escapeHtml(point.id)}">删除</button></div>`).join("");
}

async function saveMapPoints() {
  if (!activeFloor || isMappingFloor(activeFloor)) throw new Error("请先选择已保存地图");
  const out = await fetchJson(`${API_BASE_URL}/api/maps/${encodeURIComponent(activeFloor)}/assets/points`, {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ points: mapPoints }),
  });
  mapPoints = cloneMapPointRows(Array.isArray(out.points) ? out.points : mapPoints);
  savedMapPoints = cloneMapPointRows(mapPoints);
  clearMapEditorDirty("points");
  renderMapWaypointList(); renderScene();
  return out;
}

function nextMapPointId(name) {
  const base = String(name || "point").trim().toLowerCase().replace(/[^a-z0-9_-]+/g, "_").replace(/^_+|_+$/g, "") || "point";
  let id = base, n = 2;
  while (mapPoints.some((point) => point.id === id)) { id = `${base}_${n}`; n += 1; }
  return id;
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
  const { x: ox, y: oy, yaw } = getMapOriginPose();
  const dx = Number(pose.x) - ox;
  const dy = Number(pose.y) - oy;
  const cosYaw = Math.cos(yaw);
  const sinYaw = Math.sin(yaw);
  // p_grid = R(-yaw) * (p_map - origin)
  const gx = (cosYaw * dx + sinYaw * dy) / resolution;
  const gy = (-sinYaw * dx + cosYaw * dy) / resolution;
  const mapX = gx;
  const mapY = activePgm.height - gy;
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
  const { x: ox, y: oy, yaw } = getMapOriginPose();
  const gx = mapX * resolution;
  const gy = (activePgm.height - mapY) * resolution;
  const cosYaw = Math.cos(yaw);
  const sinYaw = Math.sin(yaw);
  // p_map = R(yaw) * p_grid + origin
  const x = ox + cosYaw * gx - sinYaw * gy;
  const y = oy + sinYaw * gx + cosYaw * gy;
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
  const resolution = Number(activeMeta && activeMeta.resolution);
  if (!Number.isFinite(resolution) || resolution <= 0) {
    return;
  }

  ctx.save();
  ctx.lineWidth = 1;
  const gridMapPx = 0.5 / resolution;

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
    ctx.strokeStyle = Math.round(mx / gridMapPx) % 2 === 0 ? "rgba(34, 197, 94, 0.38)" : "rgba(34, 197, 94, 0.22)";
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
    ctx.strokeStyle = Math.round(my / gridMapPx) % 2 === 0 ? "rgba(34, 197, 94, 0.38)" : "rgba(34, 197, 94, 0.22)";
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
        semanticMap: !!(semanticMapToggle && semanticMapToggle.checked),
        customPoints: !!(customPointsToggle && customPointsToggle.checked),
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
    if (semanticMapToggle && typeof p.semanticMap === "boolean") { semanticMapToggle.checked = p.semanticMap; }
    if (customPointsToggle && typeof p.customPoints === "boolean") { customPointsToggle.checked = p.customPoints; }
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
  // Debug view: backend aggregates node list + lifecycle capability.
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), 8000);
  try {
    const res = await fetch(`${API_BASE_URL}/api/ros/debug/nodes`, { signal: ctrl.signal });
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

async function transitionLifecycle(robotId, component, transition) {
  const res = await fetch(`${API_BASE_URL}/api/ros/lifecycle/transition`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ robot_id: robotId, component, transition }),
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

async function setNodeLifecycle(nodeName, transition) {
  const res = await fetch(`${API_BASE_URL}/api/ros/debug/nodes/lifecycle_set`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ node_name: nodeName, transition }),
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

async function startupSelectedRobot(robotId, simMode = "sim") {
  const res = await fetch(`${API_BASE_URL}/api/ros/lifecycle/startup`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ robot_id: robotId, sim_mode: simMode }),
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

async function shutdownSelectedRobot(robotId) {
  const res = await fetch(`${API_BASE_URL}/api/ros/lifecycle/shutdown`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ robot_id: robotId }),
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

async function cancelSimBringupForRobot(robotId) {
  const rid = String(robotId || "").trim();
  if (!rid) {
    return;
  }
  try {
    await fetch(`${API_BASE_URL}/api/ros/lifecycle/startup-cancel`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ robot_id: rid }),
    });
  } catch {
    /* ignore */
  }
}

async function runSimBringupForRobot(rid) {
  if (!rid) {
    return;
  }
  if (simBringupInFlightByRobot[rid]) {
    return;
  }
  if (pruneStaleSimOnlinePhaseIfNeeded(rid)) {
    persistSimBringupPhasesToSession();
  }
  simBringupInFlightByRobot[rid] = true;
  delete simPostOfflineButtonUntilByRobot[rid];
  simBringupPhaseByRobot[rid] = "starting";
  simBringupGlobalStartingId = rid;
  renderRobotPresencePanel();
  try {
    await startupSelectedRobot(rid, "sim");
    // Backend may mark robot_status as mapping quickly; refresh floor labels early.
    try {
      await refreshFloorOptionsPreserveSelection();
    } catch {
      /* ignore */
    }
    // API 仅触发 sim_bringup.sh；按钮保持「仿真上线...」直到 /{id}/robot_status 心跳在线（见 renderRobotPresencePanel）
    const deadline = Date.now() + 120000;
    while (Date.now() < deadline) {
      await fetchRobotStatusCache();
      try {
        latestSnapshot = await fetchJson(`${API_BASE_URL}/api/robot/pose`);
      } catch {
        /* ignore */
      }
      renderRobotPresencePanel();
      const rows = mergePresenceRows();
      const row = rows.find((x) => x.id === rid);
      if (row && row.online) {
        try {
          await refreshFloorOptionsPreserveSelection();
        } catch {
          /* ignore */
        }
        appendLog(`仿真 ${rid}：已收到 /${rid}/robot_status`);
        break;
      }
      if (simBringupPhaseByRobot[rid] !== "starting") {
        break;
      }
      await new Promise((resolve) => setTimeout(resolve, 400));
    }
    if (simBringupPhaseByRobot[rid] === "starting") {
      simBringupPhaseByRobot[rid] = "idle";
      await cancelSimBringupForRobot(rid);
      appendLog(
        `仿真 ${rid}：120s 内未收到 robot_status，已恢复按钮；请查看 backend/logs/managed_${rid}.log 与 sim_bringup.sh 日志`
      );
    }
  } catch (e) {
    simBringupPhaseByRobot[rid] = "idle";
    await cancelSimBringupForRobot(rid);
    throw e;
  } finally {
    simBringupInFlightByRobot[rid] = false;
    simBringupGlobalStartingId = "";
    renderRobotPresencePanel();
    refreshRosNodesStatus().catch(() => {});
  }
}

function renderRosNodesStatus(data) {
  if (!rosRobotGroups || !rosNodesSummary) {
    return;
  }

  if (!data) {
    rosNodesSummary.textContent = "状态: 无法连接后端";
    rosRobotGroups.innerHTML = "";
    return;
  }

  const nodes = Array.isArray(data.nodes) ? data.nodes : [];
  const packageLabel = {
    simulate: "仿真",
    slam: "slam",
    system: "system",
    nav2: "navigation",
    web_backend: "web_backend",
    other: "other",
  };
  const classifyRobot = (nodeName) => {
    const raw = String(nodeName || "");
    const legacyRecorder = raw.match(/^\/robot_log_recorder_(robot[0-9a-z_-]*)$/i);
    if (legacyRecorder) return legacyRecorder[1];
    if (raw === "/open_delivery_web_tf_bridge") return "web_backend";
    if (raw.startsWith("/drawn_model/topdown_camera/")) return "simulation";
    const m = raw.match(/^\/([^/]+)\//);
    if (!m) {
      return /\/(gazebo|spawn_entity|robot_state_publisher|joint_state_publisher)$/.test(raw)
        ? "simulation"
        : "system";
    }
    const rid = String(m[1] || "").trim();
    if (/^robot[0-9a-z_-]*$/i.test(rid)) return rid;
    return "system";
  };
  const classifyPackage = (nodeName) => {
    const n = String(nodeName || "");
    if (n === "/open_delivery_web_tf_bridge") {
      return "web_backend";
    }
    // System ownership takes precedence over the physical child namespace.
    if (/\/(heartbeat|health_monitor|task_manager|semantic_location_query|map_server|robot_log_recorder(?:_robot[0-9a-z_-]*)?|log_bag)$/.test(n)) {
      return "system";
    }
    if (
      n.includes("/navigation/") ||
      /\/(bt_navigator|planner_server|controller_server|recoveries_server|waypoint_follower|behavior_server|smoother_server|velocity_smoother)$/.test(n) ||
      n.includes("/costmap")
    ) {
      return "nav2";
    }
    if (
      n.includes("/simulate/") ||
      n.startsWith("/drawn_model/topdown_camera/") ||
      /\/(gazebo|spawn_entity|robot_state_publisher|joint_state_publisher|diff_drive_controller|laser_controller|imu_plugin|camera_controller)$/.test(n)
    ) {
      return "simulate";
    }
    if (n.includes("/slam/")) {
      return "slam";
    }
    return "other";
  };
  const isAuxiliaryNode = (nodeName) =>
    /\/(transform_listener_impl_[^/]+|[^/]+_rclcpp_node|tf_listener|tf_static_listener)$/.test(
      String(nodeName || "")
    );

  const robotMap = new Map();
  nodes.forEach((n) => {
    const name = String(n && n.name ? n.name : "").trim();
    if (!name) return;
    const rid = classifyRobot(name);
    const pkg = classifyPackage(name);
    if (!robotMap.has(rid)) {
      robotMap.set(rid, { id: rid, packages: new Map(), count: 0 });
    }
    const group = robotMap.get(rid);
    if (!group.packages.has(pkg)) group.packages.set(pkg, []);
    group.packages.get(pkg).push(n);
    group.count += 1;
  });
  const robotGroups = Array.from(robotMap.values()).sort((a, b) => a.id.localeCompare(b.id));
  rosNodesSummary.textContent = `机器人组: ${robotGroups.length} · 节点总数: ${nodes.length}`;

  const renderNodeCard = (n) => {
    const name = String(n && n.name ? n.name : "");
    const lifecycle = n && n.lifecycle ? n.lifecycle : {};
    const lcAvailable = !!lifecycle.available;
    const lcState = String(lifecycle.state || (lcAvailable ? "unknown" : "missing"));
    const statusClass = n && n.running ? "running" : "stopped";
    const transitions = Array.isArray(n && n.lifecycle_transitions) ? n.lifecycle_transitions : [];
    const lcButtons = transitions
      .map(
        (transition) =>
          `<button type="button" data-node-lifecycle="1" data-node-name="${escapeHtml(
            name
          )}" data-transition="${escapeHtml(transition)}"${lcAvailable ? "" : " disabled"}>${escapeHtml(
            transition
          )}</button>`
      )
      .join("");
    const killDisabled = !(n && n.can_kill);
    const lifecycleMeta = transitions.length
      ? `<div class="ros-node-meta">Lifecycle: <code>${escapeHtml(lcState)}</code> ${
          lcAvailable ? "" : "（不可用）"
        }</div>`
      : `<div class="ros-node-meta">Lifecycle: <code>n/a</code>（该模块暂不开放）</div>`;
    const lifecycleActions = transitions.length
      ? `<div class="ros-node-actions">
          ${lcButtons || '<button type="button" disabled>lifecycle 不可用</button>'}
        </div>`
      : "";
    const auxiliaryClass = isAuxiliaryNode(name) ? " ros-node-card--auxiliary" : "";
    return `
      <article class="ros-node-card ${n && n.running ? "" : "ros-node-card--missing"}${auxiliaryClass}">
        <div class="ros-node-head">
          <div class="ros-node-head__titles">
            <strong><code>${escapeHtml(name)}</code></strong>
          </div>
          <span class="ros-node-badge ${statusClass}">${n && n.running ? "运行中" : "未运行"}</span>
        </div>
        ${lifecycleMeta}
        <div class="ros-node-controls">
          ${lifecycleActions}
          <div class="ros-node-actions">
            <button type="button" data-node-kill="1" data-node-name="${escapeHtml(name)}"${
      killDisabled ? " disabled" : ""
    }>kill</button>
          </div>
        </div>
      </article>
    `;
  };

  rosRobotGroups.innerHTML = robotGroups
    .map((group) => {
      const pkgOrder = ["simulate", "slam", "system", "nav2", "web_backend", "other"];
      const pkgBlocks = pkgOrder
        .filter((kind) => group.packages.has(kind))
        .map((kind) => {
          const items = group.packages.get(kind) || [];
          return `
            <div class="ros-package-block">
              <div class="ros-package-title">${escapeHtml(packageLabel[kind] || kind)} · ${items.length}</div>
              <div class="ros-nodes-list">
                ${items.map(renderNodeCard).join("")}
              </div>
            </div>
          `;
        })
        .join("");
      return `
        <section class="ros-robot-group">
          <div class="ros-section-title">
            ${escapeHtml(group.id)}
            <span class="ros-section-trace"> · ${group.count} nodes</span>
          </div>
          ${pkgBlocks || '<div class="ros-node-meta">暂无节点</div>'}
        </section>
      `;
    })
    .join("");
}

function setRosNodeButtonsDisabled(disabled) {
  if (rosRobotGroups) {
    rosRobotGroups.querySelectorAll("button[data-node-lifecycle], button[data-node-kill]").forEach((b) => {
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
  if (!rosRobotGroups) {
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
  rosRobotGroups.addEventListener("click", async (ev) => {
    const lifeBtn = ev.target.closest("button[data-node-lifecycle]");
    const killBtn = ev.target.closest("button[data-node-kill]");
    if (!lifeBtn && !killBtn) return;
    setRosNodeButtonsDisabled(true);
    if (rosNodesError) {
      rosNodesError.textContent = "";
    }
    try {
      if (lifeBtn) {
        const nodeName = String(lifeBtn.dataset.nodeName || "").trim();
        const transition = String(lifeBtn.dataset.transition || "").trim();
        if (!nodeName || !transition) return;
        await setNodeLifecycle(nodeName, transition);
        appendLog(`${nodeName} -> ${transition} 成功`);
      } else if (killBtn) {
        const nodeName = String(killBtn.dataset.nodeName || "").trim();
        if (!nodeName) return;
        await killDiscoveredRosNode(nodeName);
        appendLog(`kill ${nodeName} 成功`);
      }
    } catch (err) {
      if (rosNodesError) {
        rosNodesError.textContent = err.message || String(err);
      }
      appendLog(`ROS 调试操作失败: ${err.message || err}`);
    } finally {
      setRosNodeButtonsDisabled(false);
      refreshRosNodesStatus().catch(() => {});
    }
  });
  refreshRosNodesStatus().catch(() => {});
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
  relocPickAction = "relocalize";
  if (mapEditorResumeAfterPick) {
    mapEditorResumeAfterPick = false;
    setMapEditorActive(true);
  }
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

async function publishPickedNavigationGoal(goal) {
  const rid = relocRobotId && relocRobotId.value ? relocRobotId.value.trim() : "";
  if (!rid) {
    if (relocMessage) {
      relocMessage.textContent = "请填写机器人 ID，未下发导航目标";
    }
    return;
  }
  if (relocMessage) {
    relocMessage.textContent = `正在向 ${rid} 下发导航目标…`;
  }
  try {
    const result = await postRobotGoto(rid, goal.x, goal.y, goal.yaw, activeFloor || "");
    if (relocMessage) {
      relocMessage.textContent = `任务 ${result.task_id || ""} 已提交给 ${rid}`.trim();
    }
    appendLog(`下发导航目标 → ${rid} (${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}, ${goal.yaw.toFixed(2)})`);
  } catch (err) {
    if (relocMessage) {
      relocMessage.textContent = err.message || String(err);
    }
  }
}

function finishRelocMapPick(yaw) {
  const action = relocPickAction;
  const anchor = relocPickAnchorWorld;
  if (action === "waypoint" && anchor) {
    if (relocYaw) relocYaw.value = String(Number(yaw.toFixed(4)));
    if (waypointX) waypointX.value = String(Number(anchor.x.toFixed(3)));
    if (waypointY) waypointY.value = String(Number(anchor.y.toFixed(3)));
    if (waypointYaw) waypointYaw.value = String(Number(yaw.toFixed(4)));
    exitRelocPickModeAfterDone("点位位置已填入，填写名称后保存");
    if (waypointMessage) waypointMessage.textContent = "点位位置已填入，填写名称后保存";
    return;
  }
  if (action === "goto" && anchor) {
    const goal = { x: anchor.x, y: anchor.y, yaw };
    exitRelocPickModeAfterDone("目标点已设置，正在下发导航…");
    publishPickedNavigationGoal(goal);
    return;
  }
  exitRelocPickModeAfterDone("已设置朝向，可下发重定位");
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
    if (relocPickAction === "waypoint") {
      if (waypointX) waypointX.value = String(Number(world.x.toFixed(3)));
      if (waypointY) waypointY.value = String(Number(world.y.toFixed(3)));
    }
    if (relocMessage) {
      relocMessage.textContent =
        relocPickAction === "goto"
          ? "已定导航目标位置，再点击地图设置朝向（或「跳过朝向」）"
          : relocPickAction === "waypoint"
            ? "已定点位位置，再点击地图设置朝向（或「跳过朝向」）"
            : "已定点，再点击地图设置朝向（或「跳过朝向」）";
    }
    renderScene();
    return true;
  }

  const yaw = Math.atan2(world.y - relocPickAnchorWorld.y, world.x - relocPickAnchorWorld.x);
  if (relocYaw) {
    relocYaw.value = String(Number(yaw.toFixed(4)));
  }
  finishRelocMapPick(yaw);
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
  if (!activePgm || !points || points.length === 0) {
    return;
  }
  ctx.save();
  ctx.strokeStyle = "rgba(56, 189, 248, 0.98)";
  ctx.fillStyle = "#38bdf8";
  ctx.lineWidth = 3;
  ctx.lineJoin = "round";
  ctx.shadowColor = "rgba(56, 189, 248, 0.65)";
  ctx.shadowBlur = 5;
  ctx.beginPath();
  let started = false;
  let lastScreen = null;
  points.forEach((pt) => {
    if (!pt || pt.length < 2) {
      return;
    }
    const pix = worldToMapPixels({ x: pt[0], y: pt[1] });
    if (!pix) {
      return;
    }
    const { sx, sy } = mapPixelToScreen(pix.mapX, pix.mapY);
    lastScreen = { sx, sy };
    if (!started) {
      ctx.moveTo(sx, sy);
      started = true;
    } else {
      ctx.lineTo(sx, sy);
    }
  });
  if (started) {
    ctx.stroke();
  }
  if (lastScreen) {
    ctx.beginPath();
    ctx.arc(lastScreen.sx, lastScreen.sy, 4, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.restore();
}

function drawScan2dOnMap(data, robot) {
  if (!activePgm || !data || !data.hits || data.hits.length === 0) {
    return;
  }
  const robotPose = robot && robot.pose ? robot.pose : { x: 0, y: 0, yaw: 0 };
  const robotBaseCoordinates = data.coordinates === "robot_base";
  const robotYaw = Number(robotPose && robotPose.yaw) || 0;
  const cosYaw = Math.cos(robotYaw);
  const sinYaw = Math.sin(robotYaw);
  ctx.save();
  // New bridge payloads are already in map coordinates. Keep robot_base support
  // for older payloads; without localization its fallback pose is (0, 0, 0).
  const ptRadius = Math.max(1.25, Math.min(3.5, viewScale * 0.2));
  ctx.fillStyle = "#ef4444";
  data.hits.forEach((hit) => {
    if (!hit || hit.length < 2) {
      return;
    }
    const world = robotBaseCoordinates
      ? {
          x: robotPose.x + cosYaw * hit[0] - sinYaw * hit[1],
          y: robotPose.y + sinYaw * hit[0] + cosYaw * hit[1],
        }
      : { x: hit[0], y: hit[1] };
    const pix = worldToMapPixels(world);
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

/** 在线优先用话题实时值，否则用持久化缓存（robot_status / task_status 展示用） */
function pickPresenceStatusField(r, liveKey, persistedKey) {
  const live = String((r && r[liveKey]) || "").trim();
  const persisted = String((r && r[persistedKey]) || "").trim();
  if (r && r.online && live) {
    return { value: live, src: "实时" };
  }
  if (persisted) {
    return { value: persisted, src: "持久化" };
  }
  if (live) {
    return { value: live, src: "实时" };
  }
  return { value: "—", src: "" };
}

function renderPresenceStatusBlock(r) {
  const rs = pickPresenceStatusField(r, "liveRobotStatus", "persistedRobotStatus");
  const ts = pickPresenceStatusField(r, "liveTaskStatus", "persistedTaskStatus");
  const src = (x) =>
    x.src ? `<span class="robot-presence-status__src">${escapeHtml(x.src)}</span>` : "";
  const progress = typeof r.taskProgress === "number" ? r.taskProgress : -1;
  const progressHtml =
    progress >= 0 && progress <= 1
      ? `<div class="robot-presence-status__row">
      <span class="robot-presence-status__k">progress</span>
      <span class="robot-presence-status__v">
        <span class="task-progress-bar" title="${Math.round(progress * 100)}%">
          <span class="task-progress-bar__fill" style="width:${Math.round(progress * 100)}%"></span>
        </span>
        <span class="task-progress-bar__label">${Math.round(progress * 100)}%</span>
      </span>
    </div>`
      : "";
  return `<div class="robot-presence-status" aria-label="robot_status 与 task_status">
    <div class="robot-presence-status__row">
      <span class="robot-presence-status__k">robot_status</span>
      <span class="robot-presence-status__v"><code>${escapeHtml(rs.value)}</code>${src(rs)}</span>
    </div>
    <div class="robot-presence-status__row">
      <span class="robot-presence-status__k">task_status</span>
      <span class="robot-presence-status__v"><code>${escapeHtml(ts.value)}</code>${src(ts)}</span>
    </div>
    ${progressHtml}
  </div>`;
}

function normalizeNewPresenceRobotId(raw) {
  let s = String(raw || "").trim();
  s = s.replace(/^\/+|\/+$/g, "");
  return s;
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
  if (robotStatusCacheItems.length > 0 && robotStatusCacheItems.some((x) => x && x.id)) {
    const rows = robotStatusCacheItems
      .map((it) => {
        const id = String((it && it.id) || "").trim();
        if (!id) return null;
        return {
          id,
          name: String((it && it.name) || id),
          model: String((it && it.robot_model) || "").trim(),
          floor: String((it && it.floor) || ""),
          online: !!(it && it.online),
          persistedCurrentMap: String((it && it.persisted_current_map) || "").trim(),
          persistedRobotStatus: String((it && it.persisted_robot_status) || "").trim(),
          persistedTaskStatus: String((it && it.persisted_task_status) || "").trim(),
          liveRobotStatus: String((it && it.live_robot_status) || "").trim(),
          liveTaskStatus: String((it && it.live_task_status) || "").trim(),
          persistedIsSim: !!(it && it.persisted_is_simulation),
          simButton: it && it.sim_button ? it.sim_button : null,
          taskProgress: typeof (it && it.task_progress) === "number" ? it.task_progress : -1,
        };
      })
      .filter(Boolean);
    rows.sort((a, b) => a.id.localeCompare(b.id));
    return rows;
  }
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
    const model = String((liveR && liveR.robot_model) || (cache && cache.robot_model) || "").trim();
    const floor =
      (liveR && snapshotRobotFloor(liveR)) ||
      (cache && String(cache.current_map || "").trim()) ||
      "";
    const persistedRobotStatus = cache ? String(cache.robot_status || "").trim() : "";
    const persistedTaskStatus = cache ? String(cache.task_status || "").trim() : "";
    const persistedCurrentMap = cache ? String(cache.current_map || "").trim() : "";
    const persistedIsSim = !!(cache && cache.is_simulation);
    const liveRobotStatus = liveR ? String(liveR.robot_status || "").trim() : "";
    const liveTaskStatus = liveR ? String(liveR.task_status || "").trim() : "";
    const taskProgress =
      liveR && typeof liveR.task_progress === "number" ? liveR.task_progress : -1;
    rows.push({
      id,
      name,
      model,
      floor,
      online,
      persistedCurrentMap,
      persistedRobotStatus,
      persistedTaskStatus,
      liveRobotStatus,
      liveTaskStatus,
      persistedIsSim,
      taskProgress,
    });
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
    if (robotPresenceListMarkup !== "") {
      robotPresenceList.innerHTML = "";
      robotPresenceListMarkup = "";
    }
    robotPresenceEmpty.hidden = false;
    return;
  }
  robotPresenceEmpty.hidden = true;
  const nowMs = Date.now();
  Object.keys(simPostOfflineButtonUntilByRobot).forEach((k) => {
    if (simPostOfflineButtonUntilByRobot[k] <= nowMs) {
      delete simPostOfflineButtonUntilByRobot[k];
    }
  });
  rows.forEach((r) => {
    if (r.simButton) {
      return;
    }
    const id = r.id;
    if (r.online) {
      delete simPostOfflineButtonUntilByRobot[id];
    }
    if (simBringupPhaseByRobot[id] === "starting" && r.online) {
      simBringupPhaseByRobot[id] = "sim_online";
      if (simBringupGlobalStartingId === id) {
        simBringupGlobalStartingId = "";
      }
    }
    // sessionStorage 可能残留 sim_online；当前已无 /…/robot_status 时不应再显示「仿真离线」
    pruneStaleSimOnlinePhaseIfNeeded(id);
  });
  const nextMarkup = rows
    .map((r) => {
      const selectedCls = selectedPresenceRobotId === r.id ? " robot-presence-item--selected" : "";
      const badge = r.online
        ? '<span class="robot-presence-badge robot-presence-badge--online">在线</span>'
        : '<span class="robot-presence-badge robot-presence-badge--offline">离线</span>';
      const floorLine = r.floor
        ? `<div class="robot-presence-list__meta">地图: ${escapeHtml(r.floor)}</div>`
        : "";
      const lastCurrentMapLine = r.persistedCurrentMap
        ? `<div class="robot-presence-list__lastmap"><span class="robot-presence-lastmap__k">上次 current_map</span> <code>${escapeHtml(
            r.persistedCurrentMap
          )}</code></div>`
        : "";
      const simTag = r.persistedIsSim
        ? '<span class="robot-presence-sim-tag" title="持久化记录曾标记为仿真">上次仿真</span>'
        : "";
      const statusBlock = renderPresenceStatusBlock(r);
      const phase = simBringupPhaseByRobot[r.id] || "idle";
      let simLabel = "仿真上线";
      let simAction = "bringup";
      let simDisabled = false;
      let simTitle = "";
      let simExtraClass = "";
      if (simBringupInFlightByRobot[r.id] || phase === "starting") {
        simLabel = "仿真上线...";
        simAction = "pending";
        simDisabled = true;
      } else if (r.simButton) {
        const sb = r.simButton || {};
        simLabel = String(sb.label || simLabel);
        simAction = String(sb.action || simAction);
        simDisabled = !!sb.disabled;
        simTitle = String(sb.title || "");
        simExtraClass = sb.extra_class ? ` ${String(sb.extra_class)}` : "";
      } else if (phase === "sim_online") {
        simExtraClass = " btn-sim-bringup--offline";
        if (simShutdownBusyByRobot[r.id]) {
          simLabel = "仿真离线...";
          simAction = "pending";
          simDisabled = true;
        } else {
          simLabel = "仿真离线";
          simAction = "shutdown";
          simDisabled = false;
        }
      } else if (phase === "idle" && r.online) {
        simLabel = "已在线";
        simAction = "pending";
        simDisabled = true;
        simTitle = "已检测到 /…/robot_status，避免重复仿真上线";
      } else if (
        phase === "idle" &&
        !r.online &&
        simPostOfflineButtonUntilByRobot[r.id] &&
        nowMs < simPostOfflineButtonUntilByRobot[r.id]
      ) {
        simLabel = "离线";
        simAction = "pending";
        simDisabled = true;
        simExtraClass = " btn-sim-bringup--offline";
      } else if (phase === "idle" && !r.online && simBringupGlobalStartingId && simBringupGlobalStartingId !== r.id) {
        simDisabled = true;
        simTitle = "另一台机器人正在仿真上线中";
      }
      const simBtn = `<button type="button" class="btn-sim-bringup${simExtraClass}" data-sim-bringup="1" data-sim-action="${simAction}" data-robot-id="${escapeHtml(
        r.id
      )}"${simDisabled ? " disabled" : ""}${simTitle ? ` title="${escapeHtml(simTitle)}"` : ""}>${escapeHtml(
        simLabel
      )}</button>`;
      return `<li class="${selectedCls.trim()}" data-presence-robot-id="${escapeHtml(r.id)}">
        <div>
          <div class="robot-presence-list__id">${escapeHtml(r.name)}${simTag}</div>
          <div class="robot-presence-list__meta">${escapeHtml(r.id)}</div>
          ${floorLine}
          ${lastCurrentMapLine}
          ${statusBlock}
        </div>
        <div class="robot-presence-row__tail">
          ${badge}
          ${simBtn}
        </div>
      </li>`;
    })
    .join("");
  if (nextMarkup !== robotPresenceListMarkup) {
    robotPresenceList.innerHTML = nextMarkup;
    robotPresenceListMarkup = nextMarkup;
  }
  persistSimBringupPhasesToSession();
  renderRobotQuickDock();
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
    if (robotPresenceNewMsg) {
      robotPresenceNewMsg.textContent = "";
    }
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
  if (btnRobotPresenceBringupNew && robotPresenceNewId) {
    robotPresenceNewId.addEventListener("keydown", (ev) => {
      if (ev.key === "Enter") {
        ev.preventDefault();
        btnRobotPresenceBringupNew.click();
      }
    });
    btnRobotPresenceBringupNew.addEventListener("click", async (ev) => {
      ev.preventDefault();
      ev.stopPropagation();
      const rid = normalizeNewPresenceRobotId(robotPresenceNewId.value);
      if (!rid) {
        if (robotPresenceNewMsg) {
          robotPresenceNewMsg.textContent = "请填写机器人 ID";
        }
        return;
      }
      if (!/^[\w-]+$/.test(rid)) {
        if (robotPresenceNewMsg) {
          robotPresenceNewMsg.textContent = "仅允许字母、数字、下划线与连字符";
        }
        return;
      }
      const rows = mergePresenceRows();
      if (rows.some((x) => x.id === rid)) {
        if (robotPresenceNewMsg) {
          robotPresenceNewMsg.textContent = "该 ID 已在上方列表中，请用对应行的按钮";
        }
        return;
      }
      if (robotPresenceNewMsg) {
        robotPresenceNewMsg.textContent = `正在上线 ${rid}…`;
      }
      btnRobotPresenceBringupNew.disabled = true;
      try {
        await runSimBringupForRobot(rid);
        if (robotPresenceNewMsg) {
          robotPresenceNewMsg.textContent = "已下发启动脚本，等待 /…/robot_status";
        }
        robotPresenceNewId.value = "";
        await fetchRobotStatusCache();
        try {
          latestSnapshot = await fetchJson(`${API_BASE_URL}/api/robot/pose`);
        } catch {
          /* ignore */
        }
        renderRobotPresencePanel();
        updateRobotStatus();
        scheduleMapPaint();
      } catch (err) {
        if (robotPresenceNewMsg) {
          robotPresenceNewMsg.textContent = err.message || String(err);
        }
        appendLog(`新机器人仿真上线失败: ${err.message || err}`);
      } finally {
        btnRobotPresenceBringupNew.disabled = false;
      }
    });
  }
  if (robotPresenceList) {
    // Pose SSE can refresh the panel many times per second. Trigger the action
    // on the primary pointer press so a concurrent status render cannot replace
    // the button between pointerdown and the browser-generated click event.
    robotPresenceList.addEventListener("pointerdown", (ev) => {
      if ((ev.button != null && ev.button !== 0) || ev.isPrimary === false) {
        return;
      }
      const target = ev.target instanceof Element ? ev.target : null;
      const simBtn = target && target.closest("button[data-sim-bringup]");
      if (!simBtn || simBtn.disabled || simBtn.dataset.simAction === "pending") {
        return;
      }
      ev.preventDefault();
      ev.stopPropagation();
      simBtn.focus({ preventScroll: true });
      simBtn.click();
    });
    robotPresenceList.addEventListener("click", async (ev) => {
      const simBtn = ev.target.closest("button[data-sim-bringup]");
      if (simBtn) {
        ev.preventDefault();
        ev.stopPropagation();
        const rid = String(simBtn.dataset.robotId || "").trim();
        const action = String(simBtn.dataset.simAction || "").trim();
        if (!rid || simBtn.disabled) {
          return;
        }
        if (action === "pending") {
          return;
        }
        if (action === "shutdown") {
          if (simShutdownBusyByRobot[rid]) {
            return;
          }
          simShutdownBusyByRobot[rid] = true;
          renderRobotPresencePanel();
          try {
            await shutdownSelectedRobot(rid);
            simBringupPhaseByRobot[rid] = "idle";
            simPostOfflineButtonUntilByRobot[rid] = Date.now() + 4000;
            appendLog(`仿真离线 ${rid} 完成`);
          } catch (err) {
            appendLog(`仿真离线失败: ${err.message || err}`);
          } finally {
            simShutdownBusyByRobot[rid] = false;
            renderRobotPresencePanel();
            refreshRosNodesStatus().catch(() => {});
          }
          return;
        }
        if (action === "bringup") {
          try {
            await runSimBringupForRobot(rid);
          } catch (err) {
            appendLog(`仿真上线失败: ${err.message || err}`);
            renderRobotPresencePanel();
          }
        }
        return;
      }
      const li = ev.target.closest("li[data-presence-robot-id]");
      if (!li) {
        return;
      }
      selectedPresenceRobotId = String(li.dataset.presenceRobotId || "").trim();
      renderRobotPresencePanel();
    });
  }
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
  const editingSemantic = mapEditorActive && mapEditorLayer && mapEditorLayer.value === "semantic";
  const sem = editingSemantic ? semanticEditCanvas : semanticBitmap;
  if (sem && ((semanticMapToggle && semanticMapToggle.checked) || editingSemantic)) {
    ctx.save(); ctx.globalAlpha = 0.38; ctx.drawImage(sem, viewPanX, viewPanY, mw * viewScale, mh * viewScale); ctx.restore();
  }

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
        drawScan2dOnMap(sd, r);
      }
    });
  }

  drawMapPointsOverlay();

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

function syncOnlineRobotSelect() {
  if (!relocRobotId) return;
  const previous = relocRobotId.value;
  const online = mergePresenceRows().filter((row) => row.online);
  const fallback = latestSnapshot && Array.isArray(latestSnapshot.robots)
    ? latestSnapshot.robots.filter((row) => row && row.heartbeat_online !== false).map((row) => ({ id: row.id, name: row.name || row.id, model: row.robot_model || "" })) : [];
  const rows = online.length ? online : fallback;
  const clean = [], seen = new Set();
  rows.forEach((row) => { const id = String(row.id || "").trim(); if (id && !seen.has(id)) { seen.add(id); clean.push({ id, model:String(row.model||"").trim() }); } });
  const signature = JSON.stringify(clean);
  if (signature === relocRobotOptionsSignature) return;
  relocRobotOptionsSignature = signature;
  relocRobotId.innerHTML = "";
  clean.forEach((row) => {
    const option = document.createElement("option"); option.value = row.id; option.textContent = row.model?`${row.id}(${row.model})`:row.id; relocRobotId.appendChild(option);
  });
  if (!clean.length) { relocRobotId.innerHTML = '<option value="">暂无在线机器人</option>'; relocRobotId.disabled = true; }
  else { relocRobotId.disabled = false; relocRobotId.value = seen.has(previous) ? previous : clean[0].id; }
}

function updateRobotStatus() {
  try {
    syncOnlineRobotSelect();
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
  } finally {
    updateRobotPresenceTriggerSummary();
    renderRobotQuickDock();
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
    if (mapEditorDialog) mapEditorDialog.hidden = true;
    mapEditorActive = false;
    mapEditorPainting = false;
    mapEditorResumeAfterPick = false;
    clearMapEditorDirty();
    syncMapEditorUi();
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
    syncMapEditorUi();
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
    if (mapEditorDialog) mapEditorDialog.hidden = true;
    mapEditorActive = false;
    mapEditorPainting = false;
    mapEditorResumeAfterPick = false;
    clearMapEditorDirty();
    syncMapEditorUi();
    activePgm = pgm;
    activeMeta = parseYaml(yamlText);
    mapBitmap = buildMapBitmap(pgm);
    await loadActiveMapAssets();
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
    scheduleMapPaint();
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
    scheduleMapPaint();
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

function canvasEventMapPixel(ev) {
  const rect = canvas.getBoundingClientRect();
  return screenToMapPixel(ev.clientX - rect.left, ev.clientY - rect.top);
}

function nearestMapPoint(mapX, mapY, radiusPx = 14) {
  let best = null, bestD = radiusPx / Math.max(viewScale, 0.01);
  mapPoints.forEach((point) => {
    const pix = worldToMapPixels(point); if (!pix) return;
    const d = Math.hypot(pix.mapX - mapX, pix.mapY - mapY);
    if (d <= bestD) { bestD = d; best = point; }
  });
  return best;
}

function paintMapEditorAt(ev, recordUndo = false) {
  if (!mapEditorActive || !activePgm || ev.shiftKey) return false;
  const { mapX, mapY } = canvasEventMapPixel(ev);
  if (mapX < 0 || mapY < 0 || mapX >= activePgm.width || mapY >= activePgm.height) return true;
  const layer = mapEditorLayer.value;
  const tool = mapEditorTool.value;
  if (layer === "points") {
    if (tool === "add") {
      const name = String(waypointName && waypointName.value || "").trim();
      const world = mapPixelsToWorld(mapX, mapY);
      if (!name || !world) {
        if (mapEditorMessage) mapEditorMessage.textContent = "请先在设置点位中填写名称和语义类型，再点击地图";
        return true;
      }
      if (recordUndo) pushMapEditorUndoSnapshot("points");
      mapPoints.push({ id: nextMapPointId(name), name, type: waypointType.value, x: world.x, y: world.y, yaw: Number(waypointYaw && waypointYaw.value || 0) || 0 });
      waypointName.value = "";
      markMapEditorDirty("points"); renderMapWaypointList(); renderScene();
      if (mapEditorMessage) mapEditorMessage.textContent = "点位已添加，点击保存地图修改后落盘";
      return true;
    }
    const point = nearestMapPoint(mapX, mapY);
    if (tool === "delete" && point) {
      if (recordUndo) pushMapEditorUndoSnapshot("points");
      mapPoints = mapPoints.filter((row) => row.id !== point.id); markMapEditorDirty("points"); renderMapWaypointList(); renderScene();
    } else if (tool === "move") {
      if (!mapEditorMovedPointId && point) {
        if (recordUndo) pushMapEditorUndoSnapshot("points");
        mapEditorMovedPointId = point.id;
      }
      const moving = mapPoints.find((row) => row.id === mapEditorMovedPointId);
      const world = mapPixelsToWorld(mapX, mapY);
      if (moving && world) { moving.x = world.x; moving.y = world.y; markMapEditorDirty("points"); renderScene(); }
    }
    return true;
  }
  const c = layer === "semantic" ? (semanticEditCanvas || (semanticEditCanvas = createBlankSemanticCanvas())) : mapBitmap;
  if (!c) return true;
  if (recordUndo) pushMapEditorUndoSnapshot(layer);
  const cctx = c.getContext("2d");
  const radius = Math.max(1, Number(mapEditorBrush.value || 4));
  cctx.save(); cctx.beginPath(); cctx.arc(mapX, mapY, radius, 0, Math.PI * 2);
  if (layer === "raster") {
    const val = tool === "erase" ? 254 : Number(mapEditorRasterValue.value || 0);
    cctx.fillStyle = `rgb(${val},${val},${val})`;
  } else {
    cctx.fillStyle = tool === "erase" ? "#ffffff" : (mapEditorSemanticLabel.value || "#ffffff");
  }
  cctx.fill(); cctx.restore(); markMapEditorDirty(layer); renderScene(); return true;
}

function canvasToPgmDataUrl(c) {
  const cctx = c.getContext("2d");
  const img = cctx.getImageData(0, 0, c.width, c.height).data;
  const header = `P5\n${c.width} ${c.height}\n255\n`;
  const bytes = new Uint8Array(header.length + c.width * c.height);
  for (let i = 0; i < header.length; i += 1) bytes[i] = header.charCodeAt(i);
  for (let i = 0; i < c.width * c.height; i += 1) bytes[header.length + i] = img[i * 4];
  let binary = ""; const chunk = 0x8000;
  for (let i = 0; i < bytes.length; i += chunk) binary += String.fromCharCode(...bytes.subarray(i, i + chunk));
  return "data:image/x-portable-graymap;base64," + btoa(binary);
}

async function saveActiveMapEditor() {
  const layers = mapEditorDirtyLayers.size ? Array.from(mapEditorDirtyLayers) : [mapEditorLayer.value];
  for (const layer of layers) {
    if (layer === "points") {
      await saveMapPoints();
    } else if (layer === "raster") {
      await fetchJson(`${API_BASE_URL}/api/maps/${encodeURIComponent(activeFloor)}/assets/raster`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ pgm_data: canvasToPgmDataUrl(mapBitmap) }) });
      activePgm = { ...activePgm, data: Array.from(mapBitmap.getContext("2d").getImageData(0, 0, mapBitmap.width, mapBitmap.height).data).filter((_, i) => i % 4 === 0) };
      clearMapEditorDirty("raster");
    } else if (layer === "semantic") {
      const out = semanticEditCanvas || createBlankSemanticCanvas();
      await fetchJson(`${API_BASE_URL}/api/maps/${encodeURIComponent(activeFloor)}/assets/semantic`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ png_data: out.toDataURL("image/png") }) });
      semanticBitmap = cloneCanvasElement(out);
      clearMapEditorDirty("semantic");
    }
  }
  mapEditorDirty = mapEditorDirtyLayers.size > 0;
  if (!mapEditorDirty) clearMapEditorUndoHistory();
  syncMapEditorUi();
}

function onMouseDown(ev) {
  if (ev.button !== 0) {
    return;
  }
  if (mapEditorActive && !ev.shiftKey && paintMapEditorAt(ev, true)) { mapEditorPainting = true; return; }
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
  if (mapEditorPainting && mapEditorActive) { paintMapEditorAt(ev); return; }
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
  const movedPoint = mapEditorMovedPointId;
  mapEditorPainting = false;
  mapEditorMovedPointId = "";
  if (movedPoint) renderMapWaypointList();
  isDragging = false;
  canvas.style.cursor = "";
}

function stopSensorPolling() {
  if (sensorPollTimer) {
    clearInterval(sensorPollTimer);
    sensorPollTimer = null;
  }
}

function stopScanStream() {
  if (scanStream) {
    scanStream.close();
    scanStream = null;
  }
  scanStreamActive = false;
}

function startScanStream() {
  if (
    scanStream ||
    Date.now() < scanStreamRetryAt ||
    !(scan2dToggle && scan2dToggle.checked)
  ) {
    return;
  }
  const eventSource = new EventSource(`${API_BASE_URL}/api/robot/scan/stream`);
  scanStream = eventSource;
  eventSource.onopen = () => {
    scanStreamActive = true;
    scanStreamRetryAt = 0;
  };
  eventSource.onmessage = (event) => {
    if (!(scan2dToggle && scan2dToggle.checked)) {
      return;
    }
    let payload;
    try {
      payload = JSON.parse(event.data);
    } catch {
      return;
    }
    const scans =
      payload && payload.scans && typeof payload.scans === "object" ? payload.scans : {};
    Object.entries(scans).forEach(([robotId, data]) => {
      if (data && data.available && Array.isArray(data.hits)) {
        latestScanByRobot[robotId] = data;
      } else {
        delete latestScanByRobot[robotId];
      }
    });
    scheduleMapPaint();
  };
  eventSource.onerror = () => {
    if (scanStream === eventSource) {
      scanStreamRetryAt = Date.now() + 5000;
      stopScanStream();
      if (scan2dToggle && scan2dToggle.checked) {
        startSensorPolling();
      }
    }
  };
}

function stopMapLivePolling() {
  if (mapLiveTimer) {
    clearInterval(mapLiveTimer);
    mapLiveTimer = null;
  }
  mapLiveInitializedView = false;
}

async function applyLiveMappingFrame() {
  if (mapLiveInFlight) {
    return;
  }
  const rid = activeMappingRobotId;
  if (!rid) {
    return;
  }
  mapLiveInFlight = true;
  try {
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
      // Keep origin text format for existing renderer helpers; yaw is stored separately.
      origin: `[${data.origin[0]}, ${data.origin[1]}, 0]`,
      origin_yaw: String(data.origin_yaw || 0),
      occupied_thresh: "0.65",
      free_thresh: "0.196",
    };
    // OccupancyGrid data are row-major from grid (0,0) at lower-left; flip Y once
    // during rasterization so world<->pixel conversion matches saved-map behavior.
    mapBitmap = buildMapBitmap(pgm, { flipY: true });
    if (!mapLiveInitializedView) {
      resetViewToFit();
      mapLiveInitializedView = true;
    }
    renderScene();
    refreshMetaPanel();
    mapStatus.textContent = `建图 ${rid} ${data.width}×${data.height} · res ${data.resolution} m/cell`;
  } finally {
    mapLiveInFlight = false;
  }
}

function startMapLivePolling() {
  stopMapLivePolling();
  mapLiveInitializedView = false;
  applyLiveMappingFrame().catch(() => {});
  mapLiveTimer = setInterval(() => {
    applyLiveMappingFrame().catch(() => {});
  }, 400);
}

async function postSaveMap(mapName, robotIdForMapping = "") {
  const body = { map_name: mapName };
  const rid = String(robotIdForMapping || "").trim();
  if (rid) {
    body.robot_id = rid;
  }
  const res = await fetch(`${API_BASE_URL}/api/mapping/save`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
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
  if (wantScan) {
    startScanStream();
  } else {
    stopScanStream();
  }
  if (!wantScan && !wantPath) {
    return;
  }
  sensorPollTimer = setInterval(async () => {
    if (sensorPollInFlight) {
      return;
    }
    sensorPollInFlight = true;
    try {
      const here = getRobotsOnCurrentMap();
      if (here.length === 0) {
        return;
      }
      const ws = scan2dToggle && scan2dToggle.checked;
      const wp = plannedPathToggle && plannedPathToggle.checked;
      await Promise.all(
        here.map(async (r) => {
        const id = encodeURIComponent(r.id);
        if (ws && !scanStreamActive) {
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
      scheduleMapPaint();
    } finally {
      sensorPollInFlight = false;
    }
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
}

let logBagEntries = [];
let selectedLogBagIndices = new Set();
let logBagFileChecked = new Map();
let logBagRobotOptions = [];

function flattenLogBagEntries(payload) {
  const out = [];
  const robots = Array.isArray(payload && payload.robots) ? payload.robots : [];
  robots.forEach((robot) => {
    const bags = Array.isArray(robot.bags) ? robot.bags : [];
    bags.forEach((bag) => {
      out.push({
        ...bag,
        robotName: bag.robot_name || robot.robot_name || "",
        matchPath: robot.match_path || "",
      });
    });
  });
  return out;
}

function extractLogBagRobots(payload) {
  const robots = Array.isArray(payload && payload.robots) ? payload.robots : [];
  return robots
    .map((r) => String((r && r.robot_name) || "").trim())
    .filter(Boolean);
}

async function loadLogBagRobotOptions() {
  const names = new Set();
  try {
    const cache = await fetchJson(`${API_BASE_URL}/api/robot/status/cache`);
    const items = Array.isArray(cache && cache.items) ? cache.items : [];
    items.forEach((it) => {
      const id = String((it && (it.id || it.robot_id || it.name)) || "").trim();
      if (id) names.add(id);
    });
  } catch (e) {
    // Robot cache can be unavailable while backend starts; match.json still works.
  }
  try {
    if (webBootstrapData && webBootstrapData.log_bag) {
      extractLogBagRobots(webBootstrapData.log_bag).forEach((name) => names.add(name));
    } else {
      const res = await fetch(`${API_BASE_URL}/api/log_bag/matches`, { cache: "no-store" });
      const payload = await res.json();
      if (res.ok) {
        extractLogBagRobots(payload).forEach((name) => names.add(name));
      }
    }
  } catch (e) {
    // ignore; the selected robot fetch below will surface any real error
  }
  logBagRobotOptions = Array.from(names).sort();
}

function renderLogBagRobotSelect() {
  if (!logBagRobotSelect) return;
  const previous = logBagRobotSelect.value;
  logBagRobotSelect.innerHTML = "";
  if (logBagRobotOptions.length === 0) {
    const opt = document.createElement("option");
    opt.value = "";
    opt.textContent = "暂无机器人";
    logBagRobotSelect.appendChild(opt);
    logBagRobotSelect.disabled = true;
    return;
  }
  logBagRobotSelect.disabled = false;
  logBagRobotOptions.forEach((name) => {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name;
    logBagRobotSelect.appendChild(opt);
  });
  if (previous && logBagRobotOptions.includes(previous)) {
    logBagRobotSelect.value = previous;
  }
}

function formatLogBagSize(bytes) {
  const n = Number(bytes || 0);
  if (!Number.isFinite(n) || n <= 0) return "未知大小";
  if (n >= 1024 * 1024) return `${(n / 1024 / 1024).toFixed(1)} MB`;
  if (n >= 1024) return `${(n / 1024).toFixed(1)} KB`;
  return `${n} B`;
}

function basenameOfLogPath(path) {
  const text = String(path || "");
  const slash = text.lastIndexOf("/");
  return slash >= 0 ? text.slice(slash + 1) : text;
}

function toggleLogBagSelection(idx, checked) {
  if (checked) {
    selectedLogBagIndices.add(idx);
  } else {
    selectedLogBagIndices.delete(idx);
  }
  renderLogBagList();
  renderLogBagFiles();
}

function renderLogBagList() {
  if (!logBagList) return;
  logBagList.innerHTML = "";
  if (logBagEntries.length === 0) {
    const li = document.createElement("li");
    li.className = "log-bag-empty";
    li.textContent = logBagRobotSelect && logBagRobotSelect.value
      ? "无 log"
      : "请先选择机器人";
    logBagList.appendChild(li);
    renderLogBagFiles();
    return;
  }
  logBagEntries.forEach((entry, idx) => {
    const li = document.createElement("li");
    const selected = selectedLogBagIndices.has(idx);
    li.className = `log-bag-item${selected ? " log-bag-item--selected" : ""}`;
    li.dataset.index = String(idx);

    const checkboxId = `log-bag-select-${idx}`;
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.id = checkboxId;
    checkbox.className = "log-bag-item__checkbox";
    checkbox.checked = selected;
    checkbox.addEventListener("change", () => {
      toggleLogBagSelection(idx, checkbox.checked);
    });
    li.appendChild(checkbox);

    const body = document.createElement("label");
    body.className = "log-bag-item__body";
    body.htmlFor = checkboxId;

    const title = document.createElement("div");
    title.className = "log-bag-item__title";
    title.textContent = basenameOfLogPath(entry.bag) || "(unknown bag)";
    body.appendChild(title);

    const meta = document.createElement("div");
    meta.className = "log-bag-item__meta";
    meta.textContent = `${entry.ended_at || entry.started_at || "无时间"} · ${formatLogBagSize(entry.bytes)}`;
    body.appendChild(meta);

    const tags = Array.isArray(entry.tags) ? entry.tags : [];
    const tagRow = document.createElement("div");
    tagRow.className = "log-bag-tags";
    if (tags.length === 0) {
      const empty = document.createElement("span");
      empty.className = "log-bag-tag log-bag-tag--empty";
      empty.textContent = "no tag";
      tagRow.appendChild(empty);
    } else {
      tags.forEach((tag) => {
        const chip = document.createElement("span");
        chip.className = "log-bag-tag";
        chip.textContent = tag;
        tagRow.appendChild(chip);
      });
    }
    body.appendChild(tagRow);
    li.appendChild(body);
    logBagList.appendChild(li);
  });
}

function selectedLogBagFiles() {
  const files = [];
  const seen = new Set();
  Array.from(selectedLogBagIndices)
    .sort((a, b) => a - b)
    .forEach((idx) => {
      const entry = logBagEntries[idx];
      if (!entry || !Array.isArray(entry.files)) return;
      entry.files.forEach((file) => {
        if (!file || !file.path || seen.has(file.path)) return;
        seen.add(file.path);
        files.push({
          ...file,
          bagTitle: basenameOfLogPath(entry.bag) || entry.bag || "bag",
        });
      });
    });
  return files;
}

function defaultLogBagFileChecked(file) {
  return file.kind === "bag";
}

function updateLogBagDownloadState() {
  if (!btnDownloadLogBag || !logBagSelectionSummary) return;
  const checked = Array.from(
    document.querySelectorAll(".log-bag-file-list input[type='checkbox']:checked")
  ).map((el) => el.value);
  btnDownloadLogBag.disabled = checked.length === 0;
  const bagCount = selectedLogBagIndices.size;
  logBagSelectionSummary.textContent =
    checked.length === 0
      ? bagCount > 0
        ? "未勾选下载文件"
        : "未选择 bag"
      : `${bagCount} 个 bag · ${checked.length} 个文件`;
}

function renderLogBagFiles() {
  if (!logBagFileList || !logBagFileHint) return;
  logBagFileList.innerHTML = "";
  const files = selectedLogBagFiles();
  if (selectedLogBagIndices.size === 0) {
    logBagFileHint.hidden = false;
    logBagFileHint.textContent = "请先在左侧勾选一个或多个 bag。";
    updateLogBagDownloadState();
    return;
  }
  if (files.length === 0) {
    logBagFileHint.hidden = false;
    logBagFileHint.textContent = "所选 bag 没有关联文件。";
    updateLogBagDownloadState();
    return;
  }
  logBagFileHint.hidden = true;

  let groupTitle = "";
  files.forEach((file, idx) => {
    if (file.bagTitle !== groupTitle) {
      groupTitle = file.bagTitle;
      const heading = document.createElement("div");
      heading.className = "log-bag-file-group";
      heading.textContent = groupTitle;
      logBagFileList.appendChild(heading);
    }

    const id = `log-bag-file-${idx}`;
    const row = document.createElement("label");
    row.className = `log-bag-file${file.exists ? "" : " log-bag-file--missing"}`;
    row.htmlFor = id;

    const input = document.createElement("input");
    input.type = "checkbox";
    input.id = id;
    input.value = file.path;
    const checked = logBagFileChecked.has(file.path)
      ? logBagFileChecked.get(file.path)
      : defaultLogBagFileChecked(file);
    if (!logBagFileChecked.has(file.path)) {
      logBagFileChecked.set(file.path, checked);
    }
    input.checked = checked;
    input.disabled = !file.exists;
    input.addEventListener("change", () => {
      logBagFileChecked.set(file.path, input.checked);
      updateLogBagDownloadState();
    });
    row.appendChild(input);

    const body = document.createElement("span");
    body.className = "log-bag-file__body";
    const name = document.createElement("span");
    name.className = "log-bag-file__name";
    name.textContent = basenameOfLogPath(file.path);
    const meta = document.createElement("span");
    meta.className = "log-bag-file__meta";
    meta.textContent = `${file.kind || "file"}${file.is_dir ? " · directory" : ""}${file.exists ? "" : " · missing"}`;
    body.appendChild(name);
    body.appendChild(meta);
    row.appendChild(body);
    logBagFileList.appendChild(row);
  });
  updateLogBagDownloadState();
}

async function refreshLogBags() {
  if (!logBagStatus) return;
  logBagStatus.textContent = "正在读取机器人列表...";
  try {
    await loadLogBagRobotOptions();
    renderLogBagRobotSelect();
    const robotName = logBagRobotSelect ? String(logBagRobotSelect.value || "").trim() : "";
    if (!robotName) {
      logBagEntries = [];
      selectedLogBagIndices = new Set();
      logBagFileChecked = new Map();
      renderLogBagList();
      renderLogBagFiles();
      logBagStatus.textContent = "无机器人，暂无日志索引";
      return;
    }
    logBagStatus.textContent = `正在读取 ${robotName} 的 match.json...`;
    const url = `${API_BASE_URL}/api/log_bag/matches?robot_name=${encodeURIComponent(robotName)}`;
    const res = await fetch(url, { cache: "no-store" });
    const payload = await res.json();
    if (!res.ok) throw new Error(payload.error || "读取日志索引失败");
    logBagEntries = flattenLogBagEntries(payload);
    logBagFileChecked = new Map();
    selectedLogBagIndices = logBagEntries.length > 0 ? new Set([0]) : new Set();
    renderLogBagList();
    renderLogBagFiles();
    logBagStatus.textContent =
      logBagEntries.length > 0 ? `${robotName}：已加载 ${logBagEntries.length} 个 bag` : `${robotName}：无 log`;
  } catch (err) {
    logBagEntries = [];
    selectedLogBagIndices = new Set();
    logBagFileChecked = new Map();
    renderLogBagList();
    renderLogBagFiles();
    logBagStatus.textContent = `读取失败：${err.message || err}`;
  }
}

async function downloadSelectedLogBagFiles() {
  const checked = Array.from(
    document.querySelectorAll(".log-bag-file-list input[type='checkbox']:checked")
  ).map((el) => el.value);
  if (checked.length === 0) return;
  if (btnDownloadLogBag) btnDownloadLogBag.disabled = true;
  if (logBagStatus) logBagStatus.textContent = "正在打包下载...";
  try {
    const res = await fetch(`${API_BASE_URL}/api/log_bag/download`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ files: checked }),
    });
    if (!res.ok) {
      let message = "下载失败";
      try {
        const payload = await res.json();
        message = payload.error || message;
      } catch (e) {
        // ignore non-json error bodies
      }
      throw new Error(message);
    }
    const blob = await res.blob();
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `openDelivery_logs_${Date.now()}.zip`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
    if (logBagStatus) logBagStatus.textContent = "下载已开始";
  } catch (err) {
    if (logBagStatus) logBagStatus.textContent = `下载失败：${err.message || err}`;
  } finally {
    updateLogBagDownloadState();
  }
}

function initLogs() {
  if (logBagRobotSelect) {
    logBagRobotSelect.addEventListener("change", refreshLogBags);
  }
  if (btnRefreshLogBags) {
    btnRefreshLogBags.addEventListener("click", refreshLogBags);
  }
  if (btnDownloadLogBag) {
    btnDownloadLogBag.addEventListener("click", downloadSelectedLogBagFiles);
  }
  refreshLogBags();
}

function syncMapEditorControls() {
  if (!mapEditorLayer || !mapEditorTool) return;
  const layer = mapEditorLayer.value;
  const choices = layer === "points" ? [["add", "新增点位"], ["move", "移动点位"], ["delete", "删除点位"]] : [["paint", "画笔"], ["erase", "擦除"]];
  mapEditorTool.innerHTML = choices.map(([value, label]) => `<option value="${value}">${label}</option>`).join("");
  if (mapEditorBrush) mapEditorBrush.disabled = layer === "points";
  if (mapEditorRasterValue) mapEditorRasterValue.disabled = layer !== "raster";
  if (mapEditorSemanticLabel) mapEditorSemanticLabel.disabled = layer !== "semantic";
  renderScene();
}

async function initMonitor() {
  loadAndApplyMonitorCheckboxPrefs();
  syncMapEditorControls();
  if (mapEditorLayer) mapEditorLayer.addEventListener("change", syncMapEditorControls);
  if (mapEditorSemanticLabel) mapEditorSemanticLabel.addEventListener("change", updateSemanticLabelPreview);

  bindMapInteractions();

  if (btnResetView) {
    btnResetView.addEventListener("click", () => {
      resetViewToFit();
      renderScene();
      refreshMetaPanel();
      appendLog("地图视图已重置");
    });
  }

  if (btnMapEditorOpen) {
    btnMapEditorOpen.addEventListener("click", () => {
      if (!window.StandaloneMapEditor) {
        if (mapStatus) mapStatus.textContent = "地图编辑器加载失败，请刷新页面后重试";
        return;
      }
      window.StandaloneMapEditor.open(activeFloor);
    });
  }
  if (btnMapEditorClose) {
    btnMapEditorClose.addEventListener("click", () => {
      setMapEditorDialogOpen(false);
      if (mapEditorDirty && mapEditorMessage) mapEditorMessage.textContent = "修改仍保留，重新打开后可继续或保存";
    });
  }
  if (btnMapEditorUndo) btnMapEditorUndo.addEventListener("click", undoMapEditorChange);
  if (btnMapEditorCancel) btnMapEditorCancel.addEventListener("click", discardMapEditorChanges);
  document.addEventListener("keydown", (ev) => {
    const target = ev.target;
    const isTyping = target && (target.tagName === "INPUT" || target.tagName === "TEXTAREA" || target.isContentEditable);
    if ((ev.ctrlKey || ev.metaKey) && !ev.shiftKey && String(ev.key).toLowerCase() === "z" && mapEditorDialog && !mapEditorDialog.hidden && !isTyping) {
      ev.preventDefault();
      undoMapEditorChange();
      return;
    }
    if (ev.key === "Escape" && mapEditorDialog && !mapEditorDialog.hidden) setMapEditorDialogOpen(false);
  });

  if (btnWaypointPick) {
    btnWaypointPick.addEventListener("click", () => {
      if (!activePgm || isMappingFloor(activeFloor)) { waypointMessage.textContent = "请先选择已保存地图"; return; }
      mapEditorResumeAfterPick = mapEditorActive;
      if (mapEditorActive) setMapEditorActive(false);
      relocPickAction = "waypoint";
      resetRelocPickState("单击地图设置点位位置，再单击设置朝向");
      relocPickToggle.checked = true;
      syncRelocPickCursorClass();
      waypointMessage.textContent = "请在地图上依次设置位置和朝向";
    });
  }
  function recordCurrentMapPoint() {
    const name = String(waypointName.value || "").trim();
    const xText = String(waypointX.value || "").trim();
    const yText = String(waypointY.value || "").trim();
    const x = Number(xText), y = Number(yText), yaw = Number(waypointYaw.value || 0);
    if (!name || !xText || !yText || !Number.isFinite(x) || !Number.isFinite(y)) throw new Error("请填写名称并在地图选点或填写 X/Y");
    pushMapEditorUndoSnapshot("points");
    mapPoints.push({ id: nextMapPointId(name), name, type: waypointType.value, x, y, yaw: Number.isFinite(yaw) ? yaw : 0 });
    markMapEditorDirty("points");
    renderMapWaypointList();
    renderScene();
    waypointName.value = "";
    waypointMessage.textContent = "点位已加入，点击保存地图修改后落盘";
  }
  if (btnWaypointSave) btnWaypointSave.addEventListener("click", () => {
    try { recordCurrentMapPoint(); } catch (err) { waypointMessage.textContent = err.message || err; }
  });
  if (mapWaypointList) mapWaypointList.addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-delete-map-point]"); if (!btn) return;
    pushMapEditorUndoSnapshot("points");
    mapPoints = mapPoints.filter((point) => point.id !== btn.dataset.deleteMapPoint);
    markMapEditorDirty("points");
    renderMapWaypointList();
    renderScene();
    waypointMessage.textContent = "点位已移除，点击保存地图修改后落盘";
  });
  if (btnMapEditorToggle) btnMapEditorToggle.addEventListener("click", () => {
    if (!activePgm || isMappingFloor(activeFloor)) { mapEditorMessage.textContent = "请先选择已保存地图"; return; }
    setMapEditorActive(!mapEditorActive);
    mapEditorMessage.textContent = mapEditorActive ? "编辑已开启；Shift+拖拽仍可平移。" : (mapEditorDirty ? "编辑已暂停，有未保存修改" : "编辑已暂停");
  });
  if (btnMapEditorSave) btnMapEditorSave.addEventListener("click", async () => {
    btnMapEditorSave.disabled = true; mapEditorMessage.textContent = "保存中…";
    try { await saveActiveMapEditor(); mapEditorMessage.textContent = "地图修改已保存"; }
    catch (err) { mapEditorMessage.textContent = "保存失败：" + (err.message || err); }
    finally { syncMapEditorUi(); }
  });

  if (btnRelocFillPose) {
    btnRelocFillPose.addEventListener("click", () => {
      fillRelocPoseFromScreenRobot();
    });
  }

  syncRelocPickCursorClass();
  if (relocPickToggle) {
    relocPickToggle.addEventListener("change", () => {
      syncRelocPickCursorClass();
      if (relocPickToggle.checked) {
        relocPickAction = "relocalize";
        resetRelocPickState("单击地图定位置，再单击定朝向");
      } else {
        relocPickAction = "relocalize";
        resetRelocPickState("");
        if (mapEditorResumeAfterPick) {
          mapEditorResumeAfterPick = false;
          setMapEditorActive(true);
        }
      }
      saveMonitorCheckboxPrefs();
    });
  }
  if (btnRelocPickGoal) {
    btnRelocPickGoal.addEventListener("click", () => {
      const rid = relocRobotId && relocRobotId.value ? relocRobotId.value.trim() : "";
      if (!rid) {
        if (relocMessage) {
          relocMessage.textContent = "请先填写机器人 ID";
        }
        return;
      }
      if (!activePgm || !activeMeta || !relocPickToggle) {
        if (relocMessage) {
          relocMessage.textContent = "请先选择并加载地图";
        }
        return;
      }
      relocPickAction = "goto";
      resetRelocPickState("");
      relocPickToggle.checked = true;
      syncRelocPickCursorClass();
      if (relocMessage) {
        relocMessage.textContent = `为 ${rid} 选导航目标：单击地图定位置，再单击定朝向`;
      }
    });
  }
  if (btnRelocSkipHeading) {
    btnRelocSkipHeading.addEventListener("click", () => {
      if (relocPickStep !== 1) {
        return;
      }
      const yaw = parseFloat((relocYaw && relocYaw.value) || "0");
      finishRelocMapPick(Number.isFinite(yaw) ? yaw : 0);
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
    /** Aligns with ``LocalizeNavCommand.msg`` / POST ``type: localize_nav_command`` (task_manager → 切图/重定位). */
    const payload = {
      type: "localize_nav_command",
      robot_id: rid,
      map_name: "",
      set_initial_pose: mode === "pose_only" || mode === "both",
      x: 0,
      y: 0,
      yaw: 0,
    };
    if (mode === "map_only" || mode === "both") {
      const mn = targetMapName();
      if (!mn) {
        relocMessage.textContent =
          isMappingFloor(floorSelect && floorSelect.value)
            ? "建图模式下请填写「保存为」地图名（切图经 task_manager 准备后经 robot_status 下发）"
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
      appendLog(`下发 localize_nav_command (${mode}) → ${rid}`);
    } catch (err) {
      relocMessage.textContent = err.message || String(err);
    }
  }

  async function recordRelocalizationPoint() {
    if (!relocMessage) return;
    const rid = relocRobotId && relocRobotId.value ? relocRobotId.value.trim() : "";
    if (!rid) {
      relocMessage.textContent = "请先选择在线机器人";
      return;
    }
    btnRelocRecord.disabled = true;
    relocMessage.textContent = "正在记录当前位姿和激光帧…";
    try {
      const out = await fetchJson(`${API_BASE_URL}/api/robot/relocalization/record`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ robot_id: rid }),
      });
      const point = out && out.point;
      if (point && out.map_name === activeFloor) {
        mapPoints = mapPoints.filter((row) => row.id !== point.id).concat([point]);
        savedMapPoints = savedMapPoints.filter((row) => row.id !== point.id).concat([point]);
        renderMapWaypointList();
        renderScene();
      }
      relocMessage.textContent = `已记录重定位点 ${point && (point.name || point.id) ? point.name || point.id : ""}`.trim();
      appendLog(`记录重定位点 → ${rid} / ${out.map_name || "当前地图"}`);
    } catch (err) {
      relocMessage.textContent = `记录失败：${err.message || String(err)}`;
    } finally {
      btnRelocRecord.disabled = false;
    }
  }

  if (btnRelocRecord) {
    btnRelocRecord.addEventListener("click", recordRelocalizationPoint);
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
      scheduleResizeCanvasToDisplay();
    });
    ro.observe(mapWrapper);
  } else {
    window.addEventListener("resize", scheduleResizeCanvasToDisplay);
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
      mapStatus.textContent = "正在调用 map_saver_cli 保存…";
      try {
        const mappingRid = isMappingFloor(activeFloor) ? robotIdFromMappingFloor(activeFloor) : "";
        const payload = await postSaveMap(name, mappingRid);
        saveMapNamePreference(name);
        const dirLine =
          payload && payload.map_dir ? ` → ${payload.map_dir}` : ` → map/${name}/`;
        mapStatus.textContent = `已保存${dirLine}`;
        appendLog(
          payload && payload.map_topic
            ? `地图已保存: ${name} · ${payload.map_topic}`
            : `地图已保存: ${name}`
        );
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
    if (mapEditorDirty && !window.confirm("当前地图有未保存修改，确定切换地图吗？")) { e.target.value = activeFloor || ""; return; }
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
  if (semanticMapToggle) semanticMapToggle.addEventListener("change", () => { saveMonitorCheckboxPrefs(); renderScene(); });
  if (customPointsToggle) customPointsToggle.addEventListener("change", () => { saveMonitorCheckboxPrefs(); renderScene(); });
  initMonitorTeleop();
  startSensorPolling();
}

let teleopHeldAction = "";
let teleopHeldRobotId = "";
let teleopRequestSeq = 0;
let teleopHeartbeatTimer = null;
const teleopSessionId = (window.crypto && crypto.randomUUID ? crypto.randomUUID() : `${Date.now()}_${Math.random()}`).replace(/[^A-Za-z0-9_-]/g, "_");
function teleopVector(action) {
  const lin = Math.min(1.2, Math.max(0.02, Number(teleopLinear && teleopLinear.value) || 0.2));
  const ang = Math.min(1.5, Math.max(0.05, Number(teleopAngular && teleopAngular.value) || 0.5));
  if (action === "forward") return [lin, 0];
  if (action === "backward") return [-lin, 0];
  if (action === "left") return [0, ang];
  if (action === "right") return [0, -ang];
  return [0, 0];
}
async function postTeleop(action, active, robotId = "") {
  const rid = robotId || (relocRobotId && relocRobotId.value);
  if (!rid) throw new Error("没有在线机器人可遥控");
  const [linear, angular] = teleopVector(action);
  const seq = ++teleopRequestSeq;
  const res = await fetch(`${API_BASE_URL}/api/robot/motion/teleop`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ robot_id: rid, linear, angular, active, confirmed: true, session_id: teleopSessionId, sequence: seq }) });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || `遥控失败 (${res.status})`);
  if (seq === teleopRequestSeq && teleopMessage) teleopMessage.textContent = active ? `${rid}：${action}` : `${rid}：已停车`;
}
function setTeleopHeld(action) {
  if (!action || action === "stop") { releaseTeleop(true); return; }
  const rid = relocRobotId && relocRobotId.value;
  if (!rid) { if (teleopMessage) teleopMessage.textContent = "没有在线机器人可遥控"; return; }
  if (teleopHeldAction === action && teleopHeldRobotId === rid) return;
  if (teleopHeldAction && teleopHeldRobotId && teleopHeldRobotId !== rid) releaseTeleop();
  teleopHeldAction = action;
  teleopHeldRobotId = rid;
  if (teleopHeartbeatTimer) clearInterval(teleopHeartbeatTimer);
  document.querySelectorAll("[data-teleop]").forEach((button) => button.classList.toggle("is-active", button.dataset.teleop === action));
  postTeleop(action, true, rid).catch((err) => { if (teleopMessage) teleopMessage.textContent = err.message || err; });
  teleopHeartbeatTimer = setInterval(() => { if (teleopHeldAction === action && teleopHeldRobotId === rid) postTeleop(action, true, rid).catch(() => {}); }, 250);
}
function releaseTeleop(force = false) {
  const old = teleopHeldAction; const oldRobotId = teleopHeldRobotId; const targetRobotId = oldRobotId || (force && relocRobotId ? relocRobotId.value : "");
  teleopHeldAction = ""; teleopHeldRobotId = "";
  if (teleopHeartbeatTimer) { clearInterval(teleopHeartbeatTimer); teleopHeartbeatTimer = null; }
  document.querySelectorAll("[data-teleop]").forEach((button) => button.classList.remove("is-active"));
  if ((old || force) && targetRobotId) postTeleop(old || "stop", false, targetRobotId).catch((err) => { if (teleopMessage) teleopMessage.textContent = err.message || err; });
}
function initMonitorTeleop() {
  document.querySelectorAll("button[data-teleop]").forEach((button) => {
    button.addEventListener("pointerdown", (ev) => { ev.preventDefault(); button.setPointerCapture?.(ev.pointerId); setTeleopHeld(button.dataset.teleop); });
    ["pointerup", "pointercancel", "lostpointercapture"].forEach((name) => button.addEventListener(name, releaseTeleop));
  });
  const keys = { KeyW: "forward", ArrowUp: "forward", KeyS: "backward", ArrowDown: "backward", KeyA: "left", ArrowLeft: "left", KeyD: "right", ArrowRight: "right" };
  window.addEventListener("keydown", (ev) => { if (document.activeElement && /INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName)) return; if (keys[ev.code]) { ev.preventDefault(); setTeleopHeld(keys[ev.code]); } });
  window.addEventListener("keyup", (ev) => { if (keys[ev.code] && teleopHeldAction === keys[ev.code]) releaseTeleop(); });
  window.addEventListener("blur", releaseTeleop);
  document.addEventListener("visibilitychange", () => { if (document.hidden) releaseTeleop(); });
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
  const locationsPanel = document.getElementById("locations-quick-fill");
  const locationsBtnContainer = document.getElementById("locations-quick-fill-btns");

  async function loadLocations() {
    if (!locationsPanel || !locationsBtnContainer) return;
    try {
      const data = await fetchJson(`${API_BASE_URL}/api/locations`);
      const locs = (data && Array.isArray(data.locations)) ? data.locations : [];
      if (!locs.length) return;
      locationsBtnContainer.innerHTML = "";
      locs.forEach((loc) => {
        const btn = document.createElement("button");
        btn.type = "button";
        btn.className = "locations-quick-fill__btn";
        btn.textContent = String(loc.label || loc.id);
        btn.title = `X: ${loc.x}, Y: ${loc.y}, Yaw: ${loc.yaw}°`;
        btn.addEventListener("click", () => {
          if (gazeboX) gazeboX.value = String(loc.x);
          if (gazeboY) gazeboY.value = String(loc.y);
          if (gazeboYaw) gazeboYaw.value = String(loc.yaw);
          if (gazeboMessage) gazeboMessage.textContent = `已填入：${loc.label || loc.id}`;
        });
        locationsBtnContainer.appendChild(btn);
      });
      locationsPanel.style.display = "";
    } catch {
      /* locations.json unavailable — hide the panel silently */
    }
  }
  loadLocations();
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
  let topCameraPollTimer = null;
  let topCameraTelemetryTimer = null;
  let topCameraRefreshInFlight = false;
  let topCameraImageSkip = 0;
  let camZoomDir = 0;
  /** @type {Record<string, unknown> | null} */
  let lastTopCameraStatusSnap = null;

  const gazeboCameraWrap = document.getElementById("gazebo-camera-wrap");
  const elTopCamMain = document.getElementById("gazebo-top-camera-line-main");
  const elTopCamStamp = document.getElementById("gazebo-top-camera-line-stamp");
  const elTopCamHint = document.getElementById("gazebo-top-camera-line-hint");

  const CAMERA_Z_MIN = 6.0;
  const CAMERA_Z_MAX = 80.0;
  const CAMERA_FRAME_SIZE = 20.0;
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

  function fitTopCameraCanvas() {
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
  }

  function applyGazeboCameraWrapTier(tier) {
    if (!gazeboCameraWrap) {
      return;
    }
    gazeboCameraWrap.classList.remove(
      "gazebo-camera-wrap--fresh",
      "gazebo-camera-wrap--aging",
      "gazebo-camera-wrap--stale",
      "gazebo-camera-wrap--offline"
    );
    if (tier < 0) {
      gazeboCameraWrap.classList.add("gazebo-camera-wrap--offline");
      return;
    }
    if (tier === 0) {
      gazeboCameraWrap.classList.add("gazebo-camera-wrap--fresh");
    } else if (tier === 1) {
      gazeboCameraWrap.classList.add("gazebo-camera-wrap--aging");
    } else {
      gazeboCameraWrap.classList.add("gazebo-camera-wrap--stale");
    }
  }

  function paintTopCameraOverlay(ageSec, tier, st) {
    if (!elTopCamMain || !elTopCamStamp || !elTopCamHint) {
      return;
    }
    if (!st || st.available === false) {
      elTopCamMain.className =
        "gazebo-top-camera-overlay__main gazebo-top-camera-overlay__main--stale";
      elTopCamMain.textContent = "未连接 · 无 topdown 图像";
      elTopCamStamp.textContent = "桥未缓存帧（检查 Gazebo、ros_tf_bridge 与图像 topic）";
      elTopCamHint.textContent = "";
      return;
    }
    const tierCls = tier === 0 ? "fresh" : tier === 1 ? "aging" : "stale";
    elTopCamMain.className = `gazebo-top-camera-overlay__main gazebo-top-camera-overlay__main--${tierCls}`;
    let ageLabel =
      ageSec < 120
        ? `${ageSec.toFixed(1)}s`
        : `${Math.floor(ageSec / 60)}m${Math.floor(ageSec % 60)}s`;
    let head = `距上次收到帧: ${ageLabel}`;
    if (tier === 2) {
      head += " · 历史画面 / 可能已离线 (≥30s)";
    } else if (tier === 1) {
      head += " · 帧偏旧 (≥10s)";
    } else {
      head += " · 新鲜 (<10s)";
    }
    elTopCamMain.textContent = head;
    const sec = st.stamp_sec != null ? Number(st.stamp_sec) : null;
    const nsec = st.stamp_nanosec != null ? Number(st.stamp_nanosec) : null;
    const wh =
      st.width && st.height
        ? `${Number(st.width)}×${Number(st.height)}`
        : cameraFrame.width && cameraFrame.height
          ? `${cameraFrame.width}×${cameraFrame.height}`
          : "—";
    const fid = st.frame_id ? String(st.frame_id) : "";
    if (sec != null && Number.isFinite(sec) && nsec != null && Number.isFinite(nsec)) {
      const nsPad = String(Math.max(0, Math.floor(nsec))).padStart(9, "0");
      elTopCamStamp.textContent = `ROS 时间戳: ${sec}.${nsPad} · ${wh}${fid ? ` · ${fid}` : ""}`;
    } else {
      elTopCamStamp.textContent = `分辨率: ${wh}${fid ? ` · ${fid}` : ""}`;
    }
    if (tier === 2) {
      elTopCamHint.textContent = "超过 30s 未更新：仍显示最后一帧，取点可能不准；恢复 topic 后将自动回到高频刷新。";
    } else if (tier === 1) {
      elTopCamHint.textContent = "超过 10s 未更新：请确认仿真与相机插件仍在发布。";
    } else {
      elTopCamHint.textContent = "";
    }
  }

  function tickTopCameraOverlayClock() {
    if (!gazeboViewActive()) {
      return;
    }
    if (!lastTopCameraStatusSnap || lastTopCameraStatusSnap.available === false) {
      return;
    }
    const recv = Number(lastTopCameraStatusSnap.received_at || 0);
    if (!recv) {
      return;
    }
    const age = Math.max(0, Date.now() / 1000 - recv);
    let tier = 0;
    if (age >= 30) {
      tier = 2;
    } else if (age >= 10) {
      tier = 1;
    }
    paintTopCameraOverlay(age, tier, lastTopCameraStatusSnap);
    applyGazeboCameraWrapTier(tier);
  }

  function updateTopCameraOverlayFromStatus(st) {
    lastTopCameraStatusSnap = st && typeof st === "object" ? { ...st } : null;
    if (!lastTopCameraStatusSnap || lastTopCameraStatusSnap.available === false) {
      paintTopCameraOverlay(0, -1, lastTopCameraStatusSnap);
      return;
    }
    const age = Number(lastTopCameraStatusSnap.age_received_sec ?? 0);
    const tier = Number(lastTopCameraStatusSnap.stale_tier ?? 0);
    paintTopCameraOverlay(Number.isFinite(age) ? age : 0, tier, lastTopCameraStatusSnap);
  }

  function clearTopCameraNoSignal() {
    lastTopCameraStatusSnap = { available: false };
    if (!topCameraCanvas || !topCameraCtx) {
      return;
    }
    fitTopCameraCanvas();
    const vw = topCameraCanvas.clientWidth || 320;
    const vh = topCameraCanvas.clientHeight || 240;
    topCameraCtx.fillStyle = "#0f172a";
    topCameraCtx.fillRect(0, 0, vw, vh);
    topCameraCtx.fillStyle = "#94a3b8";
    topCameraCtx.font = "13px sans-serif";
    topCameraCtx.fillText("无图像（桥未收到 topdown 相机 topic）", 14, 28);
    cameraFrame.width = 0;
    cameraFrame.height = 0;
    updateTopCameraOverlayFromStatus({ available: false });
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
        fitTopCameraCanvas();
        const vw = topCameraCanvas.clientWidth || width;
        const vh = topCameraCanvas.clientHeight || height;
        const off = document.createElement("canvas");
        off.width = width;
        off.height = height;
        const offCtx = off.getContext("2d");
        offCtx.putImageData(new ImageData(rgba, width, height), 0, 0);
        topCameraCtx.clearRect(0, 0, vw, vh);
        topCameraCtx.drawImage(off, 0, 0, vw, vh);
        lastTopCameraStatusSnap = {
          available: true,
          received_at: data.received_at,
          age_received_sec: data.age_received_sec,
          stale_tier: data.stale_tier,
          stamp_sec: data.stamp_sec,
          stamp_nanosec: data.stamp_nanosec,
          width: data.width,
          height: data.height,
          frame_id: data.frame_id,
        };
        tickTopCameraOverlayClock();
      } else if (!data || data.available === false) {
        clearTopCameraNoSignal();
      }
    } catch {
      /* ignore */
    } finally {
      topCameraRefreshInFlight = false;
    }
  }

  function stopTopCameraAdaptiveLoop() {
    if (topCameraPollTimer) {
      clearTimeout(topCameraPollTimer);
      topCameraPollTimer = null;
    }
  }

  async function runTopCameraPollCycle() {
    if (!gazeboViewActive()) {
      return;
    }
    let nextMs = 4500;
    try {
      const st = await fetchJson(`${API_BASE_URL}/api/gazebo/top_camera/status`);
      lastTopCameraStatusSnap = st && typeof st === "object" ? { ...st } : { available: false };
      updateTopCameraOverlayFromStatus(lastTopCameraStatusSnap);
      if (!st || st.available === false) {
        applyGazeboCameraWrapTier(-1);
        clearTopCameraNoSignal();
        topCameraImageSkip = 0;
        nextMs = 5000;
      } else {
        const tier = Number(st.stale_tier || 0);
        const age = Number(st.age_received_sec ?? 0);
        applyGazeboCameraWrapTier(tier);
        let fetchImg = false;
        if (tier === 0 && age < 2.5) {
          fetchImg = true;
          nextMs = 100;
        } else if (tier === 0) {
          fetchImg = topCameraImageSkip++ % 2 === 0;
          nextMs = 400;
        } else if (tier === 1) {
          fetchImg = topCameraImageSkip++ % 3 === 0;
          nextMs = 900;
        } else {
          fetchImg = topCameraImageSkip++ % 5 === 0;
          nextMs = 2200;
        }
        if (fetchImg) {
          await refreshTopCameraFrame();
        }
      }
    } catch {
      nextMs = 5000;
    }
    if (gazeboViewActive()) {
      topCameraPollTimer = window.setTimeout(runTopCameraPollCycle, nextMs);
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
    topCameraCanvas.addEventListener("click", handleTopCameraPick);
    let gazeboCameraLoopsActive = false;
    function startGazeboCameraLoops() {
      if (gazeboCameraLoopsActive) {
        return;
      }
      gazeboCameraLoopsActive = true;
      topCameraImageSkip = 0;
      fillTopdownCameraPoseFromGazebo().catch(() => {});
      fitTopCameraCanvas();
      stopTopCameraAdaptiveLoop();
      runTopCameraPollCycle();
      if (!topCameraTelemetryTimer) {
        topCameraTelemetryTimer = window.setInterval(tickTopCameraOverlayClock, 1000);
      }
      if (!cameraDriveTimer) {
        cameraDriveTimer = window.setInterval(() => {
          tickTopdownCameraDrive();
        }, 33);
      }
    }
    function stopGazeboCameraLoops() {
      if (!gazeboCameraLoopsActive) {
        return;
      }
      gazeboCameraLoopsActive = false;
      stopTopCameraAdaptiveLoop();
      if (topCameraTelemetryTimer) {
        clearInterval(topCameraTelemetryTimer);
        topCameraTelemetryTimer = null;
      }
      if (cameraDriveTimer) {
        clearInterval(cameraDriveTimer);
        cameraDriveTimer = null;
      }
      clearCamDriveKeys();
    }
    function syncGazeboCameraLoopsFromView() {
      if (gazeboViewActive()) {
        startGazeboCameraLoops();
      } else {
        stopGazeboCameraLoops();
      }
    }
    if (views.gazebo) {
      const mo = new MutationObserver(() => syncGazeboCameraLoopsFromView());
      mo.observe(views.gazebo, { attributes: true, attributeFilter: ["class"] });
    }
    syncGazeboCameraLoopsFromView();
    window.addEventListener("resize", () => {
      if (gazeboViewActive()) {
        fitTopCameraCanvas();
      }
    });
  }
}

async function bootstrap() {
  await resolveApiBaseUrl();
  // Standalone tools consume only this connection setting, never monitor-page map state.
  window.OPEN_DELIVERY_API_BASE_URL = API_BASE_URL;
  try {
    await fetchWebBootstrap();
  } catch (err) {
    webBootstrapData = null;
    if (mapStatus) {
      mapStatus.textContent = `后端初始化失败: ${err.message || err}`;
    }
  }
  initSettings();
  initLogs();
  loadSimBringupPhasesFromSession();
  initRosNodesPage();
  initRobotPresenceUi();
  initRobotDetailUi();
  updateRobotPresenceTriggerSummary();
  await initMonitor();
  initGazeboPage();
}

bootstrap();

function detailStatusValue(status, liveKey, persistedKey) {
  return String((status && (status[liveKey] || status[persistedKey])) || "—");
}

function robotDetailSettingsKey(rid) {
  return `${SETTINGS_KEY}:${rid}`;
}

function loadRobotDetailSettings(rid) {
  const defaults = { maxSpeed: 1.2, angularSpeed: 0.8, safetyDistance: 0.6, refreshInterval: 500 };
  try {
    return { ...defaults, ...(JSON.parse(localStorage.getItem(robotDetailSettingsKey(rid)) || "null") || {}) };
  } catch {
    return defaults;
  }
}

function renderRobotQuickDock() {
  if (!robotQuickDock) return;
  const online = mergePresenceRows().filter((row) => row.online);
  const nextMarkup = online
    .map(
      (row) => `<button type="button" class="robot-quick-chip${
        selectedDetailRobotId === row.id && robotDetailPanel && !robotDetailPanel.hidden ? " active" : ""
      }" data-robot-quick-id="${escapeHtml(row.id)}" title="查看 ${escapeHtml(row.name || row.id)} 详情">
        <span class="robot-quick-chip__avatar"><img src="./icons/robot.svg" alt="" /></span>
        <span class="robot-quick-chip__name">${escapeHtml(row.name || row.id)}</span>
        <span class="robot-quick-chip__dot" aria-label="在线"></span>
      </button>`
    )
    .join("");
  if (nextMarkup !== robotQuickDockMarkup) {
    robotQuickDock.innerHTML = nextMarkup;
    robotQuickDockMarkup = nextMarkup;
  }
}

function robotNodeByHint(nodes, hints) {
  return (nodes || []).find((node) => hints.some((hint) => String(node.name || "").includes(hint)));
}

function renderRobotBehaviorTree(nodes) {
  const steps = [
    ["接收导航目标", ["/bt_navigator"]],
    ["行为树调度", ["/bt_navigator"]],
    ["全局路径规划", ["/planner_server"]],
    ["局部轨迹控制", ["/controller_server"]],
    ["恢复行为", ["/recoveries_server"]],
    ["到达目标 / 任务反馈", ["/waypoint_follower", "/bt_navigator"]],
  ];
  return `<div class="robot-bt-tree">${steps
    .map(([label, hints]) => {
      const node = robotNodeByHint(nodes, hints);
      const lifecycle = node && node.lifecycle ? String(node.lifecycle.state || "unknown") : "missing";
      const running = !!(node && node.running);
      return `<div class="robot-bt-node ${running ? "running" : "missing"}">
        <strong>${escapeHtml(label)}</strong>
        <span>${node ? `${escapeHtml(node.name)} · ${escapeHtml(lifecycle)}` : "对应节点未发现"}</span>
      </div>`;
    })
    .join("")}</div>`;
}

function renderRobotDetailOverview(payload) {
  const status = payload.status || {};
  const nodes = Array.isArray(payload.nodes) ? payload.nodes : [];
  const poseRow =
    payload.pose ||
    ((latestSnapshot && Array.isArray(latestSnapshot.robots)
      ? latestSnapshot.robots.find((r) => String(r.id) === payload.robot_id)
      : null) || {});
  const pose = poseRow.pose || {};
  const sensors = payload.sensors || {};
  const scan = sensors.scan_2d || {};
  const plannedPath = sensors.planned_path || {};
  const robotModel = status.robot_model || poseRow.robot_model || "—";
  const semanticPosition = status.current_position || poseRow.current_position || "unknown;";
  const localization = poseRow.localization || (status.online ? "unknown" : "lost");
  const isSimulation = Boolean(
    status.live_is_simulation || status.persisted_is_simulation || poseRow.is_simulation
  );
  return `<div class="robot-detail-grid">
    <section class="robot-detail-card"><h3>机器人信息</h3>
      <p>ID：<code>${escapeHtml(payload.robot_id)}</code></p>
      <p>机型：<code>${escapeHtml(robotModel)}</code></p>
      <p>仿真：<code>${isSimulation ? "是" : "否"}</code></p>
      <p>心跳：<code>${status.online ? "正常" : "离线"}</code></p>
    </section>
    <section class="robot-detail-card"><h3>运行状态</h3>
      <p>在线：<code>${status.online ? "是" : "否"}</code></p>
      <p>robot_status：<code>${escapeHtml(detailStatusValue(status, "live_robot_status", "persisted_robot_status"))}</code></p>
      <p>定位：<code>${escapeHtml(localization)}</code></p>
      <p>task_status：<code>${escapeHtml(detailStatusValue(status, "live_task_status", "persisted_task_status"))}</code></p>
      <p>任务进度：<code>${Number(status.task_progress) >= 0 ? `${Math.round(Number(status.task_progress) * 100)}%` : "—"}</code></p>
    </section>
    <section class="robot-detail-card"><h3>位置</h3>
      <p>地图：<code>${escapeHtml(status.floor || status.persisted_current_map || "—")}</code></p>
      <p>语义位置：<code>${escapeHtml(semanticPosition)}</code></p>
      <p>X：<code>${Number.isFinite(Number(pose.x)) ? Number(pose.x).toFixed(3) : "—"}</code></p>
      <p>Y：<code>${Number.isFinite(Number(pose.y)) ? Number(pose.y).toFixed(3) : "—"}</code></p>
      <p>Yaw：<code>${Number.isFinite(Number(pose.yaw)) ? Number(pose.yaw).toFixed(3) : "—"}</code></p>
    </section>
    <section class="robot-detail-card"><h3>可视化数据</h3>
      <p>scan_2d：<code>${scan.available ? `${scan.point_count || 0} 点` : "无数据"}</code></p>
      <p>scan 坐标：<code>${escapeHtml(scan.coordinates || "—")} / ${escapeHtml(scan.frame_id || "—")}</code></p>
      <p>规划轨迹：<code>${plannedPath.available ? `${plannedPath.point_count || 0} 点` : "无数据"}</code></p>
    </section>
    <section class="robot-detail-card"><h3>ROS 节点</h3><p><code>${nodes.length}</code> 个该机器人节点</p></section>
    <section class="robot-detail-card"><h3>资源</h3><p><code>${(payload.processes || []).length}</code> 个关联进程</p></section>
  </div>`;
}

function renderRobotDetailTasks(payload) {
  const task = payload.task && typeof payload.task === "object" ? payload.task : null;
  if (!task || !task.task_id) {
    return `<div class="robot-detail-grid">
      <section class="robot-detail-card"><h3>当前任务</h3><p class="robot-detail-empty">当前没有任务状态。</p></section>
      <section class="robot-detail-card"><h3>任务工作队列</h3><p class="robot-detail-empty">暂无待执行任务。</p></section>
    </div>`;
  }
  const progress = Number(task.progress);
  const workQueue = Array.isArray(task.work_queue) ? task.work_queue : [];
  const modelStatus = Array.isArray(task.model_status) ? task.model_status : [];
  const rows = workQueue.map((work, index) => `<tr>
    <td>${index + 1}</td><td><code>${escapeHtml(work)}</code></td>
    <td><code>${escapeHtml(modelStatus[index] || "Waiting")}</code></td>
  </tr>`).join("");
  return `<div class="robot-detail-grid">
    <section class="robot-detail-card"><h3>当前任务</h3>
      <p>任务 ID：<code>${escapeHtml(task.task_id)}</code></p>
      <p>汇总状态：<code>${escapeHtml(task.task_status || "Waiting")}</code></p>
      <p>${escapeHtml(task.message || "—")}</p>
      <p>进度：<code>${progress >= 0 ? `${Math.round(progress * 100)}%` : "未上报"}</code></p>
      <p>当前项：<code>${Number(task.total_count || 0) > 0 ? `${Number(task.current_index || 0) + 1} / ${Number(task.total_count)}` : "—"}</code></p>
    </section>
    <section class="robot-detail-card"><h3>任务工作队列</h3>
      ${rows ? `<table class="robot-detail-table"><thead><tr><th>#</th><th>执行模块</th><th>模块状态</th></tr></thead><tbody>${rows}</tbody></table>` : '<p class="robot-detail-empty">暂无工作项。</p>'}
    </section>
  </div>`;
}

function renderRobotDetailCpu(payload) {
  const processes = Array.isArray(payload.processes) ? payload.processes : [];
  const nodes = Array.isArray(payload.nodes) ? payload.nodes : [];
  const processRows = processes.length
    ? processes.map((p) => `<tr><td>${p.pid}</td><td>${Number(p.cpu_percent).toFixed(1)}%</td><td>${Number(p.memory_percent).toFixed(1)}%</td><td>${(Number(p.rss_kb) / 1024).toFixed(1)} MB</td><td class="robot-detail-command" title="${escapeHtml(p.command)}">${escapeHtml(p.command)}</td></tr>`).join("")
    : '<tr><td colspan="5">暂无匹配的机器人进程</td></tr>';
  const nodeRows = nodes.length
    ? nodes.map((n) => `<tr><td><code>${escapeHtml(n.name)}</code></td><td>${n.running ? "运行中" : "未运行"}</td><td>${escapeHtml((n.lifecycle && n.lifecycle.state) || "n/a")}</td></tr>`).join("")
    : '<tr><td colspan="3">暂无该机器人 ROS 节点</td></tr>';
  return `<section class="robot-detail-card"><h3>关联进程 CPU / 内存</h3><table class="robot-detail-table"><thead><tr><th>PID</th><th>CPU</th><th>MEM</th><th>RSS</th><th>命令</th></tr></thead><tbody>${processRows}</tbody></table></section>
    <section class="robot-detail-card" style="margin-top:10px"><h3>该机器人 ROS 节点</h3><table class="robot-detail-table"><thead><tr><th>节点</th><th>状态</th><th>Lifecycle</th></tr></thead><tbody>${nodeRows}</tbody></table></section>`;
}

function renderRobotDetailLogs(payload) {
  const entries = flattenLogBagEntries(payload.logs || {});
  if (!entries.length) return '<p class="robot-detail-empty">该机器人暂无日志索引。</p>';
  return `<div class="robot-detail-grid">${entries.slice(0, 12).map((entry) => `<section class="robot-detail-card"><h3>${escapeHtml(basenameOfLogPath(entry.bag) || "bag")}</h3><p>${escapeHtml(entry.ended_at || entry.started_at || "无时间")}</p><p>${escapeHtml(formatLogBagSize(entry.bytes))} · ${(entry.files || []).length} 个关联文件</p></section>`).join("")}</div><div class="robot-detail-param-actions"><button type="button" data-robot-detail-open-logs="1">打开完整日志页面</button></div>`;
}

function renderRobotDetailParams(payload) {
  const p = loadRobotDetailSettings(payload.robot_id);
  return `<form id="robot-detail-params-form">
    <div class="robot-detail-params">
      <label>最大速度 (m/s)<input name="maxSpeed" type="number" step="0.1" min="0" value="${p.maxSpeed}" required /></label>
      <label>角速度 (rad/s)<input name="angularSpeed" type="number" step="0.1" min="0" value="${p.angularSpeed}" required /></label>
      <label>避障距离 (m)<input name="safetyDistance" type="number" step="0.1" min="0" value="${p.safetyDistance}" required /></label>
      <label>刷新间隔 (ms)<input name="refreshInterval" type="number" step="100" min="100" value="${p.refreshInterval}" required /></label>
    </div>
    <div class="robot-detail-param-actions"><button type="submit">保存 ${escapeHtml(payload.robot_id)} 参数</button><span id="robot-detail-param-message" class="message"></span></div>
  </form>`;
}

function renderRobotDetail() {
  if (!robotDetailBody || !robotDetailPayload) return;
  const payload = robotDetailPayload;
  const renderers = {
    overview: renderRobotDetailOverview,
    tree: (p) => renderRobotBehaviorTree(p.nodes || []),
    tasks: renderRobotDetailTasks,
    cpu: renderRobotDetailCpu,
    logs: renderRobotDetailLogs,
    params: renderRobotDetailParams,
  };
  robotDetailBody.innerHTML = (renderers[robotDetailActiveTab] || renderRobotDetailOverview)(payload);
}

async function refreshRobotDetail() {
  if (!selectedDetailRobotId || !robotDetailPanel || robotDetailPanel.hidden) return;
  if (robotDetailRefreshInFlight) return;
  const requestedRobotId = selectedDetailRobotId;
  robotDetailRefreshInFlight = true;
  try {
    const payload = await fetchJson(`${API_BASE_URL}/api/robot/${encodeURIComponent(requestedRobotId)}/detail`, { cache: "no-store" });
    if (requestedRobotId !== selectedDetailRobotId || robotDetailPanel.hidden) return;
    robotDetailPayload = payload;
    const status = payload.status || {};
    if (robotDetailName) robotDetailName.textContent = status.name || payload.robot_id;
    if (robotDetailSubtitle) robotDetailSubtitle.textContent = `${payload.robot_id} · ${status.robot_model || "未知机型"} · ${status.online ? "在线" : "离线"} · ${status.floor || "无地图"} · ${status.current_position || "unknown;"}`;
    renderRobotDetail();
    renderRobotQuickDock();
  } finally {
    robotDetailRefreshInFlight = false;
  }
}

function closeRobotDetail() {
  if (robotDetailPanel) {
    robotDetailPanel.hidden = true;
    robotDetailPanel.setAttribute("aria-hidden", "true");
  }
  selectedDetailRobotId = "";
  robotDetailPayload = null;
  if (robotDetailPollTimer) clearInterval(robotDetailPollTimer);
  robotDetailPollTimer = null;
  renderRobotQuickDock();
}

function openRobotDetail(rid) {
  const normalizedRobotId = String(rid || "").trim();
  if (!normalizedRobotId) {
    return;
  }
  selectedDetailRobotId = normalizedRobotId;
  robotDetailActiveTab = "overview";
  robotDetailPayload = null;
  if (robotDetailPanel) {
    robotDetailPanel.hidden = false;
    robotDetailPanel.removeAttribute("hidden");
    robotDetailPanel.setAttribute("aria-hidden", "false");
  }
  if (robotDetailBody) robotDetailBody.innerHTML = '<p class="robot-detail-empty">正在读取机器人详情…</p>';
  if (robotDetailTabs) robotDetailTabs.querySelectorAll("button").forEach((b) => b.classList.toggle("active", b.dataset.robotDetailTab === "overview"));
  refreshRobotDetail().catch((err) => {
    if (robotDetailBody) robotDetailBody.innerHTML = `<p class="robot-detail-empty">读取失败：${escapeHtml(err.message || err)}</p>`;
  });
  if (robotDetailPollTimer) clearInterval(robotDetailPollTimer);
  robotDetailPollTimer = setInterval(() => refreshRobotDetail().catch(() => {}), 4000);
  renderRobotQuickDock();
}

function initRobotDetailUi() {
  renderRobotQuickDock();
  document.addEventListener("click", (ev) => {
    const target = ev.target instanceof Element ? ev.target : null;
    const button = target && target.closest("button[data-robot-quick-id]");
    if (!button) return;
    ev.preventDefault();
    openRobotDetail(String(button.dataset.robotQuickId || ""));
  }, true);
  if (btnRobotDetailClose) btnRobotDetailClose.addEventListener("click", closeRobotDetail);
  if (robotDetailTabs) robotDetailTabs.addEventListener("click", (ev) => {
    const button = ev.target.closest("button[data-robot-detail-tab]");
    if (!button) return;
    robotDetailActiveTab = String(button.dataset.robotDetailTab || "overview");
    robotDetailTabs.querySelectorAll("button").forEach((b) => b.classList.toggle("active", b === button));
    renderRobotDetail();
  });
  if (robotDetailBody) robotDetailBody.addEventListener("click", (ev) => {
    const button = ev.target.closest("button[data-robot-detail-open-logs]");
    if (!button || !selectedDetailRobotId) return;
    const menu = document.querySelector(`.menu-item[data-view="logs"]`);
    if (menu) menu.click();
    if (logBagRobotSelect) {
      logBagRobotSelect.value = selectedDetailRobotId;
      refreshLogBags();
    }
    closeRobotDetail();
  });
  if (robotDetailBody) robotDetailBody.addEventListener("submit", (ev) => {
    const form = ev.target.closest("#robot-detail-params-form");
    if (!form || !selectedDetailRobotId) return;
    ev.preventDefault();
    const payload = {
      maxSpeed: Number(form.elements.maxSpeed.value),
      angularSpeed: Number(form.elements.angularSpeed.value),
      safetyDistance: Number(form.elements.safetyDistance.value),
      refreshInterval: Number(form.elements.refreshInterval.value),
    };
    localStorage.setItem(robotDetailSettingsKey(selectedDetailRobotId), JSON.stringify(payload));
    const message = document.getElementById("robot-detail-param-message");
    if (message) message.textContent = `已保存 ${new Date().toLocaleTimeString()}`;
    appendLog(`${selectedDetailRobotId} 参数已更新 ${JSON.stringify(payload)}`);
  });
}
