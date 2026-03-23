const API_BASE_URL =
  window.API_BASE_URL || `${window.location.protocol}//${window.location.hostname}:8001`;

const menuButtons = Array.from(document.querySelectorAll(".menu-item"));
const views = {
  monitor: document.getElementById("view-monitor"),
  settings: document.getElementById("view-settings"),
  logs: document.getElementById("view-logs"),
};

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

const SETTINGS_KEY = "robotSettings";
const LOGS_KEY = "robotLogs";
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

menuButtons.forEach((btn) => {
  btn.addEventListener("click", () => {
    const target = btn.dataset.view;
    menuButtons.forEach((b) => b.classList.remove("active"));
    Object.values(views).forEach((v) => v.classList.remove("active"));
    btn.classList.add("active");
    views[target].classList.add("active");
  });
});

function addFloorOptions() {
  floorSelect.innerHTML = "";
  floors.forEach((floor) => {
    const option = document.createElement("option");
    option.value = floor;
    option.textContent = floor;
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

async function fetchFloors() {
  const data = await fetchJson(`${API_BASE_URL}/api/floors`);
  if (!Array.isArray(data.floors) || data.floors.length === 0) {
    throw new Error("后端没有可用地图楼层");
  }
  floors = data.floors;
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

function getRobotsOnCurrentMap() {
  if (!latestSnapshot || !Array.isArray(latestSnapshot.robots) || !activeFloor) {
    return [];
  }
  return latestSnapshot.robots.filter((r) => r && r.current_map === activeFloor);
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
}

function updateRobotStatus() {
  if (!robotStatus) {
    return;
  }
  const here = getRobotsOnCurrentMap();
  if (!activeFloor) {
    robotStatus.textContent = "定位: 未选择地图";
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
}

function refreshMetaPanel() {
  if (!activePgm || !activeMeta) {
    return;
  }
  const here = getRobotsOnCurrentMap();
  mapMeta.textContent = JSON.stringify(
    {
      floor: activeFloor,
      source: "backend-api",
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
    mapStatus.textContent = `${floor} 加载完成 · 滚轮缩放 · 拖拽平移`;
  } catch (err) {
    mapStatus.textContent = `加载失败: ${err.message}`;
    mapBitmap = null;
    activePgm = null;
    const { w, h } = getCanvasCssSize();
    ctx.fillStyle = "#020617";
    ctx.fillRect(0, 0, w, h);
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
  isDragging = true;
  dragLastX = ev.clientX;
  dragLastY = ev.clientY;
  canvas.style.cursor = "grabbing";
}

function onMouseMove(ev) {
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
  bindMapInteractions();

  if (btnResetView) {
    btnResetView.addEventListener("click", () => {
      resetViewToFit();
      renderScene();
      refreshMetaPanel();
      appendLog("地图视图已重置");
    });
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
  floorSelect.value = floors[0];
  resizeCanvasToDisplay();
  await loadFloorMap(floors[0]);
  await fetchPoseOnce();
  startPoseStream();

  floorSelect.addEventListener("change", async (e) => {
    await loadFloorMap(e.target.value);
    appendLog(`切换楼层到 ${e.target.value}`);
  });

  gridToggle.addEventListener("change", () => {
    renderScene();
  });
}

async function bootstrap() {
  initSettings();
  initLogs();
  await initMonitor();
}

bootstrap();
