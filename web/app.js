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
const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");

const settingsForm = document.getElementById("settings-form");
const settingsMessage = document.getElementById("settings-message");
const logList = document.getElementById("log-list");
const addLogBtn = document.getElementById("btn-add-log");
const clearLogBtn = document.getElementById("btn-clear-log");

const SETTINGS_KEY = "robotSettings";
const LOGS_KEY = "robotLogs";
const ROBOT_ICON_PATH = "./icons/robot.svg";
let floors = [];
let activeFloor = null;
let activePgm = null;
let activeMeta = null;
let activeRender = null;
let latestPose = null;
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

function drawMap(pgm, showGrid) {
  const { width, height, maxVal, data } = pgm;
  const imageData = ctx.createImageData(width, height);

  for (let i = 0; i < width * height; i += 1) {
    const v = Math.round((data[i] / maxVal) * 255);
    const p = i * 4;
    imageData.data[p] = v;
    imageData.data[p + 1] = v;
    imageData.data[p + 2] = v;
    imageData.data[p + 3] = 255;
  }

  const offscreen = document.createElement("canvas");
  offscreen.width = width;
  offscreen.height = height;
  offscreen.getContext("2d").putImageData(imageData, 0, 0);

  const scale = Math.min(canvas.width / width, canvas.height / height);
  const drawW = Math.floor(width * scale);
  const drawH = Math.floor(height * scale);
  const offsetX = Math.floor((canvas.width - drawW) / 2);
  const offsetY = Math.floor((canvas.height - drawH) / 2);

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(offscreen, offsetX, offsetY, drawW, drawH);

  if (showGrid) {
    ctx.strokeStyle = "rgba(34, 197, 94, 0.18)";
    ctx.lineWidth = 1;
    const gridSize = Math.max(8, Math.floor(drawW / 80));

    for (let x = offsetX; x <= offsetX + drawW; x += gridSize) {
      ctx.beginPath();
      ctx.moveTo(x + 0.5, offsetY);
      ctx.lineTo(x + 0.5, offsetY + drawH);
      ctx.stroke();
    }

    for (let y = offsetY; y <= offsetY + drawH; y += gridSize) {
      ctx.beginPath();
      ctx.moveTo(offsetX, y + 0.5);
      ctx.lineTo(offsetX + drawW, y + 0.5);
      ctx.stroke();
    }
  }

  return { offsetX, offsetY, drawW, drawH };
}

function drawRobot(pose) {
  if (!activeRender || !activeMeta || !activePgm || !pose) {
    return;
  }
  const resolution = Number(activeMeta.resolution);
  if (!resolution || Number.isNaN(resolution)) {
    return;
  }
  const [ox, oy] = parseOrigin(activeMeta.origin);
  const mapX = (pose.pose.x - ox) / resolution;
  const mapY = activePgm.height - (pose.pose.y - oy) / resolution;
  const px = activeRender.offsetX + (mapX / activePgm.width) * activeRender.drawW;
  const py = activeRender.offsetY + (mapY / activePgm.height) * activeRender.drawH;

  if (
    px < activeRender.offsetX ||
    px > activeRender.offsetX + activeRender.drawW ||
    py < activeRender.offsetY ||
    py > activeRender.offsetY + activeRender.drawH
  ) {
    return;
  }

  ctx.save();
  ctx.translate(px, py);
  ctx.rotate(-(pose.pose.yaw || 0));
  if (robotIconLoaded) {
    const size = 28;
    ctx.drawImage(robotIcon, -size / 2, -size / 2, size, size);
  } else {
    // fallback while icon is loading or if it fails.
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
}

function renderScene() {
  if (!activePgm) {
    return;
  }
  activeRender = drawMap(activePgm, gridToggle.checked);
  drawRobot(latestPose);
}

function updateRobotStatus(pose) {
  if (!robotStatus) {
    return;
  }
  if (!pose || !pose.pose) {
    robotStatus.textContent = "机器人: 未连接";
    return;
  }
  robotStatus.textContent = `机器人: x=${pose.pose.x.toFixed(2)} y=${pose.pose.y.toFixed(2)} yaw=${pose.pose.yaw.toFixed(2)}`;
}

function refreshMetaPanel() {
  if (!activePgm || !activeMeta) {
    return;
  }
  mapMeta.textContent = JSON.stringify(
    {
      floor: activeFloor,
      source: "backend-api",
      width: activePgm.width,
      height: activePgm.height,
      resolution: activeMeta.resolution,
      origin: activeMeta.origin,
      robot_pose: latestPose ? latestPose.pose : null,
      robot_source: latestPose ? latestPose.source : null,
      occupied_thresh: activeMeta.occupied_thresh,
      free_thresh: activeMeta.free_thresh,
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
    renderScene();
    refreshMetaPanel();
    mapStatus.textContent = `${floor} 加载完成 (backend)`;
  } catch (err) {
    mapStatus.textContent = `加载失败: ${err.message}`;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
  }
}

async function fetchPoseOnce() {
  try {
    latestPose = await fetchJson(`${API_BASE_URL}/api/robot/pose`);
    updateRobotStatus(latestPose);
    renderScene();
    refreshMetaPanel();
  } catch (err) {
    updateRobotStatus(null);
  }
}

function startPoseStream() {
  const eventSource = new EventSource(`${API_BASE_URL}/api/robot/pose/stream`);
  eventSource.onmessage = (event) => {
    latestPose = JSON.parse(event.data);
    updateRobotStatus(latestPose);
    renderScene();
    refreshMetaPanel();
  };
  eventSource.onerror = () => {
    eventSource.close();
    setInterval(fetchPoseOnce, 1000);
  };
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
  await fetchFloors();
  addFloorOptions();
  floorSelect.value = floors[0];
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
