const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const app = fs.readFileSync(path.join(__dirname, '../app.js'), 'utf8');
const editor = fs.readFileSync(path.join(__dirname, '../map_editor.js'), 'utf8');
function section(source, start, end) {
  return source.slice(source.indexOf(start), source.indexOf(end, source.indexOf(start)));
}
function deferred() {
  let resolve, reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
}

test('rotated editor map coordinates round-trip and match world axes', () => {
  const context = { s: { meta: { origin: [10, -3, Math.PI / 2], resolution: 0.05 }, pgm: { height: 100 } } };
  vm.createContext(context);
  vm.runInContext(section(editor, '  function toWorld(', '  function pointTypeLabel('), context);
  const world = context.toWorld({ x: 20, y: 60 });
  assert.ok(Math.abs(world.x - 8) < 1e-9);
  assert.ok(Math.abs(world.y + 2) < 1e-9);
  const pixel = context.toPixel(world);
  assert.ok(Math.abs(pixel.x - 20) < 1e-9);
  assert.ok(Math.abs(pixel.y - 60) < 1e-9);
});

test('corrupt or unavailable browser logs cannot interrupt operations', () => {
  const context = { LOGS_KEY: 'logs', localStorage: { getItem: () => '{broken', setItem: () => { throw Error('quota'); } } };
  vm.createContext(context);
  vm.runInContext(section(app, 'function getLogs()', 'let logBagEntries'), context);
  assert.equal(context.getLogs().length, 0);
  assert.doesNotThrow(() => context.appendLog('successful operation'));
  context.localStorage.getItem = () => '{}';
  assert.equal(context.getLogs().length, 0);
  context.localStorage.getItem = () => '["valid", null, 42]';
  assert.equal(context.getLogs().length, 1);
});

function mapContext() {
  const requests = new Map();
  const context = {
    floorLoadSequence: 0, activeFloor: '', activeMappingRobotId: null,
    activePgm: null, activeMeta: null, mapBitmap: null, mapEditorDialog: null,
    API_BASE_URL: 'http://api', mapStatus: {}, ctx: { fillRect() {} },
    isMappingFloor: () => false, stopMapLivePolling() {}, updateMappingToolbar() {},
    clearMapEditorDirty() {}, syncMapEditorUi() {}, resetViewToFit() {},
    renderScene() {}, updateRobotStatus() {}, refreshMetaPanel() {},
    getCanvasCssSize: () => ({ w: 1, h: 1 }),
    parseYaml: yaml => ({ yaml }), buildMapBitmap: pgm => pgm,
    loadActiveMapAssets: async () => {},
    fetchJson: url => {
      const request = deferred(); requests.set(url.split('/').pop(), request); return request.promise;
    },
  };
  vm.createContext(context);
  vm.runInContext(section(app, 'async function loadFloorMap(', 'async function fetchPoseOnce('), context);
  return { context, requests };
}

test('a late map response cannot overwrite a newer floor', async () => {
  const { context, requests } = mapContext();
  const first = context.loadFloorMap('floor1');
  const second = context.loadFloorMap('floor2');
  requests.get('floor2').resolve({ pgm: { width: 2 }, yaml: 'second' });
  await second;
  requests.get('floor1').resolve({ pgm: { width: 1 }, yaml: 'first' });
  await first;
  assert.equal(context.activeFloor, 'floor2');
  assert.equal(context.activePgm.width, 2);
  assert.match(context.mapStatus.textContent, /floor2/);
});

test('a late failed map request cannot clear a newer map', async () => {
  const { context, requests } = mapContext();
  const first = context.loadFloorMap('floor1');
  const second = context.loadFloorMap('floor2');
  requests.get('floor2').resolve({ pgm: { width: 2 }, yaml: 'second' });
  await second;
  requests.get('floor1').reject(Error('old failure'));
  await first;
  assert.equal(context.activeFloor, 'floor2');
  assert.equal(context.mapBitmap.width, 2);
});

test('a failed settings read keeps stale values unavailable for submission', async () => {
  const fields = [{ disabled: false }];
  const context = {
    settingsLoadSequence: 0, settingsForm: { elements: fields },
    settingsRobotId: { value: 'robot2' }, settingsMessage: {},
    settingsBindingStatus: {}, API_BASE_URL: 'http://api',
    setSettingsFormDisabled(disabled) { fields.forEach(field => { field.disabled = disabled; }); },
    fetchJson: async () => { throw Error('network'); },
  };
  vm.createContext(context);
  vm.runInContext(section(app, 'async function loadSettingsForRobot(', 'function syncSettingsRobotSelect('), context);
  await context.loadSettingsForRobot('robot2');
  assert.equal(fields[0].disabled, true);
  assert.match(context.settingsMessage.textContent, /network/);
});

function sensorContext() {
  const ticks = [];
  const requests = [];
  const context = {
    sensorPollGeneration: 0, sensorPollInFlight: false, sensorPollTimer: null,
    sensorPollControllers: new Set(), latestPathByRobot: {}, latestScanByRobot: {},
    API_BASE_URL: 'http://api', AbortController,
    plannedPathToggle: { checked: true }, scan2dToggle: { checked: false }, scanStreamActive: false,
    startScanStream() {}, stopScanStream() {},
    getRobotsOnCurrentMap: () => [{ id: 'robot2' }], paints: [],
    scheduleMapPaint() { context.paints.push(context.latestPathByRobot.robot2); },
    setTimeout, clearTimeout, setInterval: callback => { ticks.push(callback); return ticks.length; }, clearInterval() {},
    fetchJsonOptional: (url, options) => {
      const request = deferred(); requests.push({ ...request, url, options });
      options.signal.addEventListener('abort', () => request.reject(Error('aborted')));
      return request.promise;
    },
  };
  vm.createContext(context);
  vm.runInContext(section(app, 'function stopSensorPolling()', 'function stopScanStream()'), context);
  vm.runInContext(section(app, 'async function pollRobotSensor(', 'function bindMapInteractions()'), context);
  return { context, ticks, requests };
}
const flush = () => new Promise(resolve => setImmediate(resolve));

test('second path on the same map updates without reloading, and fetch bypasses caches', async () => {
  const { context, ticks, requests } = sensorContext();
  context.startSensorPolling();
  assert.equal(requests[0].options.cache, 'no-store');
  requests[0].resolve({ points: [[1, 1], [2, 2]] }); await flush();
  const next = ticks[0]();
  requests[1].resolve({ points: [[3, 3], [4, 4]] }); await next;
  assert.equal(context.latestPathByRobot.robot2.points[0][0], 3);
  assert.equal(context.paints.length, 2);
  context.stopSensorPolling();
});

test('a slow scan cannot block painting a new path', async () => {
  const { context, requests } = sensorContext();
  context.scan2dToggle.checked = true;
  context.startSensorPolling();
  requests.find(r => r.url.endsWith('planned_path')).resolve({ points: [[9, 9]] }); await flush();
  assert.equal(context.paints.length, 1);
  context.stopSensorPolling(); await flush();
});

test('restart discards stale responses and a failed sensor does not stop the next poll', async () => {
  const { context, requests, ticks } = sensorContext();
  context.startSensorPolling();
  context.startSensorPolling();
  assert.equal(requests[0].options.signal.aborted, true);
  requests[0].resolve({ points: [[1, 1]] });
  requests[1].reject(Error('temporary network failure')); await flush();
  const next = ticks[1]();
  requests[2].resolve({ points: [[8, 8]] }); await next;
  assert.equal(context.latestPathByRobot.robot2.points[0][0], 8);
  context.stopSensorPolling();
});
