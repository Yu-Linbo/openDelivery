const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const app = fs.readFileSync(path.join(__dirname, '../app.js'), 'utf8');
function section(source, start, end) {
  return source.slice(source.indexOf(start), source.indexOf(end, source.indexOf(start)));
}
function replayTrailContext(poses, activeMap = 'floor4') {
  const edges = [], starts = [];
  let previous;
  const context = {
    bagReplayState: { mapPgm: { width: 100, height: 100 } },
    bagReplayCtx: {
      save() {}, restore() {}, beginPath() {}, stroke() { this.stroked = true; },
      moveTo(x, y) { previous = { x, y }; starts.push(previous); },
      lineTo(x, y) { edges.push([previous, { x, y }]); previous = { x, y }; },
    },
    bagReplayTimeline: () => poses,
    bagReplayMapAt: time => poses.findLast(pose => pose.t <= time)?.map || activeMap,
    bagReplaySegmentAt: () => null,
    bagReplaySegmentsCompatible: () => true,
    bagReplayWorldToPixel: pose => ({ x: pose.x, y: pose.y }),
    bagReplayPixelToScreen: point => point,
  };
  vm.createContext(context);
  vm.runInContext(section(app, 'function bagReplayTrailIsContinuous(', 'function drawBagReplayScan('), context);
  return { context, edges, starts };
}

test('bag trail breaks at the floor-change localization offset and retains normal motion', () => {
  // The delivery bag briefly reports (-16, 22) after changing floors, then
  // localization returns to the destination elevator around (3.6, 2).
  const poses = [
    { t: 80.4, x: 3.6, y: 2.1, map: 'floor1' },
    { t: 80.5, x: -16.4, y: 22.1, map: 'floor4' },
    { t: 86.2, x: -16.4, y: 22.1, map: 'floor4' },
    { t: 86.3, x: 3.6, y: 2.0, map: 'floor4' },
    { t: 86.4, x: 3.6, y: 2.1, map: 'floor4' },
  ];
  const { context, edges } = replayTrailContext(poses);
  context.drawBagReplayTrail(87);
  assert.equal(edges.length, 1);
  assert.equal(edges[0][0].x, 3.6);
  assert.equal(edges[0][1].y, 2.1);
});

test('bag trail notices skipped floor changes and localization jumps before decimation', () => {
  for (const excursion of [{ map: 'floor1' }, { x: 40 }]) {
    const poses = Array.from({ length: 3002 }, (_, i) => ({ t: i * .1, x: 10 + i * .001, y: 10, map: 'floor4' }));
    Object.assign(poses[1], excursion); // Not one of the stride-3 drawing samples.
    const { context, edges, starts } = replayTrailContext(poses);
    context.drawBagReplayTrail(301);
    assert.ok(starts.length >= 2);
    assert.ok(edges.length > 900, 'normal trajectory remains visible');
    assert.ok(edges.every(([a, b]) => a.x !== 10 || b.x < 10.002), 'no bridge over the hidden discontinuity');
  }
});

test('bag trail treats gaps, frame changes and bag boundaries as separate strokes', () => {
  const { context } = replayTrailContext([]);
  const previous = { t: 1, x: 10, y: 10, frame_id: 'map', segment_index: 0 };
  for (const change of [{ t: 4 }, { frame_id: 'odom' }, { segment_index: 1 }, { x: NaN }]) {
    assert.equal(context.bagReplayTrailIsContinuous(previous, { ...previous, t: 1.1, ...change }), false);
  }
  assert.equal(context.bagReplayTrailIsContinuous(previous, { ...previous, t: 1.1, x: 10.1 }), true);
});

test('an invalid trailing pose does not discard the previously drawn bag trail', () => {
  const { context, edges } = replayTrailContext([
    { t: 0, x: 10, y: 10 }, { t: .1, x: 10.1, y: 10 }, { t: .2, x: NaN, y: 10 },
  ]);
  context.drawBagReplayTrail(1);
  assert.equal(edges.length, 1);
  assert.equal(context.bagReplayCtx.stroked, true);
});

test('bag view fits the current floor map independently of poses from other floors', () => {
  const context = {
    bagReplayCanvas: { clientWidth: 1000, clientHeight: 500 },
    bagReplayState: { mapPgm: { width: 100, height: 100 } },
    bagReplayTimeline: () => [{ x: -1600, y: 2200 }],
    bagReplayWorldToPixel: point => point,
  };
  vm.createContext(context);
  vm.runInContext(section(app, 'function resetBagReplayView(', 'function drawBagReplayGrid('), context);
  context.resetBagReplayView();
  assert.ok(Math.abs(context.bagReplayState.viewScale - 4.6) < 1e-9);
  assert.ok(Math.abs(context.bagReplayState.panX - 270) < 1e-9);
  assert.ok(Math.abs(context.bagReplayState.panY - 20) < 1e-9);
});
