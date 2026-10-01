const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
// Run the actual parser; the panel code following its export needs a browser.
const source = fs.readFileSync(require('node:path').join(__dirname, '../log_viewer.js'), 'utf8');
const parserCode = source.slice(0, source.indexOf('  function timeText')) + '\n})();';
const context = { window: {} }; vm.runInNewContext(parserCode, context);
const parse = context.window.ROSLogParser.parseLine;
assert.equal(parse('[WARN] [1789885202.024] [amcl]: late scan', 0).level, 'WARN');
assert.equal(parse('[node-1] \x1b[31m[ERROR] [12.5] [node]: failed\x1b[0m', 2).message, 'failed');
assert.equal(parse('12.5 [WARNING] [node][tick][42]: late', 0).message, 'tick · L42 · late');
assert.equal(parse('[12.5] [INFO] [launch]: started', 0).node, 'launch');
assert.equal(parse('[2026-09-30T12:00:00+0800] rosout level=40 name=amcl file=amcl.cpp line=42 msg=failed', 0).level, 'ERROR');
assert.equal(parse('[2026-09-30T12:00:00+0800] opening bag', 0).node, 'robot_log_recorder');
assert.equal(parse('  traceback <script>alert(1)</script>', 0).level, 'RAW');
assert.equal(parse('[INFO] [0.000000000] [clock]: zero', 0).epoch, 0);
assert.equal(parse('[INFO] [12.123456789] [node]: hello', 0).raw, '[INFO] [12.123456789] [node]: hello');
console.log('9 parser cases passed');
