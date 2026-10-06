// Run after installing the existing lidarpointcloud development dependencies:
// node dashboard/unittest/worker_health_test.cjs
// Uses Grafana's real transformation operators, not a reimplementation.
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const dataPath = path.resolve(__dirname, '../plugins/lidarpointcloud/node_modules/@grafana/data');
const data = require(dataPath);
const {of, lastValueFrom} = require(require.resolve('rxjs', {paths: [require.resolve(dataPath)]}));
const dashboard = JSON.parse(fs.readFileSync(path.resolve(__dirname, '../VISTA_Live_Dashboard_V4_Free_Space.json'), 'utf8'));
const panel = dashboard.panels.find(panel => panel.id === 28);
const transformers = Object.fromEntries(Object.values(data.standardTransformers).map(transform => [transform.id, transform]));
assert.equal(dashboard.uid, 'vistalive');
assert.equal(panel.targets[0].channel, 'stream/vista/worker_health_rows');
assert(!panel.transformations.some(transform => transform.id === 'prepareTimeSeries'));

const fields = [
  {name: 'labels', type: 'string'}, {name: 'time', type: 'time'},
  {name: 'worker', type: 'string'}, {name: 'failed', type: 'number'},
  {name: 'priority', type: 'number'}, {name: 'running', type: 'number'},
  {name: 'uptime_seconds', type: 'number'},
];

async function check(count, splitFrames) {
  const names = Array.from({length: count}, (_, i) => 'worker ' + String(i).padStart(2, '0'));
  const rows = [];
  // Reversed worker order and three snapshots; one worker fails/stops later.
  for (let epoch = 0; epoch < 3; epoch++) {
    for (let i = count - 1; i >= 0; i--) {
      rows.push(['', 1000 + epoch * 100 + i, names[i], epoch === 2 && i === 0 ? 1 : 0,
        i % 5, epoch === 2 && i === 0 ? 0 : 1, epoch * 10 + i]);
    }
  }
  const toFrame = rows => data.toDataFrame({name: 'worker_health_rows', fields: fields.map((field, index) => ({
    ...field, values: rows.map(row => row[index]),
  }))});
  let frames = splitFrames ? [toFrame(rows.slice(0, count)), toFrame(rows.slice(count))] : [toFrame(rows)];
  for (const transform of panel.transformations) {
    assert(transformers[transform.id], 'Unknown dashboard transform: ' + transform.id);
    frames = await lastValueFrom(transformers[transform.id].operator(transform.options, {interpolate: value => value})(of(frames)));
  }
  assert.equal(frames.length, 1);
  const frame = frames[0];
  const columns = frame.fields.map(field => field.config.displayName || field.name);
  assert.deepEqual(columns, ['STT', 'Worker', 'Failed', 'Priority', 'Running', 'Seconds']);
  assert.equal(frame.length, count);
  const values = name => Array.from(frame.fields[columns.indexOf(name)].values);
  assert.deepEqual(values('STT'), Array.from({length: count}, (_, index) => index + 1));
  assert.deepEqual(values('Worker'), names);
  assert.deepEqual(values('Failed'), names.map((_, index) => index === 0 ? 1 : 0));
  assert.deepEqual(values('Running'), names.map((_, index) => index === 0 ? 0 : 1));
  assert.deepEqual(values('Priority'), names.map((_, index) => index % 5));
  assert.deepEqual(values('Seconds'), names.map((_, index) => 20 + index));
}

(async () => {
  for (const count of [1, 9, 11, 12, 13]) {
    await check(count, false);
    await check(count, true);
  }
  console.log('Worker Health: 10 Grafana transformation scenarios passed.');
})().catch(error => {console.error(error); process.exitCode = 1;});
