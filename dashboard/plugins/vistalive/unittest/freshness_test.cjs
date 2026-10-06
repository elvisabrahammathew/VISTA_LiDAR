const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const ts = require('typescript');
const rx = require('rxjs');

function load(name, dependencies, globals = {}) {
  const module = {exports: {}};
  const file = path.resolve(__dirname, '../src/' + name + '.ts');
  const code = ts.transpileModule(fs.readFileSync(file, 'utf8'), {
    compilerOptions: {module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020},
  }).outputText;
  vm.runInNewContext(code, {module, exports: module.exports,
    require: id => dependencies[id], Date, setInterval, clearInterval, ...globals}, {filename: file});
  return module.exports;
}

const {Freshness} = load('freshness', {});
const f = new Freshness();
assert.equal(f.live(1000), false);
f.observe(NaN); f.observe(Infinity);
assert.equal(f.live(1000), false);
f.observe(1000);
assert.equal(f.live(1000), true);
assert.equal(f.live(7000), true);
assert.equal(f.live(7001), false);
f.observe(999); // Replayed/duplicate buffers must not reset timeout.
assert.equal(f.live(7001), false);
f.observe(8000);
assert.equal(f.live(8000), true);
assert.equal(f.live(5000), false); // Do not trust a wildly future timestamp.
f.reset();
assert.equal(f.live(8000), false);

const {snapshotFrames} = load('datasource', {
  '@grafana/data': {DataSourceApi: class {}},
  '@grafana/runtime': {}, 'rxjs': rx, './freshness': {Freshness},
});
// Model a native StreamingDataFrame, including mutable ring-buffer values and
// cached field state. Tests must not depend on a local Grafana installation.
const nativeFrame = Object.assign(Object.create({serialize() {}}), {
  name: 'Workers', refId: 'A', meta: {preferredVisualisationType: 'table'},
  packetInfo: {schemaChanged: false}, length: 2,
  fields: [{name: 'Running', type: 'number', config: {unit: 'short'},
    labels: {worker: 'worker-1'}, state: {calcs: {last: 1}}, values: [1, 1]}],
});
const detached = snapshotFrames([nativeFrame, {
  name: 'Timing', refId: 'B', length: 2,
  fields: [{name: 'Seconds', type: 'number', config: {unit: 's'}, values: new Float64Array([10, 20])}],
}]);
assert.equal(detached.length, 2);
assert.equal(detached[0].name, 'Workers');
assert.equal(detached[0].refId, 'A');
assert.equal(detached[0].meta.preferredVisualisationType, 'table');
assert.equal(detached[0].fields[0].config.unit, 'short');
assert.equal(detached[0].fields[0].labels.worker, 'worker-1');
assert.equal(detached[0].length, 2);
assert.equal(detached[0].packetInfo, undefined);
assert.equal(detached[0].serialize, undefined);
assert.equal(detached[0].fields[0].state, undefined);
assert.equal(JSON.stringify(detached[1].fields[0].values), '[10,20]');
nativeFrame.fields[0].values[0] = 0;
nativeFrame.fields[0].config.unit = 'none';
nativeFrame.fields[0].labels.worker = 'changed';
nativeFrame.meta.preferredVisualisationType = 'graph';
assert.equal(detached[0].fields[0].values[0], 1);
assert.equal(detached[0].fields[0].config.unit, 'short');
assert.equal(detached[0].fields[0].labels.worker, 'worker-1');
assert.equal(detached[0].meta.preferredVisualisationType, 'table');
// Same condition as Grafana 13.2.2's unsafe fast path after lastProcessedFrames
// is cleared. Native frames fail; plain snapshots bypass it and recover safely.
function panelResume(frame) {
  const lastProcessedFrames = [];
  const streamingDataFrame = frame.packetInfo ? frame : undefined;
  return !!(streamingDataFrame && !streamingDataFrame.packetInfo.schemaChanged &&
    lastProcessedFrames[0].fields.length === streamingDataFrame.fields.length);
}
assert.throws(() => panelResume(nativeFrame), TypeError);
assert.equal(panelResume(detached[0]), false);

async function fixture() {
  let now = 1000000, active = 0;
  const intervals = new Set();
  const streams = {heartbeat: [], data: []};
  const requests = [];
  const builtin = {query(request) {
    requests.push(request);
    const key = request.targets[0].refId === '__VISTA_HEARTBEAT' ? 'heartbeat' : 'data';
    const subject = new rx.Subject(); streams[key].push(subject);
    return new rx.Observable(subscriber => {
      active++;
      const subscription = subject.subscribe(subscriber);
      return () => {active--; subscription.unsubscribe();};
    });
  }};
  const {VistaLiveDataSource} = load('datasource', {
    '@grafana/data': {DataSourceApi: class {}, FieldType: {time: 'time'}, LoadingState: {Streaming: 'Streaming'}},
    '@grafana/runtime': {getDataSourceSrv: () => ({get: async () => builtin})},
    'rxjs': rx, './freshness': {Freshness},
  }, {Date: {now: () => now}, setInterval: callback => {intervals.add(callback); return callback;}, clearInterval: callback => intervals.delete(callback)});
  const source = new VistaLiveDataSource({});
  const results = [];
  const subscription = source.query({targets: [{refId: 'A', queryType: 'measurements', channel: 'stream/vista/worker_health_rows', buffer: 60000}]}).subscribe(result => results.push(result));
  await Promise.resolve(); await Promise.resolve();
  const frames = [{name: 'Workers', refId: 'A', packetInfo: {schemaChanged: false}, length: 2, fields: [{name: 'worker', type: 'string', values: ['worker-1','worker-2']}, {name: 'running', type: 'number', labels: {source: 'test'}, values: [1,1]}]}];
  const payload = () => streams.data.at(-1).next({data: frames});
  const heartbeat = timestamp => streams.heartbeat.at(-1).next({data: [{fields: [{name: 'time', type: 'time', values: [timestamp]}]}]});
  const tick = delta => {now += delta; for (const callback of intervals) {callback();}};
  const latest = () => results.at(-1).data;
  const assertVisible = () => {
    assert.equal(latest().length, 1);
    assert.notEqual(latest(), frames);
    assert.equal(latest()[0].packetInfo, undefined);
    assert.equal(latest()[0].name, 'Workers');
    assert.equal(latest()[0].refId, 'A');
    assert.equal(latest()[0].length, 2);
    assert.equal(latest()[0].fields[0].type, 'string');
    assert.equal(latest()[0].fields[1].labels.source, 'test');
    assert.equal(JSON.stringify(latest()[0].fields[0].values), '["worker-1","worker-2"]');
    assert.equal(panelResume(latest()[0]), false);
  };
  assert.equal(latest().length, 0); // Startup must not show cached data.
  assert.equal(active, 2);
  assert.equal(intervals.size, 1);
  assert.equal(requests[0].targets[0].datasource.type, 'grafana');
  assert.equal(requests[1].targets[0].channel, 'stream/vista/worker_health_rows');
  payload(); assert.equal(latest().length, 0);
  heartbeat(now); assertVisible(); // Preserve all rows/labels/field types.
  const oldResult = latest();
  frames[0].fields[1].values[0] = 0;
  assert.equal(oldResult[0].fields[1].values[0], 1); // No mutable live-buffer alias.
  frames[0].fields[1].values[0] = 1;
  tick(6000); assertVisible();
  tick(1); assert.equal(latest().length, 0);
  payload(); assert.equal(latest().length, 0); // Stale buffered payload cannot revive panel.
  heartbeat(now-6001); assert.equal(latest().length, 0);
  heartbeat(now); assertVisible(); // Resume without a query refresh.
  tick(20000); assert.equal(latest().length, 0); // Background-tab timer catch-up.
  heartbeat(now); assertVisible();
  streams.data.at(-1).error(new Error('test connection lost'));
  assert.equal(latest().length, 0); // Real errors clear immediately.
  await new Promise(resolve => setTimeout(resolve, 1100));
  assert.equal(streams.data.length, 2); // Automatically re-subscribed.
  payload(); assertVisible();
  // Completion is not an error: retry alone cannot recover either stream.
  streams.data.at(-1).complete();
  assert.equal(latest().length, 0);
  await new Promise(resolve => setTimeout(resolve, 1100));
  assert.equal(streams.data.length, 3);
  payload(); assertVisible();
  streams.heartbeat.at(-1).complete();
  assert.equal(latest().length, 0);
  await new Promise(resolve => setTimeout(resolve, 1100));
  assert.equal(streams.heartbeat.length, 2);
  payload(); assert.equal(latest().length, 0);
  heartbeat(now); assertVisible();
  streams.heartbeat.at(-1).next({data: [], error: {message: 'test Grafana disconnected'}});
  assert.equal(latest().length, 0); // Response-level errors, not just thrown errors.
  streams.data.at(-1).complete(); // Teardown cancels repeat AND retry timers.
  subscription.unsubscribe();
  assert.equal(active, 0);
  assert.equal(intervals.size, 0);
  await new Promise(resolve => setTimeout(resolve, 1100));
  assert.equal(streams.heartbeat.length, 2); // Retry timer cancelled on teardown.
  assert.equal(streams.data.length, 3); // Repeat timer cancelled on teardown.
  assert.equal((await source.testDatasource()).status, 'success');
}

async function emptyCompletion() {
  let attempts = 0;
  const intervals = new Set();
  const builtin = {query() {attempts++; return rx.EMPTY;}};
  const {VistaLiveDataSource} = load('datasource', {
    '@grafana/data': {DataSourceApi: class {}, FieldType: {time: 'time'}, LoadingState: {Streaming: 'Streaming'}},
    '@grafana/runtime': {getDataSourceSrv: () => ({get: async () => builtin})},
    'rxjs': rx, './freshness': {Freshness},
  }, {setInterval: callback => {intervals.add(callback); return callback;}, clearInterval: callback => intervals.delete(callback)});
  const subscription = new VistaLiveDataSource({}).query({targets: [{refId: 'A'}]}).subscribe();
  await Promise.resolve(); await Promise.resolve();
  assert.equal(attempts, 2);
  await new Promise(resolve => setTimeout(resolve, 150));
  assert.equal(attempts, 2); // Completed/empty stream must not spin in a tight loop.
  await new Promise(resolve => setTimeout(resolve, 1000));
  assert.equal(attempts, 4);
  subscription.unsubscribe();
  assert.equal(intervals.size, 0);
  await new Promise(resolve => setTimeout(resolve, 1100));
  assert.equal(attempts, 4);
}

const dashboard = JSON.parse(fs.readFileSync(path.resolve(__dirname, '../../../VISTA_Live_Dashboard_V4_Free_Space.json'), 'utf8'));
assert.equal(dashboard.uid, 'vistalive');
assert.equal(dashboard.__inputs[0].pluginId, 'vista-live-datasource');
const metrics = dashboard.panels.filter(panel => ['stat','table','timeseries'].includes(panel.type));
assert.equal(metrics.length, 26);
for (const panel of metrics) {
  assert.equal(panel.datasource.type, 'vista-live-datasource');
  assert.equal(panel.fieldConfig.defaults.noValue, 'No data');
  assert.equal(panel.targets.length, 1);
  assert.equal(panel.targets[0].datasource.uid, '${DS_VISTA_LIVE}');
  assert(!JSON.stringify(panel.transformations).includes('__Bridge'));
}
const threeD = dashboard.panels.filter(panel => panel.type === 'vista-lidarpointcloud-panel');
assert.equal(threeD.length, 2);
for (const panel of threeD) {assert.notEqual(panel.datasource?.type, 'vista-live-datasource');}
Promise.all([fixture(), emptyCompletion()]).then(() => console.log('VISTA Live: freshness, detached snapshots, Grafana empty-result recovery, error/retry, completion/repeat, bounded reconnect, teardown and 26-panel dashboard checks passed.')).catch(error => {console.error(error); process.exitCode = 1;});
