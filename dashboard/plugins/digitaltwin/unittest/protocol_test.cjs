const assert = require('node:assert/strict');
const ts = require('typescript');
const fs = require('fs');
const vm = require('vm');
const result = {exports: {}};
vm.runInNewContext(ts.transpileModule(fs.readFileSync('src/protocol.ts', 'utf8'), {
  compilerOptions: {module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020}
}).outputText, {exports: result.exports, module: result, TextDecoder, DataView, Uint32Array, Float32Array, Uint8Array});
const {decodeMesh, decodeFaces, visible, validateStatus, VERTICES} = result.exports;
function packet(overrides = {}) {
  const text = JSON.stringify({mesh_sequence: 1, coordinates: 'body-relative x-right y-down z-forward', units: 'predicted metres', ...overrides});
  const n = Math.ceil(Buffer.byteLength(text) / 4) * 4;
  const b = Buffer.alloc(8 + n + VERTICES * 12, 0);
  b.write('HM01'); b.writeUInt32LE(n, 4); b.fill(32, 8, 8 + n); b.write(text, 8);
  b.writeFloatLE(1, 8 + n); b.writeFloatLE(2, 12 + n); b.writeFloatLE(3, 16 + n);
  return b.buffer.slice(b.byteOffset, b.byteOffset + b.length);
}
const decoded = decodeMesh(packet());
assert.deepEqual(Array.from(decoded.positions.slice(0, 3)), [1, -2, -3]);
assert.throws(() => decodeMesh(packet({coordinates: 'world'})));
assert.throws(() => decodeMesh(packet().slice(0, 100)));
const nonfinite = packet(); new DataView(nonfinite).setFloat32(nonfinite.byteLength - 4, NaN, true);
assert.throws(() => decodeMesh(nonfinite));
const faces = new ArrayBuffer(12); new DataView(faces).setUint32(8, 18439, true);
assert.throws(() => decodeFaces(faces));
new DataView(faces).setUint32(8, 2, true);
assert.deepEqual(Array.from(decodeFaces(faces)), [0, 0, 2]);
const status = {session: 'a', topology: 'a'.repeat(64), source: 'camera', source_error: null,
  mesh_available: true, stale: false, mesh_age_seconds: .3, max_age_seconds: 2,
  control: {running: true, worker: 'lab'}, workers: {}};
assert.equal(visible(validateStatus(status)), true);
for (const patch of [{stale: true}, {source_error: 'camera stopped'}, {mesh_available: false},
  {mesh_age_seconds: 3}, {mesh_age_seconds: null}, {control: {running: false}}]) {
  assert.equal(visible({...status, ...patch}), false);
}
assert.equal(visible({...status, source: 'demo', stale: true}), true);
assert.throws(() => validateStatus({...status, topology: 'bad'}));
assert.throws(() => validateStatus({...status, max_age_seconds: Infinity}));
console.log('PASS: HM01 decoding, coordinate rotation, topology bounds, invalid values, stale/absent/stop/demo visibility');
