// Pure protocol/camera tests: no running Grafana, WebGL, sensor, or network needed.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const Module = require('node:module');
const ts = require('typescript');
const THREE = require('three');
const root = path.resolve(__dirname, '..');
const loadTypeScript = (relative) => {
  const filename = path.join(root, relative);
  const output = ts.transpileModule(fs.readFileSync(filename, 'utf8'), {
    compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020 }
  }).outputText;
  const module = new Module(filename, moduleParent);
  module.filename = filename;
  module.paths = Module._nodeModulePaths(root);
  module._compile(output, filename);
  return module.exports;
};
const moduleParent = module;
const { decodeRoomMapMetadata, makeRoomMapViewRequest } = loadTypeScript('src/room_map_view.ts');
const { decodePointCloudPacket } = loadTypeScript('src/protocol.ts');
const { GaussianRenderer } = loadTypeScript('src/gaussian_renderer.ts');
let passed = 0;
const test = (name, run) => { run(); ++passed; console.log(`[PASS] ${name}`); };
const metadata = { type: 'roommap', totalPoints: 300000, pointBudget: 250000, state: 2, bounds: [-1,-2,-3,4,5,6] };
const gaussianPacket = () => {
  const bytes=new ArrayBuffer(76),v=new DataView(bytes);
  new Uint8Array(bytes).set([71,83,80,49]);
  v.setUint32(4,32,true);v.setUint32(8,1,true);v.setUint32(12,44,true);
  v.setBigUint64(16,42n,true);v.setBigUint64(24,123n,true);
  [1,2,3,.5,.003025,0,0,.003025,0,.000064,.85].forEach((x,i)=>v.setFloat32(32+4*i,x,true));
  return bytes;
};
test('GSP1 carries world covariance, opacity, intensity and coherent revision',()=>{
  const frame=decodePointCloudPacket(gaussianPacket());
  assert.equal(frame.protocol,'GSP1');assert.equal(frame.frameId,42n);
  assert.deepEqual(Array.from(frame.positions),[1,2,3]);
  assert(frame.covariances[0]>frame.covariances[5]*40);
  assert(Math.abs(frame.opacities[0]-.85)<1e-6);
  const empty=gaussianPacket().slice(0,32);new DataView(empty).setUint32(8,0,true);
  assert.equal(decodePointCloudPacket(empty).pointCount,0);
});
test('GSP1 rejects malformed lengths, nonfinite data and non-positive covariance',()=>{
  assert.throws(()=>decodePointCloudPacket(gaussianPacket().slice(0,75)));
  for(const [offset,value] of [[32,NaN],[48,-1],[72,2],[44,2],[52,1]]) {
    const bytes=gaussianPacket();new DataView(bytes).setFloat32(offset,value,true);
    assert.throws(()=>decodePointCloudPacket(bytes));
  }
});
test('Gaussian renderer uses covariance ellipses, sorted alpha and clears empty replacements',()=>{
  const renderer=new GaussianRenderer();
  renderer.set(new Float32Array([0,0,-1,0,0,-3]),new Float32Array(12).fill(.001),
    new Float32Array([1,0,0,0,1,0]),new Float32Array([.8,.9]));
  const camera=new THREE.PerspectiveCamera(55,1,.01,100);
  renderer.update(camera,600,600,0);
  assert.equal(renderer.geometry.instanceCount,2);
  assert.equal(renderer.geometry.getAttribute('center').getZ(0),-3);
  assert.equal(renderer.material.depthWrite,false);
  assert(renderer.material.vertexShader.includes('C*jx'));
  assert(renderer.material.fragmentShader.includes('exp(-0.5*radius2)'));
  renderer.set(new Float32Array(0),new Float32Array(0),new Float32Array(0),new Float32Array(0));
  assert.equal(renderer.geometry.instanceCount,0);renderer.dispose();
});
test('total stored points may exceed render budget', () => {
  const result = decodeRoomMapMetadata(JSON.stringify(metadata));
  assert.equal(result.totalPoints, 300000);
  assert.equal(result.pointBudget, 250000);
});
test('invalid bounds/state/budgets are rejected', () => {
  assert.equal(decodeRoomMapMetadata(JSON.stringify({ ...metadata, bounds: [5,0,0,1,1,1] })), undefined);
  assert.equal(decodeRoomMapMetadata(JSON.stringify({ ...metadata, state: 99 })), undefined);
  assert.equal(decodeRoomMapMetadata(JSON.stringify({ ...metadata, pointBudget: 0 })), undefined);
  assert.equal(decodeRoomMapMetadata(JSON.stringify({ ...metadata, totalPoints: -1 })), undefined);
  assert.equal(decodeRoomMapMetadata(JSON.stringify({ ...metadata, bounds: [null,0,0,1,1,1] })), undefined);
});
test('storage error metadata is accepted', () => {
  assert.equal(decodeRoomMapMetadata(JSON.stringify({ ...metadata, state: 6 })).state, 6);
});
const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 1000);
camera.position.set(0,0,10); camera.lookAt(0,0,0);
test('VIEW includes bounded budget, viewport, camera, and six normalized planes', () => {
  const values = makeRoomMapViewRequest(camera, 10000, 3000000).split(' ');
  assert.equal(values.length, 30); assert.equal(values[0], 'VIEW');
  assert.equal(values[1], '2000000'); assert.equal(values[2], '8192');
  assert(values.slice(3).map(Number).every(Number.isFinite));
  for (let i = 6; i < 30; i += 4) {
    assert(Math.abs(Math.hypot(...values.slice(i, i + 3).map(Number)) - 1) < 0.00001);
    const plane = values.slice(i, i + 4).map(Number);
    assert(plane[3] > -0.00001); // World origin is inside this view.
  }
});
test('camera moves change requests; unchanged camera requests remain identical', () => {
  const initial = makeRoomMapViewRequest(camera, 600, 1000);
  assert.equal(makeRoomMapViewRequest(camera, 600, 1000), initial);
  camera.position.x = 5; camera.lookAt(0,0,0);
  assert.notEqual(makeRoomMapViewRequest(camera, 600, 1000), initial);
});
test('room-map LPC1 view and empty replacement remain compatible', () => {
  const buffer = new ArrayBuffer(48); const view = new DataView(buffer);
  new Uint8Array(buffer).set([76,80,67,49,1,1,32,0]);
  view.setBigUint64(8, 7n, true); view.setBigUint64(16, 10n, true);
  view.setUint32(24, 1, true); view.setUint32(28, 16, true);
  view.setFloat32(32, 1, true); view.setFloat32(36, 2, true); view.setFloat32(40, 3, true);
  view.setFloat32(44, 4, true);
  const frame = decodePointCloudPacket(buffer);
  assert.equal(frame.pointCount, 1); assert.deepEqual(Array.from(frame.positions), [1,2,3]);
  const empty = buffer.slice(0,32); new DataView(empty).setUint32(24,0,true);
  assert.equal(decodePointCloudPacket(empty).pointCount, 0);
});
console.log(`${passed}/${passed} plugin tests passed`);
