// Test production scene/layer and socket lifecycle without a sensor/Grafana/WebGL.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const ts=require('typescript'),THREE=require('three');
const root=path.resolve(__dirname,'..'),previous=require.extensions['.ts'];
require.extensions['.ts']=(module,file)=>module._compile(ts.transpileModule(fs.readFileSync(file,'utf8'),
  {compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2020}}).outputText,file);
const {OverlayLayers,boundedCount}=require('../src/overlay_layers.ts');
const {OverlayStream}=require('../src/overlay_stream.ts');
const {decodePointCloudPacket}=require('../src/protocol.ts');
const options={maxPoints:100,foregroundMaxPoints:100,foregroundPointSize:3,foregroundColor:'#ff7043',colorMode:'solid',solidColor:'#a6b4c4'};
function packet(kind,count=3){
  const stride=kind==='GSP1'?44:16,buffer=new ArrayBuffer(32+count*stride),v=new DataView(buffer);
  new Uint8Array(buffer).set([...kind].map(x=>x.charCodeAt()));
  if(kind==='GSP1'){v.setUint32(4,32,true);v.setUint32(8,count,true);v.setUint32(12,44,true);v.setBigUint64(16,42n,true);}
  else{v.setUint8(4,1);v.setUint8(5,1);v.setUint16(6,32,true);v.setBigUint64(8,42n,true);v.setUint32(24,count,true);v.setUint32(28,16,true);}
  for(let i=0;i<count;++i){const values=kind==='GSP1'?[i,.1,1,.5,.01,0,0,.01,0,.001,.8]:[i,.1,1,128];
    values.forEach((x,j)=>v.setFloat32(32+i*stride+4*j,x,true));}
  return buffer;
}
const frame=(kind,count)=>decodePointCloudPacket(packet(kind,count));
let passed=0;const test=(name,run)=>{run();++passed;console.log('[PASS] '+name);};
test('overlay retains Gaussian covariance and independent foreground world points',()=>{
  const scene=new THREE.Scene(),layers=new OverlayLayers(scene);
  layers.setGaussian(frame('GSP1',4),options);layers.setForeground(frame('LPC1',3),options);
  assert.equal(scene.children.length,2);assert.equal(layers.gaussianCount,4);assert.equal(layers.foregroundCount,3);
  assert.equal(layers.foregroundGeometry.getAttribute('position').getX(2),2);
  assert.equal(layers.gaussian.geometry.instanceCount,4);assert(layers.gaussian.geometry.getAttribute('covarianceA'));
  layers.dispose();assert.equal(scene.children.length,0);
});
test('empty, expired or disconnected foreground never clears Gaussian and vice versa',()=>{
  const layers=new OverlayLayers(new THREE.Scene());layers.setGaussian(frame('GSP1',3),options);
  layers.setForeground(frame('LPC1',2),options);layers.setForeground(frame('LPC1',0),options);
  assert.equal(layers.gaussianCount,3);assert.equal(layers.foregroundCount,0);
  layers.setForeground(frame('LPC1',2),options);layers.clearGaussian();
  assert.equal(layers.gaussianCount,0);assert.equal(layers.foregroundCount,2);
  layers.setGaussian(frame('GSP1',2),options);layers.clearForeground();
  assert.equal(layers.gaussianCount,2);assert.equal(layers.foregroundGeometry.drawRange.count,0);layers.dispose();
});
test('overlay budgets, colors and sizes are independent and bounded',()=>{
  const layers=new OverlayLayers(new THREE.Scene()),o={...options,maxPoints:7,foregroundMaxPoints:11};
  layers.setGaussian(frame('GSP1',100),o);layers.setForeground(frame('LPC1',100),o);
  assert(layers.gaussianCount<=7);assert(layers.foregroundCount<=11);
  assert(layers.foregroundGeometry.getAttribute('position').count<=11);
  layers.configure({...o,foregroundColor:'#ff0000',foregroundPointSize:5,solidColor:'#ffffff'});
  assert.equal(layers.foregroundMaterial.color.getHexString(),'ff0000');assert.equal(layers.foregroundMaterial.size,5);
  assert.equal(layers.gaussianCount,7);assert.equal(layers.gaussian.mesh.visible,true);
  assert.equal(boundedCount(Infinity,100,200),100);assert.equal(boundedCount(-5,100,200),1);
  assert.equal(boundedCount(10000000,12000,50000),50000);layers.dispose();
});
test('layer toggles never combine buffers; a smaller fresh frame has no old trail',()=>{
  const layers=new OverlayLayers(new THREE.Scene());layers.setGaussian(frame('GSP1',2),options);
  layers.setForeground(frame('LPC1',80),options);layers.foreground.visible=false;
  layers.setForeground(frame('LPC1',2),options);assert.equal(layers.foreground.visible,false);
  assert.equal(layers.foregroundBounds().max.x,1);layers.foreground.visible=true;
  layers.gaussian.mesh.visible=false;layers.clearForeground();layers.foreground.visible=false;layers.foreground.visible=true;
  assert.equal(layers.foregroundGeometry.drawRange.count,0);assert.equal(layers.gaussianCount,2);layers.dispose();
});
test('dispose releases both layers GPU resources',()=>{
  const layers=new OverlayLayers(new THREE.Scene());let disposed=0;
  for(const item of [layers.gaussian.geometry,layers.gaussian.material,layers.foregroundGeometry,layers.foregroundMaterial])
    item.addEventListener('dispose',()=>++disposed);
  layers.dispose();assert.equal(disposed,4);
});
function harness(kind){
  let now=0,id=0;const sockets=[],timers=new Map(),events={frames:[],clears:[],status:[],maps:[],metadata:[]};
  const env={now:()=>now,later:(f)=>{timers.set(++id,f);return id;},cancel:i=>timers.delete(i),socket:url=>{
    const s={url,readyState:0,sent:[],close(){this.readyState=3;this.onclose?.({target:this});},
      send(x){this.sent.push(x);},open(){this.readyState=1;this.onopen?.({target:this});},
      message(data){this.onmessage?.({target:this,data});}};sockets.push(s);return s;}};
  const stream=new OverlayStream(kind,'ws://test/'+kind,{frame:x=>events.frames.push(x),clear:x=>events.clears.push(x),
    status:(...x)=>events.status.push(x),map:x=>events.maps.push(x),foreground:x=>events.metadata.push(x)},250,1000,env);
  return {stream,sockets,timers,events,time:x=>{now=x;},retry:()=>{const all=[...timers.values()];timers.clear();all.forEach(f=>f());}};
}
const meta=(active=true)=>JSON.stringify({type:'foreground-status',version:1,active,reason:active?'foreground; not a motion label':'localization unavailable',inputPoints:3,backgroundPoints:0,unknownPoints:0,foregroundPoints:active?3:0});
test('foreground expires once; metadata heartbeat is not a fresh binary frame',()=>{
  const h=harness('foreground');h.stream.start();h.sockets[0].open();h.sockets[0].message(packet('LPC1'));
  h.time(900);h.sockets[0].message(meta());h.stream.expire();assert.equal(h.events.clears.length,0);
  h.time(1000);h.stream.expire();h.time(2000);h.stream.expire();assert.equal(h.events.clears.length,1);
  h.sockets[0].message(packet('LPC1',1));assert.equal(h.events.frames.length,2);h.stream.stop();
});
test('Gaussian traffic cannot refresh foreground expiry and map transport retains geometry',()=>{
  const f=harness('foreground'),g=harness('gaussian');f.stream.start();g.stream.start();f.sockets[0].open();g.sockets[0].open();
  f.sockets[0].message(packet('LPC1'));g.sockets[0].message(packet('GSP1'));
  f.time(1000);g.sockets[0].message(packet('GSP1'));f.stream.expire();
  assert.equal(f.events.clears.length,1);assert.equal(g.events.clears.length,0);
  g.sockets[0].close();assert.equal(g.events.clears.length,0);assert.equal(g.timers.size,1);f.stream.stop();g.stream.stop();
});
test('paused foreground clears immediately and resumes on its first new frame',()=>{
  const h=harness('foreground');h.stream.start();h.sockets[0].open();h.sockets[0].message(packet('LPC1'));
  h.sockets[0].message(meta(false));assert.equal(h.events.clears.length,1);
  h.sockets[0].message(meta(true));assert.equal(h.events.frames.length,1);
  h.sockets[0].message(packet('LPC1'));assert.equal(h.events.frames.length,2);h.stream.stop();
});
test('wrong protocols, malformed packets, NaN and oversized foreground cannot replace layers',()=>{
  const f=harness('foreground'),g=harness('gaussian');f.stream.start();g.stream.start();f.sockets[0].open();g.sockets[0].open();
  f.sockets[0].message(packet('LPC1'));f.sockets[0].message(packet('GSP1'));assert.equal(f.events.frames.length,1);assert.equal(f.events.clears.length,1);
  g.sockets[0].message(packet('GSP1'));g.sockets[0].message(packet('LPC1'));assert.equal(g.events.frames.length,1);assert.equal(g.events.clears.length,0);
  const nan=packet('LPC1');new DataView(nan).setFloat32(32,NaN,true);f.sockets[0].message(nan);
  const huge=packet('LPC1',0);new DataView(huge).setUint32(24,50001,true);f.sockets[0].message(huge);
  f.sockets[0].message('{not JSON');assert.equal(f.events.frames.length,1);assert(f.events.status.at(-1)[1]);f.stream.stop();g.stream.stop();
});
test('disconnect/reconnect is per-layer and late callbacks cannot resurrect stale foreground',()=>{
  const h=harness('foreground');h.stream.start();const first=h.sockets[0];first.open();first.message(packet('LPC1'));
  const late=first.onmessage;first.close();assert.equal(h.events.clears.length,1);h.retry();h.sockets[1].open();
  late({target:first,data:packet('LPC1')});assert.equal(h.events.frames.length,1);
  h.sockets[1].message(packet('LPC1'));assert.equal(h.events.frames.length,2);
  h.sockets[1].close();const pending=[...h.timers.values()][0];h.stream.stop();pending();assert.equal(h.sockets.length,2);
  late({target:first,data:packet('LPC1')});assert.equal(h.events.frames.length,2);
});
test('Gaussian camera VIEW requests are shared, deduplicated and resent after reconnect',()=>{
  const h=harness('gaussian');h.stream.start();h.sockets[0].open();h.stream.requestView('VIEW camera1');h.stream.requestView('VIEW camera1');
  h.stream.requestView('VIEW camera2');assert.deepEqual(h.sockets[0].sent,['VIEW camera1','VIEW camera2']);
  h.sockets[0].close();h.retry();h.sockets[1].open();h.stream.requestView('VIEW camera2');assert.deepEqual(h.sockets[1].sent,['VIEW camera2']);h.stream.stop();
});
test('Gaussian explicit map errors clear only that map; startup socket failures retry and stop cleanly',()=>{
  const h=harness('gaussian');h.stream.start();h.sockets[0].open();h.sockets[0].message(packet('GSP1'));
  h.sockets[0].message(JSON.stringify({type:'roommap',totalPoints:0,pointBudget:100,state:6,bounds:[0,0,0,0,0,0]}));
  assert.equal(h.events.maps.length,1);assert.equal(h.events.clears.length,1);h.stream.stop();
  let retries=0,cancelled=0;const env={socket:()=>{throw Error('invalid URL');},now:()=>0,later:()=>++retries,cancel:()=>++cancelled};
  const s=new OverlayStream('foreground','bad',{frame:()=>{},clear:()=>{},status:()=>{}},250,1000,env);
  s.start();assert.equal(retries,1);s.stop();assert.equal(cancelled,1);
});
if(previous){require.extensions['.ts']=previous;}else{delete require.extensions['.ts'];}
console.log(`${passed}/${passed} overlay tests passed`);
