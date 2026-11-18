const assert = require('node:assert/strict');
const fs = require('node:fs');
const Module = require('node:module');
const path = require('node:path');
const ts = require('typescript');
const root = path.resolve(__dirname,'..');
function load(relative) {
  const filename = path.join(root,relative);
  const source = ts.transpileModule(fs.readFileSync(filename,'utf8'),{
    compilerOptions:{module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2020}
  }).outputText;
  const compiled = new Module(filename,module);
  compiled._compile(source,filename);
  return compiled.exports;
}
const { SettingsClient, encodeSettingsApply } = load('src/runtime_settings.ts');
const { settingGroups } = load('src/settings_schema.ts');
const dashboard = JSON.parse(fs.readFileSync(path.join(root,'../../VISTA_Edge_Settings.json'),'utf8'));
assert.equal(dashboard.panels.length,13);
assert.deepEqual(dashboard.panels.map(panel=>panel.options.settingsGroup),Object.keys(settingGroups));
assert(dashboard.panels.every(panel=>panel.options.settingsUrl==='$vista_settings_url'));
assert(!Object.values(settingGroups).some(group=>group.fields.some(field=>field.key==='LioEnabled' || field.key.startsWith('Mount'))));
assert.equal(new Set(Object.values(settingGroups).flatMap(group=>group.fields.map(field=>field.key))).size,
  Object.values(settingGroups).reduce((count,group)=>count+group.fields.length,0));
assert.equal(encodeSettingsApply(2,'lio_noise',{LioImuAccelerationCovariance:'0.2',LioImuGyroscopeCovariance:'0.2'}),
  'APPLY 2 lio_noise\nLioImuAccelerationCovariance=0.2\nLioImuGyroscopeCovariance=0.2');
assert.throws(()=>encodeSettingsApply(0,'logging',{RawLoggingEnabled:'1\nInjected=1'}));
assert.throws(()=>encodeSettingsApply(0,'logging',{}));
console.log('[PASS] 13 unique settings panels and bounded Apply commands');

class FakeSocket {
  static OPEN=1;
  static sockets=[];
  sent=[];
  constructor(){FakeSocket.sockets.push(this);}
  open(){this.readyState=1;this.onopen();}
  message(value){this.onmessage({data:JSON.stringify(value)});}
  close(){this.readyState=3;this.onclose?.();}
  send(value){this.sent.push(value);}
}
global.WebSocket=FakeSocket;
const client = new SettingsClient('ws://edge-settings.test:8769');
const stopA = client.subscribe(()=>{});
const stopB = client.subscribe(()=>{});
assert.equal(FakeSocket.sockets.length,1);
const socket=FakeSocket.sockets[0];
socket.open();
client.authenticate('test_only_settings_token_01234567');
assert.deepEqual(socket.sent,['AUTH test_only_settings_token_01234567']);
const active={type:'state',revision:0,busy:false,realSense:false,settings:{RawLoggingEnabled:'1'}};
socket.message(active);
client.apply('logging',{RawLoggingEnabled:'0'});
client.apply('logging',{RawLoggingEnabled:'1'});
assert.equal(socket.sent.length,2);
assert.equal(client.getSnapshot().snapshot.settings.RawLoggingEnabled,'1');
socket.message({...active, revision:1, settings:{RawLoggingEnabled:'0'}});
socket.message({type:'result',group:'logging',ok:true,message:'Applied.'});
assert.equal(client.getSnapshot().snapshot.settings.RawLoggingEnabled,'0');
assert.equal(client.getSnapshot().pending,undefined);
stopA();
stopB();
console.log('[PASS] shared client, server-backed values, and serialized Apply');
