'use strict';
const assert=require('assert/strict');
const fs=require('fs');
const path=require('path');
const file=path.join(__dirname,'..','VISTA_Live_Dashboard_V5_Gaussian_Map.json');
const doc=JSON.parse(fs.readFileSync(file,'utf8'));
assert.equal(doc.uid,'vistalive');
assert.match(doc.title,/\(V5\)/);
assert.equal(new Set(doc.panels.map(p=>p.id)).size,doc.panels.length);
assert(!doc.panels.some(p=>p.id===52 || p.id===53));
assert(!JSON.stringify(doc).includes('stream/vista/ground_'));
assert.equal(doc.panels.find(p=>p.id===50).title,'Room Map, Gaussian Map and Foreground');
const state=doc.panels.find(p=>p.id===61).gridPos;
const localization=doc.panels.find(p=>p.id===85).gridPos;
assert.deepEqual(state,{x:0,y:localization.y+localization.h,w:24,h:4});
for(const id of [51,60,85,86,74,83,84,3,35,36,28,29])
  assert(doc.panels.some(p=>p.id===id),`Required map/health panel missing: ${id}`);
for(let i=0;i<doc.panels.length;i++){
  const a=doc.panels[i].gridPos;
  assert(a.x>=0 && a.y>=0 && a.w>0 && a.h>0 && a.x+a.w<=24);
  for(let j=0;j<i;j++){
    const b=doc.panels[j].gridPos;
    assert(!(a.x<b.x+b.w && b.x<a.x+a.w && a.y<b.y+b.h && b.y<a.y+a.h),
      `Panel overlap: ${doc.panels[i].title} / ${doc.panels[j].title}`);
  }
}
console.log('PASS: V5 Ground retirement, retained map/health panels, stable UID and non-overlapping layout.');
