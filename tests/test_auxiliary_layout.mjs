// Dependency-free geometry checks. Firefox integration and screenshots are in AG.
import assert from 'node:assert/strict';
import fs from 'node:fs';
const source=fs.readFileSync(new URL('../src/gamepad_vla/workstation/collection_web/auxiliary-panel.mjs',import.meta.url),'utf8').replace(/^import .*;\n/,'');
const {paneLayout}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
let checked=0;
for(const [width,height] of [[1920,917],[1536,701],[864,1341],[1048,500]])for(let count=0;count<=6;count++){
 const p=paneLayout(width,height,count),[cw,ch]=p.cell;
 assert.equal(p.tiles.length,count);
 assert.deepEqual(p.main,[0,0,2*cw,2*ch]);
 const rects=[p.main,...p.tiles,...p.info];
 for(const [i,[x,y,w,h]] of rects.entries()){
  assert.ok(x>=0&&y>=0&&x+w<=width+1e-8&&y+h<=height+1e-8);
  assert.ok(Math.abs(w/h-16/9)<1e-8);
  if(i)assert.deepEqual([w,h],[cw,ch]);
  for(const [xx,yy,ww,hh] of rects.slice(i+1))assert.ok(Math.min(x+w,xx+ww)-Math.max(x,xx)<1e-8||Math.min(y+h,yy+hh)-Math.max(y,yy)<1e-8);
 }
 const expected=[];
 for(let y=0;y<p.rows;y++)for(let x=0;x<p.columns;x++)if(x>=2||y>=2)expected.push([x*cw,y*ch,cw,ch]);
 assert.deepEqual([...p.tiles,...p.info],expected.slice(0,count+p.info.length),'Cameras compact before information and trailing empty cells');
 for(let c=2;c<=6;c++)assert.ok(cw+1e-8>=Math.min(width/c,height/Math.max(2,Math.ceil((4+count)/c))*16/9),'Largest feasible cell');
 checked++;
}
for(const [width,height,cell] of [[1920,917,[480,270]],[1536,701,[384,216]]]){
 const p=paneLayout(width,height,6);assert.deepEqual([p.columns,p.rows],[4,3]);assert.deepEqual(p.cell,cell);assert.equal(p.info.length,2);
}
console.log(`${checked} layouts: equal 16:9 cells, main 2x2, compact cameras, largest cell, in bounds`);
