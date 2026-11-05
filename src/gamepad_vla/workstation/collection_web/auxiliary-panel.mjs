import {operatorVideo} from '/web/video.mjs';

const views=[['face','Button face'],['front','Approach'],['back','Back'],['left','Left side'],['right','Right side'],['overview','Overview']];
const storageKey='collection.auxiliaryViews';

// Main camera occupies the first 2x2 cells; other cameras fill free cells in
// row order. Maximize the common 16:9 cell, then prefer fewer trailing cells.
export function paneLayout(width,height,count){
  let best;
  for(let columns=2;columns<=6;columns++){
    const rows=Math.max(2,Math.ceil((4+count)/columns));
    const w=Math.min(width/columns,height/rows*16/9),h=w*9/16;
    if(best&&(w<best.cell[0]-1e-6||(Math.abs(w-best.cell[0])<1e-6&&columns*rows>=best.columns*best.rows)))continue;
    const free=[];
    for(let y=0;y<rows;y++)for(let x=0;x<columns;x++)if(x>=2||y>=2)free.push([x*w,y*h,w,h]);
    best={columns,rows,cell:[w,h],main:[0,0,2*w,2*h],tiles:free.slice(0,count),info:free.slice(count,count+2)};
  }
  return best;
}

export function auxiliaryPanel(root,status,toolbar) {
  const panes=new Map();let session=null;
  let selected=views.map(([name])=>name);
  try{const saved=JSON.parse(localStorage.getItem(storageKey));if(Array.isArray(saved))selected=saved;}catch{}
  const workspace=root.parentElement,main=workspace.querySelector('.view');
  function place(element,[x,y,w,h]){Object.assign(element.style,{left:x+'px',top:y+'px',width:w+'px',height:h+'px'});}
  function layout(){
    const visible=[...panes.values()].filter(p=>p.check.checked);
    const plan=paneLayout(workspace.clientWidth,workspace.clientHeight,visible.length);
    place(main,plan.main);
    visible.forEach((p,i)=>place(p.element,plan.tiles[i]));
    workspace.querySelectorAll('.info-pane').forEach((pane,i)=>{pane.hidden=!plan.info[i];if(plan.info[i])place(pane,plan.info[i]);});
    workspace.dataset.layout=`${plan.columns}x${plan.rows}`;
  }
  const video=operatorVideo(document.createElement('canvas'),{
    path:'/aux-video',onStatus:value=>status.textContent=value,
    drawFrame(frame,meta){
      for(const [name,pane] of panes){
        const t=meta.tiles[name];
        if(pane.canvas.width!==t.width||pane.canvas.height!==t.height){pane.canvas.width=t.width;pane.canvas.height=t.height;}
        if(!pane.check.checked||!meta.views.includes(name))continue;
        pane.canvas.getContext('2d',{alpha:false}).drawImage(frame,t.x,t.y,t.width,t.height,0,0,t.width,t.height);
      }
    }
  });
  function update(){
    const names=[];
    for(const [name,pane] of panes){
      pane.element.hidden=!pane.check.checked;
      if(pane.check.checked)names.push(name);
      else{pane.element.classList.remove('enlarged');pane.element.setAttribute('aria-expanded','false');const ctx=pane.canvas.getContext('2d',{alpha:false});ctx.fillStyle='#000';ctx.fillRect(0,0,pane.canvas.width,pane.canvas.height);}
    }
    root.hidden=names.length===0;
    layout();
    video.send({views:names});
    if(session&&names.length&&!document.hidden)video.connect(session);else video.stop();
  }
  for(const [name,labelText] of views){
    const pane=document.createElement('section');pane.className='aux-pane video-pane';pane.dataset.view=name;
    const label=document.createElement('label'),check=document.createElement('input');check.type='checkbox';check.checked=selected.includes(name);check.setAttribute('aria-label',labelText+' view');
    label.append(check,document.createTextNode(labelText));toolbar.append(label);
    const title=document.createElement('span');title.className='pane-label';title.textContent=labelText;
    const canvas=document.createElement('canvas');canvas.width=640;canvas.height=540;canvas.setAttribute('aria-label',labelText+' simulation view');
    pane.append(canvas,title);root.append(pane);panes.set(name,{check,canvas,element:pane});check.onchange=()=>{
      try{localStorage.setItem(storageKey,JSON.stringify([...panes].filter(([,p])=>p.check.checked).map(([n])=>n)));}catch{}
      update();
    };
  }
  new ResizeObserver(layout).observe(workspace);
  document.addEventListener('visibilitychange',()=>{if(document.hidden)video.stop();else update();});
  update();
  return {stats:video.stats,setSession(id){session=id;update();},stop(){session=null;video.stop();}};
}

export function installPaneZoom(root){
  const panes=()=>root.querySelectorAll('.video-pane');
  function toggle(pane){const expand=!pane.classList.contains('enlarged');for(const p of panes()){p.classList.remove('enlarged');p.setAttribute('aria-expanded','false');}if(expand){pane.classList.add('enlarged');pane.setAttribute('aria-expanded','true');}}
  for(const pane of panes()){pane.tabIndex=0;pane.setAttribute('role','button');pane.setAttribute('aria-expanded','false');pane.title='Click to enlarge; click again to restore';}
  root.addEventListener('click',event=>{const pane=event.target.closest('.video-pane');if(pane)toggle(pane);});
  root.addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){const pane=event.target.closest('.video-pane');if(pane){event.preventDefault();toggle(pane);}}});
  document.addEventListener('keydown',event=>{if(event.key==='Escape')for(const p of panes()){p.classList.remove('enlarged');p.setAttribute('aria-expanded','false');}});
}
