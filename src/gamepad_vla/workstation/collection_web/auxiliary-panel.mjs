import {operatorVideo} from '/web/video.mjs';

const views=[['face','Button face'],['front','Approach'],['back','Back'],['left','Left side'],['right','Right side'],['overview','Overview']];

export function auxiliaryPanel(root,status,toolbar) {
  const panes=new Map();let session=null;
  const video=operatorVideo(document.createElement('canvas'),{
    path:'/aux-video',onStatus:value=>status.textContent=value,
    drawFrame(frame,meta){
      root.style.width='max-content';
      for(const [name,pane] of panes){
        const t=meta.tiles[name];
        if(pane.canvas.width!==t.width||pane.canvas.height!==t.height){pane.canvas.width=t.width;pane.canvas.height=t.height;}
        if(!pane.check.checked||!meta.views.includes(name))continue;
        // Each source tile and destination canvas have the same native dimensions.
        pane.canvas.getContext('2d',{alpha:false}).drawImage(frame,t.x,t.y,t.width,t.height,0,0,t.width,t.height);
      }
    }
  });
  function update(){
    const names=[];
    for(const [name,pane] of panes){
      pane.title.textContent=pane.label;
      pane.element.hidden=!pane.check.checked;
      if(pane.check.checked)names.push(name);
      else{const ctx=pane.canvas.getContext('2d',{alpha:false});ctx.fillStyle='#000';ctx.fillRect(0,0,pane.canvas.width,pane.canvas.height);}
    }
    root.hidden=names.length===0;
    root.style.width='max-content';
    root.style.gridTemplateColumns=`repeat(${Math.max(1,Math.min(3,names.length))},max-content)`;
    video.send({views:names});
    if(session&&names.length&&!document.hidden)video.connect(session);else video.stop();
  }
  for(const [name,labelText] of views){
    const pane=document.createElement('section');pane.className='aux-pane video-pane';pane.dataset.view=name;
    const label=document.createElement('label'),check=document.createElement('input');check.type='checkbox';check.checked=name==='face'||name==='overview';check.setAttribute('aria-label',labelText+' view');
    label.append(check,document.createTextNode(labelText));toolbar.append(label);
    const title=document.createElement('span');title.className='pane-label';title.textContent=labelText;
    const canvas=document.createElement('canvas');canvas.width=640;canvas.height=540;canvas.setAttribute('aria-label',labelText+' simulation view');
    pane.append(canvas,title);root.append(pane);panes.set(name,{check,canvas,title,label:labelText,element:pane});check.onchange=update;
  }
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
