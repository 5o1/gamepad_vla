import {operatorVideo} from './video.mjs';
let recent=[];
const video=operatorVideo(document.querySelector('#video'),{
 onStatus:s=>document.querySelector('#status').textContent=s,
 onFrame:m=>{const now=performance.now();recent=recent.filter(t=>now-t<1000);recent.push(now);document.querySelector('#fps').textContent=recent.length+' fps';document.querySelector('#signal').textContent='方向键 '+m.signals.slice(17,21).map(x=>x.toFixed(2)).join(' / ');}
});
window.operatorStats=video.stats;video.connect('operator');
