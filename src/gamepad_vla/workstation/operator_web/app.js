// Adapted from xbox_gamepad_physics teleop/web/app.js: same packet + WebCodecs path.
// Copyright (c) 2026 5o1. MIT; see THIRD_PARTY_NOTICES.md.
const canvas=document.querySelector('#video'),ctx=canvas.getContext('2d',{alpha:false});
const status=document.querySelector('#status'),stats={received:0,presented:0,errors:[],last:null};
window.operatorStats=stats;
let configured=false,latest=null,lastMetadata=null,batch=[],recent=[];
const metadata=new Map();
if(!window.VideoDecoder)status.textContent='浏览器缺少 WebCodecs，请使用支持 H.264 的 Chrome / Edge（localhost）';
const decoder=new VideoDecoder({output(frame){if(latest){metadata.delete(latest.timestamp);latest.close();}latest=frame;},error(error){stats.errors.push(String(error));status.textContent=String(error);}});
function present(){
  if(latest){const frame=latest;latest=null;const m=metadata.get(frame.timestamp);metadata.delete(frame.timestamp);
    if(m && (canvas.width!==m.width || canvas.height!==m.height)){canvas.width=m.width;canvas.height=m.height;}
    ctx.drawImage(frame,0,0,canvas.width,canvas.height);frame.close();
    if(m){const now=performance.now();stats.presented++;stats.last=m;recent.push(now);recent=recent.filter(t=>now-t<1000);
      document.querySelector('#fps').textContent=recent.length+' fps';
      document.querySelector('#signal').textContent='方向键 '+m.signals.slice(17,21).map(x=>x.toFixed(2)).join(' / ');
      batch.push({frame_id:m.frame_id,step:m.step,command_id:m.command_id,encoded_ns:m.encoded_ns,
        received_ms:m.received_ms,presented_ms:now,server_clock_estimate_ns:m.clock_offset_ns+now*1e6,clock_rtt_ms:m.clock_rtt_ms});
    }
  }requestAnimationFrame(present);
}
requestAnimationFrame(present);
let offset=0,rtt=null;
async function probe(){const start=performance.now(),m=await(await fetch('/clock')).json(),end=performance.now();offset=m.server_ns-(start+end)/2*1e6;rtt=end-start;}
probe().catch(e=>stats.errors.push(String(e)));
setInterval(()=>probe().catch(e=>stats.errors.push(String(e))),10000);
const ws=new WebSocket(`ws://${location.host}/video`);ws.binaryType='arraybuffer';
ws.onmessage=e=>{const bytes=new Uint8Array(e.data),length=new DataView(e.data).getUint32(0),m=JSON.parse(new TextDecoder().decode(bytes.subarray(4,4+length))),data=bytes.subarray(4+length);
  stats.received++;m.received_ms=performance.now();m.clock_offset_ns=offset;m.clock_rtt_ms=rtt;
  if(!configured){if(!m.key||!m.codec)return;decoder.configure({codec:m.codec,optimizeForLatency:true});configured=true;status.textContent='实时画面';}
  const timestamp=m.frame_id*16667;metadata.set(timestamp,m);
  decoder.decode(new EncodedVideoChunk({type:m.key?'key':'delta',timestamp,data}));
};
ws.onclose=()=>status.textContent='已断开';
setInterval(()=>{if(!batch.length)return;const rows=batch;batch=[];fetch('/telemetry',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({time_origin_ms:performance.timeOrigin,frames:rows})}).catch(e=>stats.errors.push(String(e)));},1000);
