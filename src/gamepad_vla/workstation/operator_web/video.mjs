// Shared operator_view WebCodecs client (teleop packet layout). MIT, see THIRD_PARTY_NOTICES.md.
export function operatorVideo(canvas, {onFrame=()=>{},onStatus=()=>{}}={}) {
  const ctx=canvas.getContext('2d',{alpha:false});
  const stats={received:0,presented:0,errors:[],last:null};
  let ws=null,decoder=null,latest=null,metadata=new Map(),timer=null,wanted=null,configured=false;
  function dispose(){if(timer)clearTimeout(timer);timer=null;if(ws){ws.onclose=null;ws.close();ws=null;}if(latest){latest.close();latest=null;}if(decoder&&decoder.state!=='closed')decoder.close();decoder=null;metadata.clear();configured=false;}
  function open(){
    dispose();if(!wanted)return;
    if(!window.VideoDecoder){onStatus('H.264 WebCodecs unavailable. Use Chrome or Edge.');return;}
    decoder=new VideoDecoder({output(frame){if(latest){metadata.delete(latest.timestamp);latest.close();}latest=frame;},error(e){stats.errors.push(String(e));onStatus(String(e));}});
    ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/video?session=${encodeURIComponent(wanted)}`);ws.binaryType='arraybuffer';
    ws.onmessage=e=>{try{const bytes=new Uint8Array(e.data),length=new DataView(e.data).getUint32(0),m=JSON.parse(new TextDecoder().decode(bytes.subarray(4,4+length))),data=bytes.subarray(4+length);stats.received++;
      if(!configured){if(!m.key||!m.codec)return;decoder.configure({codec:m.codec,optimizeForLatency:true});configured=true;onStatus('Live');}
      const timestamp=m.frame_id*16667;metadata.set(timestamp,m);decoder.decode(new EncodedVideoChunk({type:m.key?'key':'delta',timestamp,data}));
    }catch(e){stats.errors.push(String(e));onStatus(String(e));}};
    ws.onclose=()=>{onStatus('Video reconnecting');if(wanted)timer=setTimeout(open,500);};
  }
  function present(){if(latest){const f=latest;latest=null;const m=metadata.get(f.timestamp);metadata.delete(f.timestamp);if(m){canvas.width=m.width;canvas.height=m.height;}ctx.drawImage(f,0,0,canvas.width,canvas.height);f.close();if(m){stats.presented++;stats.last=m;onFrame(m);}}requestAnimationFrame(present);}
  requestAnimationFrame(present);
  return {stats,connect(id){if(id===wanted)return;wanted=id;open();},stop(){wanted=null;dispose();onStatus('Video stopped');}};
}
