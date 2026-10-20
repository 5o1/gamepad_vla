// Node 22 built-ins + an already installed Chrome, no npm dependencies.
// Start Chrome with --remote-debugging-port=19222; use a forwarded localhost URL.
import fs from 'node:fs';
import {fileURLToPath} from 'node:url';
import path from 'node:path';
const configDir=path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../configs');
const settings=JSON.parse(fs.readFileSync(process.env.GAMEPAD_VLA_SITE_CONFIG || (fs.existsSync(path.join(configDir,'site.local.json')) ? path.join(configDir,'site.local.json') : path.join(configDir,'site.example.json')), 'utf8'));
const operator={...JSON.parse(fs.readFileSync(path.join(configDir,'site.example.json'),'utf8')).operator,...settings.operator};
const [url=operator.browser_url,seconds='20',output='/tmp/operator-browser.json']=process.argv.slice(2);
const targets=await(await fetch(operator.debug_url+'/json/list')).json();
const target=targets.find(t=>t.type==='page');
const ws=new WebSocket(target.webSocketDebuggerUrl);
await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
let sequence=0;const pending=new Map();
ws.onmessage=e=>{const m=JSON.parse(e.data);if(m.id){const p=pending.get(m.id);pending.delete(m.id);m.error?p.reject(m.error):p.resolve(m.result);}};
function call(method,params={}){return new Promise((resolve,reject)=>{const id=++sequence;pending.set(id,{resolve,reject});ws.send(JSON.stringify({id,method,params}));});}
async function evaluate(expression){const r=await call('Runtime.evaluate',{expression,returnByValue:true});if(r.exceptionDetails)throw new Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
await call('Page.enable');
await call('Page.navigate',{url});
const samples=[];let ready=false;
for(let i=0;i<120;i++){
  await new Promise(r=>setTimeout(r,1000));
  const s=await evaluate('window.operatorStats ?? null');
  if(s?.presented>30){ready=true;break;}
}
if(!ready)throw new Error('No decoded frames after 120 seconds');
const start=await evaluate('({now:performance.now(),stats:operatorStats})');
for(let i=0;i<Number(seconds);i++){
  await new Promise(r=>setTimeout(r,1000));
  const s=await evaluate('({now:performance.now(),stats:operatorStats})');
  samples.push(s);
  fs.writeFileSync(output,JSON.stringify({url,start,samples},null,2));
  if((i+1)%30===0)console.log(JSON.stringify({elapsed_s:i+1,received:s.stats.received,presented:s.stats.presented,errors:s.stats.errors}));
}
console.log(JSON.stringify({output,samples:samples.length,last:samples.at(-1)}));
ws.close();
