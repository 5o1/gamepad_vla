"""Extract three matched encoded frames (approach/pressed/released), on the node."""
import argparse
import json
from pathlib import Path
import subprocess

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('run',type=Path)
a=p.parse_args()
if not (a.run/'stream_timing.jsonl').is_file() or not (a.run/'operator.h264').is_file():
    p.error('Start the capture session with --diagnostics; production sessions do not save timing or video.')
frames=[json.loads(line) for line in (a.run/'stream_timing.jsonl').read_text().splitlines()]
selected={}
for label,remainder in (('approach',1),('pressed',2),('released',0)):
    index,frame=next((i,f) for i,f in enumerate(frames) if f['command_id']>=10 and
        f['command_id']%3==remainder and f['capture_ns']-f['command_applied_ns']>=800_000_000)
    path=a.run/f'{label}.jpg'
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-i',str(a.run/'operator.h264'),
        '-vf',f'select=eq(n\\,{index})','-frames:v','1','-y',str(path)],check=True)
    selected[label]=dict(encoded_index=index,file=str(path),**frame)
(a.run/'screenshots.json').write_text(json.dumps(selected,indent=2)+'\n')
print(json.dumps({k:{'encoded_index':v['encoded_index'],'command_id':v['command_id'],'step':v['step'],'file':v['file']} for k,v in selected.items()},indent=2))
