"""Short isolated camera_3 mmap/NVENC measurement through OperatorView."""
import argparse,json,time,urllib.request
from pathlib import Path
from dataclasses import asdict
import numpy as np
from gamepad_vla.workstation.collection_sapien import CollectionSapienBackend
from gamepad_vla.workstation.operator_view import OperatorView
from gamepad_vla.workstation.spec import load_spec


def main(a):
    spec=load_spec(a.spec);work=a.output;work.mkdir(parents=True,exist_ok=True)
    b=CollectionSapienBackend(spec,work_dir=work/'physics',environment_collisions=False,gamepad_collisions=True)
    view=OperatorView(b,'configs/render/operator_view.yaml',work/'stream',a.spec,port=a.port,collection=True,resources=a.resources)
    samples=[];start=time.monotonic();index=0
    try:
        # Publish completed neutral simulation states at the collection cadence.
        # No ROS subscriptions, hardware, recording, or input command source.
        while time.monotonic()-start<a.seconds+15:
            b.step(.004);row=asdict(b.observe());row={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in row.items()};row['applied_monotonic_ns']=time.monotonic_ns();view.publish(row)
            if (work/'stream/first_frame').exists():break
            time.sleep(.02)
        else:raise TimeoutError('first encoded frame')
        begin=time.monotonic();deadline=begin
        while time.monotonic()-begin<a.seconds:
            b.step(.004);row=asdict(b.observe());row={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in row.items()};row['applied_monotonic_ns']=time.monotonic_ns();view.publish(row)
            if index%30==0:
                try:
                    with urllib.request.urlopen(f'http://127.0.0.1:{a.port}/state',timeout=2) as r:samples.append(json.load(r))
                except Exception as e:samples.append(dict(error=str(e)))
            index+=1;deadline+=1/30;time.sleep(max(0,deadline-time.monotonic()))
        result=dict(spec=str(a.spec),duration_seconds=time.monotonic()-begin,published=index,samples=samples,pipeline='existing operator_view -> camera_3 -> mmap -> NVENC -> isolated loopback gateway',port=a.port)
        (work/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    finally:view.close(cancelled=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--spec',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--resources',required=True);p.add_argument('--port',type=int,default=18980);p.add_argument('--seconds',type=float,default=10);main(p.parse_args())
