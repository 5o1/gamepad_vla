#!/usr/bin/env python3
"""Twenty-minute CoACD sweep using installed local CAD environment only."""
import concurrent.futures as futures
import json,os,subprocess,sys,time
from pathlib import Path

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
BASE=ROOT/'.local/fixture-convex/coacd_scan'
THRESHOLDS=[.005,.01,.02,.03,.05]
START=time.monotonic();DEADLINE=START+1180
ENV=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')


def generate(job):
    threshold,part=job;folder=BASE/f't{threshold:g}';folder.mkdir(parents=True,exist_ok=True)
    start=time.monotonic();remaining=DEADLINE-start
    if remaining<180:return dict(threshold=threshold,part=part,status='generation_budget_reserved_for_metrics')
    command=[sys.executable,str(HERE/'convex_collision.py'),'--method','coacd','--threshold',str(threshold),'--part',part,'--output',str(folder),'--mcts-iterations','60','--mcts-nodes','10']
    with (folder/(part+'.log')).open('w') as log:
        try:
            proc=subprocess.run(command,cwd=ROOT,env=ENV,stdout=log,stderr=subprocess.STDOUT,timeout=min(120,remaining-180))
            row=dict(threshold=threshold,part=part,status='complete' if proc.returncode==0 else 'error',returncode=proc.returncode,seconds=time.monotonic()-start)
        except subprocess.TimeoutExpired:row=dict(threshold=threshold,part=part,status='timeout',seconds=time.monotonic()-start)
    meta=folder/f'generation_{part}.json'
    if row['status']=='complete' and meta.exists():
        m=json.loads(meta.read_text());row.update(hulls=m['hulls'],generation_seconds=m['elapsed_seconds'])
    return row


def measure(threshold):
    folder=BASE/f't{threshold:g}';remaining=DEADLINE-time.monotonic()
    if remaining<5:return dict(threshold=threshold,status='metric_budget_exhausted')
    command=[sys.executable,str(HERE/'check_collision.py'),str(folder),'--density','4']
    start=time.monotonic()
    with (folder/'accuracy.log').open('w') as log:
        try:
            proc=subprocess.run(command,cwd=ROOT,env=ENV,stdout=log,stderr=subprocess.STDOUT,timeout=remaining)
            row=dict(threshold=threshold,status='complete' if proc.returncode==0 else 'error',returncode=proc.returncode,metric_seconds=time.monotonic()-start)
        except subprocess.TimeoutExpired:row=dict(threshold=threshold,status='timeout',metric_seconds=time.monotonic()-start)
    if (folder/'accuracy.json').exists():row['accuracy']=json.loads((folder/'accuracy.json').read_text())
    return row


def main():
    import importlib.metadata
    BASE.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((HERE/'out/manifest.json').read_text());parts=[p['name'] for p in manifest['parts'] if not p['name'].startswith('hardware_')]
    result=dict(coacd_version=importlib.metadata.version('coacd'),thresholds=THRESHOLDS,parts=parts,workers=4,threads_per_worker=1,wall_ceiling_seconds=1180,parameters=dict(seed=0,preprocess_mode='off',max_convex_hull=-1,decimate=True,max_ch_vertex=255,resolution=2000,mcts_nodes=10,mcts_iterations=60,mcts_max_depth=3),generation=[],metrics=[])
    # Keep all data inspectable if a difficult native split reaches its timeout.
    with futures.ThreadPoolExecutor(max_workers=4) as pool:
        jobs=[(threshold,part) for threshold in THRESHOLDS for part in parts]
        for future in futures.as_completed([pool.submit(generate,j) for j in jobs]):
            row=future.result();result['generation'].append(row);(BASE/'sweep.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(row),flush=True)
    complete=[t for t in THRESHOLDS if sum(r['threshold']==t and r['status']=='complete' for r in result['generation'])==len(parts)]
    with futures.ThreadPoolExecutor(max_workers=4) as pool:
        for future in futures.as_completed([pool.submit(measure,t) for t in complete]):
            row=future.result();result['metrics'].append(row);(BASE/'sweep.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in row.items() if k!='accuracy'}),flush=True)
    result['elapsed_seconds']=time.monotonic()-START
    (BASE/'sweep.json').write_text(json.dumps(result,indent=2)+'\n')
if __name__=='__main__':main()
