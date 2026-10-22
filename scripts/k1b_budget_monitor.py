#!/usr/bin/env python3
"""Enforce the one-run wall budget and report CPU use of all owned children."""
import argparse,json,os,re,signal,time
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--stop-file',type=Path)
a=p.parse_args();root=a.root;budget=json.loads((root/'budget.json').read_text());hz=os.sysconf('SC_CLK_TCK')
peaks={};started=time.time();threshold=budget.get('budget_cpu_seconds',768*3600)-200
while True:
 completed={}
 for f in root.rglob('*time*.txt'):
  text=f.read_text();u=re.search(r'User time \(seconds\):\s*([\d.]+)',text);s=re.search(r'System time \(seconds\):\s*([\d.]+)',text)
  if u and s:completed[str(f)]=float(u[1])+float(s[1])
 processes={};jobs={}
 for d in Path('/proc').iterdir():
  if not d.name.isdigit():continue
  try:
   raw=(d/'stat').read_text();f=raw[raw.rfind(')')+2:].split();argv=(d/'cmdline').read_bytes().split(b'\0')
   # Include reaped child CPU at each live parent; a live child is not yet
   # included in that parent's cutime/cstime, so summing the live tree is valid.
   processes[int(d.name)]=(int(f[1]),sum(int(f[i]) for i in (11,12,13,14))/hz,argv)
   if argv and Path(argv[0].decode(errors='ignore')).name=='time' and b'-o' in argv:
    output=argv[argv.index(b'-o')+1].decode()
    if output.startswith(str(root)+'/'):jobs[int(d.name)]=output
  except (OSError,ValueError,IndexError):pass
 active=set()
 for pid,output in jobs.items():
  family={pid}
  while True:
   children={i for i,(ppid,_,_) in processes.items() if ppid in family}
   if children<=family:break
   family|=children
  active|=family
  amount=sum(processes[i][1] for i in family if i in processes)
  peaks[output]=max(peaks.get(output,0),amount)
 # Completed /usr/bin/time captures descendants even if they lived <1 sample.
 # For live jobs use the cumulative live-tree high-water mark. Never add a
 # finished job's process sample a second time on top of its time-file total.
 total=60.+budget.get('legacy_aborted_cpu_upper_bound_seconds',0)+budget.get('previous_monitor_cpu_seconds',0)+sum(max(completed.get(k,0),peaks.get(k,0)) for k in completed.keys()|peaks.keys())
 own=processes.get(os.getpid());total+=own[1] if own else 0
 wall=time.time()-budget['start_unix']
 status=dict(cpu_seconds_conservative=total,cpu_limit_seconds=budget['budget_cpu_seconds'],stop_threshold_seconds=threshold,
    wall_seconds=wall,wall_limit_seconds=43200,deadline_unix=budget['start_unix']+43200,max_core_limit=64,actual_affinity_core_allocation=budget.get('all_task_cpus',budget.get('proposal_cpus',budget['main_cpus'])),
    completed_jobs={str(Path(k).relative_to(root)):v for k,v in completed.items()},active_owned_pids=sorted(active),
    accounting='GNU time completed trees + cumulative live parent/reaped-child CPU; 60 CPU seconds overhead allowance',monitor_seconds=time.time()-started)
 (root/'budget-status.json').write_text(json.dumps(status,indent=2)+'\n')
 # Reviewer clarification: 64 concurrent CPUs for 12 wall hours, not 12 CPU hours.
 # Reserve ten minutes to collect results and write the requested report.
 reason='CPU budget' if total>=threshold else 'wall budget: ten-minute reporting reserve' if wall>=43200-600 else None
 if reason:
  status['stop_reason']=reason;(root/'budget-stop.json').write_text(json.dumps(status,indent=2)+'\n')
  for pid in sorted(active,reverse=True):
   try:os.kill(pid,signal.SIGTERM)
   except ProcessLookupError:pass
  break
 if a.stop_file and a.stop_file.exists():break
 time.sleep(5)
