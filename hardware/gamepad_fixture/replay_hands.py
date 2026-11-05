"""Bounded CPU-only cooked SAPIEN replay on the configured compute workspace."""
import io,json,shlex,subprocess,tarfile,time
from pathlib import Path
from fixture import ROOT

def replay(out,spec):
    site=json.loads((ROOT/'configs/site.local.json').read_text());c=site['compute'];workspace=site['workspace']
    remote=workspace+'/runs/gamepad-fixture-af-'+str(time.time_ns());code=remote+'/code'
    ssh=['bash',c['ssh_profile'],'--wohint']
    # Copy production code/config only to an isolated run; never mutate the shared checkout.
    command=f'mkdir -p {shlex.quote(code)}; cp -a {shlex.quote(c["remote_code"]+"/src")} {shlex.quote(c["remote_code"]+"/configs")} {shlex.quote(code)}/'
    subprocess.run(ssh+[command],check=True)
    report=ROOT/'docs/reports/2026-11-05-gamepad-fixture-v3_1';files=[spec,report/'clearance_replay.py',report/'hand_states.json',report/'mujoco_smoke.py']+list((out/'simulation/runtime').glob('*'))+list((ROOT/'src').rglob('*.py'))
    buffer=io.BytesIO()
    with tarfile.open(fileobj=buffer,mode='w:gz') as tar:
        for path in files:tar.add(path,arcname=str(path.resolve().relative_to(ROOT)))
    subprocess.run(ssh+[f'tar xzf - -C {shlex.quote(code)}'],input=buffer.getvalue(),check=True)
    script='docs/reports/2026-11-05-gamepad-fixture-v3_1/clearance_replay.py'
    buffer=io.BytesIO()
    with tarfile.open(fileobj=buffer,mode='w:gz') as tar:
        for path in (out/'simulation/runtime').glob('*'):
            tar.add(path,arcname='assets/derived/gamepad_fixture_'+out.name+'/'+path.name)
    subprocess.run(ssh+[f'tar xzf - -C {shlex.quote(workspace)}'],input=buffer.getvalue(),check=True)
    command=f'cd {shlex.quote(code)} && env GAMEPAD_VLA_WORKSPACE={shlex.quote(workspace)} PYTHONPATH={shlex.quote(code+"/src")} XBOX_GAMEPAD_PHYSICS_RESOURCES={shlex.quote(workspace+"/assets/xbox_gamepad_physics_v0.4.1/resources")} CUDA_VISIBLE_DEVICES= GAMEPAD_VLA_RENDER_DEVICE= VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.x86_64.json OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 timeout 3000 nice -n 10 {shlex.quote(c["python"])} {script} --spec {shlex.quote(str(spec.resolve().relative_to(ROOT)))} --out {shlex.quote(remote+"/clearance")}'
    subprocess.run(ssh+[command],check=True)
    smoke=command.split('timeout 3000')[0]+'timeout 300 nice -n 10 '+shlex.quote(c.get('mujoco_python',workspace+'/envs/fixed-gamepad-mujoco-py311/bin/python'))+' docs/reports/2026-11-05-gamepad-fixture-v3_1/mujoco_smoke.py --spec '+shlex.quote(str(spec.resolve().relative_to(ROOT)))+' --out '+shlex.quote(remote+'/clearance')
    subprocess.run(ssh+[smoke],check=True)
    (out/'mujoco_smoke.json').write_bytes(subprocess.check_output(ssh+['cat '+shlex.quote(remote+'/clearance/mujoco_smoke.json')]))
    result=subprocess.check_output(ssh+['cat '+shlex.quote(remote+'/clearance/clearance.json')]);data=json.loads(result)
    accepted=[r for r in data['states'] if r['mode']!='rejected'];controls=[r for r in accepted if r['control']!='hold']
    data['pass']=len(controls)==34 and len(accepted)==38 and all(r['minimum_distance_mm']>0 and not r['contacts'] for r in accepted)
    data['input_mesh_sha256']=json.loads((out/'simulation/runtime/provenance.json').read_text())['asset_sha256']
    (out/'hand_clearance.json').write_text(json.dumps(data,indent=2)+'\n')
    if not data['pass']:raise RuntimeError('34-state hand clearance gate failed')
    return data
if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--spec',type=Path,required=True);a=p.parse_args();replay(a.out,a.spec)
