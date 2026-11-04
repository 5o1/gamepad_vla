"""Export the measured AD layers and omit live device identifiers from public evidence."""
import copy
import csv
import json
import re
from pathlib import Path
import sys

source = json.loads(Path(sys.argv[1]).read_text())
out = Path(__file__).resolve().parent
public = copy.deepcopy(source)
for key in ('module', 'handcore_module'):
    public[key] = re.sub(r'^/home/[^/]+', '~', public[key])
for case in public['cases']:
    for hand in case['hands'].values():
        hand['message'].pop('glove_id', None)
        hand['message'].pop('raw_sensor_orientation', None)
(out/'results.json').write_text(json.dumps(public, indent=2)+'\n')
with (out/'layer_values.csv').open('w', newline='') as stream:
    writer = csv.writer(stream)
    writer.writerow(['input','angle_deg','hand','control','motor','joint','q_raw_rad','q_emit_rad','motor_value','q_sim_rad','delta_emit_rad','delta_raw_rad'])
    for case in source['cases']:
        for side, hand in case['hands'].items():
            for control, value in hand['controls'].items():
                for motor, idx in ((5,0),(6,5),(7,9),(8,13),(9,17),(10,1)):
                    writer.writerow([case['category'],case['angle_deg'],side,control,motor,source['joint_names'][idx],hand['optimizer_q'][idx],value['retarget_q'][idx],value['motor'][motor],value['mapped_q'][idx],value['delta_q'][idx],value['mapped_q'][idx]-hand['optimizer_q'][idx]])
summary = {'cases': len(source['cases']), 'hands_per_case':2, 'controls':['a','b','c'], 'elapsed_s':source['elapsed_s'], 'mirror_residual_m':max(c['mirror_residual_m']for c in source['cases']), 'max_last_optimizer_step_rad':max(h['settle_max_last_step_rad']for c in source['cases']for h in c['hands'].values()), 'left_max_roundtrip_error_rad':{}}
for m,i in ((5,0),(6,5),(7,9),(8,13),(9,17),(10,1)):
    summary['left_max_roundtrip_error_rad'][str(m)]={control:max(abs(c['hands']['left']['controls'][control]['delta_q'][i]) for c in source['cases'])for control in 'abc'}
summary['m10_identical_across_controls']=all(len({(h['controls'][control]['motor'][10],h['controls'][control]['mapped_q'][1])for control in 'abc'})==1 for c in source['cases']for h in c['hands'].values())
summary['recording_idle_before_and_after']=all(source[k]['recording_status']['state']=='idle'for k in ('initial_status','final_status'))
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary, indent=2))
