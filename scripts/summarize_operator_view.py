"""Summarize a fixed server window, with command visibility and frame denominators."""
import argparse
import json
from pathlib import Path
import numpy as np


def read(path):
    if not Path(path).is_file():
        raise ValueError(f'{path} is absent. Start the benchmark session with --diagnostics; production sessions do not capture timing.')
    return [json.loads(line) for line in path.read_text().splitlines()]


def distribution(values):
    return None if not values else dict(count=len(values),p50=float(np.percentile(values,50)),
        p95=float(np.percentile(values,95)),p99=float(np.percentile(values,99)),max=float(max(values)))


def summarize(run, seconds=600, warmup=10):
    all_frames = read(run/'stream_timing.jsonl')
    start = all_frames[0]['encoded_ns']+warmup*1e9
    end = start+seconds*1e9
    frames = [f for f in all_frames if start<=f['encoded_ns']<end]
    bins = np.histogram([f['encoded_ns']/1e9 for f in frames],bins=np.linspace(start/1e9,end/1e9,seconds+1))[0]
    physics = read(run/'physics_timing.jsonl')
    ticks = [r for r in physics if r['kind']=='tick' and start<=r['published_ns']<end]
    commands = [r for r in physics if r['kind']=='command' and start<=r['applied_ns']<end]
    first = {}
    for f in all_frames:first.setdefault(f['command_id'],f)
    delays = [(first[c['command_id']]['encoded_ns']-c['applied_ns'])/1e6 for c in commands if c['command_id'] in first]
    browser = read(run/'browser_timing.jsonl')
    displayed = [f for row in browser for f in row['browser']['frames'] if start<=f['encoded_ns']<end]
    first_displayed = {}
    for f in displayed:
        first_displayed.setdefault(f['command_id'],f)
    displayed_command_delays = [(first_displayed[c['command_id']]['server_clock_estimate_ns']-c['applied_ns'])/1e6
        for c in commands if c['command_id'] in first_displayed and first_displayed[c['command_id']].get('clock_rtt_ms') is not None]
    result = dict(run=str(run),scope='server applied -> encoded, not end-to-end display latency',
        window_seconds=seconds,warmup_seconds=warmup,start_ns=start,end_ns=end,
        full_window=bool(all_frames[-1]['encoded_ns']>=end),frames=len(frames),
        average_encoded_fps=len(frames)/seconds,min_one_second_encoded_fps=int(bins.min()),
        one_second_encoded_fps=bins.tolist(),unique_physics_steps=len({f['step'] for f in frames}),
        state_applied_to_encoded_ms=distribution([(f['encoded_ns']-f['applied_ns'])/1e6 for f in frames]),
        commands_applied=len(commands),commands_visible=len(delays),commands_unseen=len(commands)-len(delays),
        first_command_frame_to_encoded_ms=distribution(delays),
        encode_pipeline_ms=distribution([f['encode_ns']/1e6 for f in frames]),
        physics_ticks=len(ticks),physics_interval_ms=distribution([r['interval_ns']/1e6 for r in ticks if r['interval_ns'] is not None]),
        publish_ms=distribution([r['publish_ns']/1e6 for r in ticks]),
        browser_presented=len(displayed),
        approximate_command_applied_to_first_display_ms=distribution(displayed_command_delays),
        browser_decode_and_present_ms=distribution([f['presented_ms']-f['received_ms'] for f in displayed]),
        browser_clock_rtt_ms=distribution([f['clock_rtt_ms'] for f in displayed if f.get('clock_rtt_ms') is not None]),
        approximate_encoded_to_display_ms=distribution([(f['server_clock_estimate_ns']-f['encoded_ns'])/1e6 for f in displayed if f.get('clock_rtt_ms') is not None]),
        browser_clock_note='Midpoint HTTP clock probe assumes symmetric path; estimate uncertainty at least RTT/2; not a calibrated one-way measurement.')
    if displayed:
        ordered=sorted(displayed,key=lambda f:f['presented_ms'])
        local_start=ordered[0]['presented_ms']
        # Complete browser-local seconds only; server clock and browser clock differ.
        complete=int((ordered[-1]['presented_ms']-local_start)//1000)
        counts=np.histogram([f['presented_ms'] for f in ordered],bins=local_start+np.arange(complete+1)*1000)[0]
        result['browser_complete_one_second_windows']=complete
        result['browser_min_one_second_fps']=int(counts.min()) if complete else None
        result['browser_mean_complete_window_fps']=float(counts.mean()) if complete else None
    # A constant offset is insufficient when the two monotonic domains do not
    # share a verified rate. Keep the old calculation as diagnostic evidence.
    batches = [r for r in browser if r['browser']['frames']]
    bx = np.array([r['browser']['frames'][-1]['presented_ms'] for r in batches])
    sy = np.array([r['server_received_ns']/1e6 for r in batches])
    if len(bx)>2:
        slope,intercept = np.polyfit(bx-bx[0],sy-sy[0],1)
        result['clock_mapping_diagnostic'] = dict(batches=len(bx),
            observed_server_ms_per_browser_ms=float(slope),
            server_span_s=float((sy[-1]-sy[0])/1000),
            browser_span_s=float((bx[-1]-bx[0])/1000),
            affine_residual_p95_ms=float(np.percentile(np.abs(sy-sy[0]-slope*(bx-bx[0])-intercept),95)))
    result['uncorrected_constant_offset_diagnostic'] = {
        'command_applied_to_first_display_ms':result.pop('approximate_command_applied_to_first_display_ms'),
        'encoded_to_display_ms':result.pop('approximate_encoded_to_display_ms')}
    result['cross_clock_latency_validated'] = False
    result['command_applied_to_display_ms'] = None
    result['encoded_to_display_ms'] = None
    result['browser_clock_note'] = ('Browser receive/present and server apply/encode durations are reported in their own clock domains. '
        'HTTP midpoint offset alone has no validated clock-rate mapping; uncorrected diagnostics are not one-way latency measurements.')
    result['passed_server'] = bool(result['full_window'] and bins.min()>=30 and delays and
        result['first_command_frame_to_encoded_ms']['p95']<=150 and result['commands_unseen']==0)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    p.add_argument('--seconds',type=int,default=600)
    p.add_argument('--warmup',type=float,default=10)
    a=p.parse_args()
    result=summarize(a.run,a.seconds,a.warmup)
    (a.run/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='one_second_encoded_fps'},indent=2))
