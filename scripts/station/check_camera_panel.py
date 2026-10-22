#!/usr/bin/env python3
"""Station-local browser smoke using a separate collection server (no simulation).

Run in the existing browser environment with PLAYWRIGHT_BROWSERS_PATH set.
Default is UI-only. --hardware requires the user's RealSense authorization and
confirmed station camera configuration. Never enables remote control.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.request import build_opener, ProxyHandler

from playwright.sync_api import sync_playwright
from gamepad_vla.site import repository_root, site_value
from gamepad_vla.workstation.camera_panel import camera_owners


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server-python', required=True)
    parser.add_argument('--port', type=int, default=18791)
    parser.add_argument('--baseline-url', default='http://127.0.0.1:8780')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hardware', action='store_true')
    args = parser.parse_args()
    root = repository_root()
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    opener = build_opener(ProxyHandler({}))
    base = f'http://127.0.0.1:{args.port}'
    def get(url):
        with opener.open(url, timeout=3) as r:
            return json.load(r)
    def baseline():
        state = get(args.baseline_url + '/api/v1/status')
        return {k: state.get(k) for k in ('session_id', 'session_state', 'revision', 'sim_time_s', 'remote_control_enabled')}
    result = dict(hardware=args.hardware, baseline_before=baseline())
    server = None
    try:
        if args.hardware:
            serial = site_value('camera.camera_3') or site_value('camera.serials.camera_3')
            if not serial:
                raise RuntimeError('站点尚未配置 camera_3；确认设备后再执行硬件验收。')
            owners = camera_owners(serial)
            if owners:
                result['owners'] = owners
                raise RuntimeError('相机被占用，停止验收，不结束占用进程。')
        env = os.environ.copy()
        env['CUDA_VISIBLE_DEVICES'] = ''  # This instance never starts simulation/rendering.
        with (output/'server.log').open('w') as log:
            server = subprocess.Popen([args.server_python, str(root/'scripts/collection_server.py'),
                '--host', '127.0.0.1', '--port', str(args.port), '--output-root', str(output/'episodes')],
                cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+20
            while True:
                try:
                    get(base+'/api/v1/status')
                    break
                except OSError:
                    if server.poll() is not None or time.monotonic()>deadline:
                        raise RuntimeError('临时服务启动失败。')
                    time.sleep(.1)
            with sync_playwright() as pw:
                browser = pw.chromium.launch(headless=True, args=['--disable-gpu','--disable-dev-shm-usage','--password-store=basic'])
                page = browser.new_page(viewport=dict(width=1400,height=1200))
                errors = []
                page.on('pageerror', lambda e: errors.append(str(e)))
                page.goto(base, wait_until='domcontentloaded')
                page.locator('#cameraPanel').evaluate('(el)=>el.open=true')
                page.wait_for_function("document.querySelector('#cameraMode').textContent==='空闲'")
                page.locator('#cameraGrid').check()
                page.locator('#cameraCross').check()
                if args.hardware:
                    page.locator('#cameraStart').click()
                    page.wait_for_function("document.querySelector('#cameraMode').textContent==='预览中'", timeout=60000)
                    page.wait_for_function("document.querySelector('#cameraPreview').naturalWidth===1280", timeout=15000)
                    start = time.monotonic()
                    frames = page.evaluate('window.cameraPanelMetrics.frames')
                    for _ in range(30):
                        page.wait_for_timeout(1000)
                    elapsed = time.monotonic()-start
                    count = page.evaluate('window.cameraPanelMetrics.frames')-frames
                    result['preview'] = dict(duration_s=elapsed,frames=count,fps=count/elapsed,
                        status=get(base+'/api/v1/camera/status'))
                    assert result['preview']['fps']>=10, result['preview']
                    page.locator('#cameraLabel').fill('q_smoke')
                    page.locator('#cameraSnapshot').click()
                    page.wait_for_function("!document.querySelector('#cameraLastPhoto').hidden", timeout=15000)
                    photos = get(base+'/api/v1/camera/photos')
                    photo = photos[0]
                    folder = Path(result['preview']['status']['output_dir'])
                    png = folder/(photo['name']+'.png')
                    metadata = json.loads(png.with_suffix('.json').read_text())
                    assert png.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
                    expected = json.loads((root/'references/calibration/2026-10-20-ego/measured/intrinsics.json').read_text())['K']
                    expected = [v for row in expected for v in row]
                    result['snapshot'] = dict(path=str(png),json=str(png.with_suffix('.json')),K=metadata['K'],K_matches_calibration=metadata['K']==expected)
                    assert result['snapshot']['K_matches_calibration'], result['snapshot']
                    page.locator('#cameraRecordStart').click()
                    page.wait_for_function("document.querySelector('#cameraMode').textContent==='录像中'", timeout=60000)
                    page.wait_for_timeout(5000)
                    page.locator('#cameraRecordStop').click()
                    page.wait_for_function("document.querySelector('#cameraMode').textContent==='预览中' && !document.querySelector('#cameraStop').disabled", timeout=240000)
                    state = get(base+'/api/v1/camera/status')
                    result['recording'] = state['recording']
                    result['recording_report'] = state['recording_report']
                    result['preview_restored'] = state['mode']=='preview'
                    for key in ('rgb_path','depth_path','depth_metadata_path','frame_log_path','recording_report_path'):
                        path = Path(result['recording'][key])
                        assert path.is_file() and path.stat().st_size>0,(key,path)
                    assert result['recording_report'] is not None
                    page.screenshot(path=str(output/'camera-panel.png'),full_page=True)
                    page.locator('#cameraStop').click()
                    page.wait_for_function("document.querySelector('#cameraMode').textContent==='空闲' && !document.querySelector('#cameraStart').disabled", timeout=60000)
                    result['owners_after_stop'] = camera_owners(serial)
                    assert not result['owners_after_stop']
                else:
                    page.wait_for_timeout(1000)
                    page.screenshot(path=str(output/'camera-panel-idle.png'),full_page=True)
                result['js_errors'] = errors
                assert not errors,errors
                browser.close()
        result['passed'] = True
    except Exception as exc:
        result.update(passed=False,error=str(exc))
    finally:
        if server and server.poll() is None:
            server.send_signal(signal.SIGINT)
            try:
                server.wait(timeout=45)
            except subprocess.TimeoutExpired:
                result.update(passed=False,cleanup_error='临时服务退出超时，请查看日志，保留 PID 供排查。',owned_server_pid=server.pid)
        result['baseline_after'] = baseline()
        before,after = result['baseline_before'],result['baseline_after']
        result['original_session_preserved'] = (before['session_id']==after['session_id'] and before['revision']==after['revision']
            and before['session_state']==after['session_state'] and before['remote_control_enabled']==after['remote_control_enabled'])
        if not result['original_session_preserved']:
            result['passed'] = False
        (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],hardware=args.hardware,error=result.get('error'),output=str(output)),ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    sys.exit(main())
