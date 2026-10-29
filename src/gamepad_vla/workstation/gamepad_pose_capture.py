"""One authorized station-local camera capture through the existing camera_service.

Never starts arm/hand nodes. Refuses occupied cameras and never touches 8780.
"""
import os
from pathlib import Path
import shlex
import signal
import subprocess
import time
from urllib.request import build_opener, ProxyHandler, Request

from gamepad_vla.site import repository_root, site_value
from .camera_panel import camera_owners, free_port


def capture_snapshot(output, timeout=45, *, count=1, interval=.5):
    if not 1 <= count <= 30:
        raise ValueError('capture count must be 1..30')
    output = Path(output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    serial = site_value('camera.camera_3') or site_value('camera.serials.camera_3')
    if not serial:
        raise RuntimeError('Configure camera.serials.camera_3 in the station site configuration')
    owners = camera_owners(serial)
    if owners:
        raise RuntimeError(f'camera_3 is occupied; owner processes left untouched: {owners}')
    port = free_port()
    command = [site_value('station.python', '/usr/bin/python3'), '-u', '-m',
               'gamepad_vla.workstation.camera_service', '--serve-camera', '--serial', serial,
               '--width', '1280', '--height', '720', '--remote-port', str(port)]
    env = os.environ.copy()
    env.pop('PYTHONHOME', None)
    env.update(ROS_DOMAIN_ID='89', ROS_LOCALHOST_ONLY='1',
               PYTHONPATH=str(repository_root()/'src'),
               ROS_LOG_DIR=str(output.parent/'camera-ros-log'),
               ROS_HOME=str(output.parent/'camera-ros-home'))
    shell = 'source '+shlex.quote(site_value('station.ros_setup', '/opt/ros/humble/setup.bash'))+' && exec '+shlex.join(command)
    opener = build_opener(ProxyHandler({}))
    child = subprocess.Popen(['bash', '-c', shell], env=env, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        deadline = time.monotonic()+timeout
        last_error = None
        while child.poll() is None and time.monotonic() < deadline:
            try:
                with opener.open(Request(f'http://127.0.0.1:{port}/snapshot', data=b'{}'), timeout=2) as r:
                    raw = r.read()
                if count == 1:
                    output.write_bytes(raw)
                    return output
                paths=[]
                for index in range(count):
                    if index:
                        time.sleep(interval)
                        with opener.open(Request(f'http://127.0.0.1:{port}/snapshot', data=b'{}'), timeout=2) as response:
                            raw=response.read()
                    path=output.with_name(output.stem+f'-{index+1:02d}'+output.suffix)
                    path.write_bytes(raw);paths.append(path)
                return paths
            except OSError as error:
                last_error = error
                time.sleep(.2)
        raise RuntimeError(f'camera_service did not produce a fresh frame; camera was not taken over: {last_error}')
    finally:
        if child.poll() is None:
            try:
                with opener.open(Request(f'http://127.0.0.1:{port}/stop', data=b'{}'), timeout=2) as r:
                    r.read()
            except OSError:
                child.send_signal(signal.SIGTERM)
            try:
                child.wait(timeout=17)
            except subprocess.TimeoutExpired:
                # Only the process group created above belongs to this capture.
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
