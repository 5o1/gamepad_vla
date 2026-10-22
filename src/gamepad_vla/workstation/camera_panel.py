"""Own camera children and proxy the existing preview and station ego web APIs."""
import io
import json
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import threading
import time
from datetime import datetime, timezone
from urllib.error import HTTPError, URLError
from urllib.request import build_opener, ProxyHandler, Request
import zipfile

from gamepad_vla.site import repository_root, site_value


class CameraConflict(RuntimeError):
    pass


def camera_owners(serial, proc=Path('/proc'), video_ids=Path('/dev/v4l/by-id'), usb=Path('/sys/bus/usb/devices')):
    """Read process identities and open device FDs; never open or signal a device."""
    devices = {str(p.resolve()) for p in video_ids.glob('*') if serial and serial in p.name}
    for device in usb.glob('*'):
        try:
            if device.joinpath('serial').read_text().strip() == serial:
                bus, number = (int(device.joinpath(k).read_text()) for k in ('busnum', 'devnum'))
                devices.add(f'/dev/bus/usb/{bus:03d}/{number:03d}')
        except (OSError, ValueError):
            continue
    owners = []
    for process in proc.glob('[0-9]*'):
        try:
            argv = process.joinpath('cmdline').read_bytes().decode(errors='replace').rstrip('\0').split('\0')
            if not argv or not argv[0] or int(process.name) == os.getpid():
                continue
            name = Path(argv[0]).name
            # Ignore shell payloads: inspect their real executable children instead.
            candidate = False
            if name not in ('bash', 'sh', 'zsh', 'grep', 'rg'):
                identity = ' '.join(argv[:4])
                candidate = any(s in identity for s in ('realsense2_camera', 'realsense2_camera_node',
                    'ego_realsense_rgbd_stream', 'record_ego_lerobot_direct.py',
                    'gamepad_vla.workstation.camera_service'))
                selected = re.findall(r'(?:serial_no:=_?|--serial[ =])([\w-]+)', ' '.join(argv))
                if selected and serial not in selected:
                    candidate = False
                if name == 'ego_realsense_rgbd_stream' and len(argv)>1 and argv[1] != serial:
                    candidate = False
            holds_device = any(str(p.resolve()) in devices for p in process.joinpath('fd').glob('*')) if devices else False
            if candidate or holds_device:
                owners.append(dict(pid=int(process.name), command=' '.join(argv)))
        except (OSError, ValueError):
            continue
    return owners


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def owned_groups(process):
    """The station ego app creates child sessions; collect those before it exits."""
    parents = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            parents[int(path.parent.name)] = (int(fields[1]), int(fields[2]), fields[0])
        except (OSError, ValueError, IndexError):
            continue
    descendants = {process.pid}
    while True:
        found = {pid for pid, (parent, _, _) in parents.items() if parent in descendants}
        if found <= descendants:
            break
        descendants.update(found)
    return {process.pid} | {parents[pid][1] for pid in descendants if pid in parents and parents[pid][1] in descendants}


def stop_group(process, timeout=15, groups=None):
    """Only signal owned sessions, including ego's separately grouped children."""
    groups = set(groups or ()) | owned_groups(process)
    def alive():
        process.poll()  # Reap our direct child; zombies no longer hold USB devices.
        active = set()
        for path in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields = path.read_text().rsplit(')', 1)[1].split()
                if fields[0] != 'Z' and int(fields[2]) in groups:
                    active.add(int(fields[2]))
            except (OSError, ValueError, IndexError):
                continue
        return active
    for sig, wait in ((signal.SIGTERM, timeout), (signal.SIGKILL, 3)):
        for group in alive():
            try:
                os.killpg(group, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            if not alive():
                process.wait(timeout=3)
                return
            time.sleep(.05)
    raise RuntimeError(f'相机子进程组 {process.pid} 未退出，请查看日志。')


class CameraPanel:
    def __init__(self, data_root=None, *, settings=None, owner_scan=camera_owners):
        self.settings = settings or {}
        self.data_root = Path(data_root or site_value('station.data_root', required=True)).expanduser().resolve()
        self.owner_scan = owner_scan
        self.lock = threading.Lock()
        self.preview = self.ego = None
        self.preview_port = self.ego_port = None
        self.preview_log = self.ego_log = None
        self.mode = 'idle'
        self.resume_preview = False
        self.session = None
        self.last_recording = None
        self.last_recording_report = None
        self.error = None
        self.opener = build_opener(ProxyHandler({}))

    def setting(self, key, default=None):
        return self.settings.get(key, site_value(key, default))

    @property
    def serial(self):
        value = self.setting('camera.camera_3') or self.setting('camera.serials.camera_3')
        if not isinstance(value, str) or not value:
            raise ValueError('请在站点配置填写 camera.camera_3 相机序列号。')
        return value

    def check_available(self):
        owners = self.owner_scan(self.serial)
        if owners:
            raise CameraConflict('相机被其他进程占用，请现场人员处理，未结束这些进程：' +
                                 '；'.join(f"PID {p['pid']} {p['command']}" for p in owners))

    def ensure_session(self):
        if self.session is None:
            self.session = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')
            path = self.data_root/'calibration'/self.session
            if path.exists():
                self.session += '-' + datetime.now(timezone.utc).strftime('%f')
            (self.data_root/'calibration'/self.session).mkdir(parents=True)
        return self.session

    def request(self, port, path, *, post=False, timeout=5):
        if port is None:
            raise RuntimeError('相机服务尚未启动。')
        try:
            with self.opener.open(Request(f'http://127.0.0.1:{port}'+path,
                                         data=b'{}' if post else None,
                                         headers={'Content-Type':'application/json'}), timeout=timeout) as r:
                return r.read()
        except HTTPError as exc:
            detail = exc.read().decode(errors='replace')
            raise RuntimeError(f'相机服务 HTTP {exc.code}：{detail}') from exc
        except (URLError, OSError) as exc:
            raise RuntimeError(f'相机服务不可达：{exc}') from exc

    def ego_request(self, path, *, post=False):
        return json.loads(self.request(self.ego_port, path, post=post, timeout=200 if path in ('/api/record/stop','/api/stop') else 20))

    def launch(self, command, log):
        # Do not inherit a conda Python path into the system Python/ROS worker.
        env = os.environ.copy()
        env.pop('PYTHONHOME', None)
        env['PYTHONPATH'] = str(repository_root()/'src')
        env['ROS_DOMAIN_ID'] = str(self.setting('camera.ros_domain_id', 73))
        if env['ROS_DOMAIN_ID'] == os.environ.get('ROS_DOMAIN_ID'):
            raise ValueError('camera.ros_domain_id 必须与采集仿真 ROS 域不同。')
        env['ROS_LOCALHOST_ONLY'] = '1'
        return subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait_ready(self, process, port, path, timeout=45):
        deadline = time.monotonic() + timeout
        while process.poll() is None:
            try:
                return json.loads(self.request(port, path, timeout=1))
            except RuntimeError:
                if time.monotonic() >= deadline:
                    break
                time.sleep(.15)
        raise RuntimeError('相机服务启动失败或超时，请查看本次会话 camera 日志。')

    def _start_preview(self):
        if self.preview and self.preview.poll() is None:
            return
        self.check_available()
        session = self.ensure_session()
        self.mode = 'starting'
        self.preview_port = free_port()
        path = self.data_root/'calibration'/session
        self.preview_log = (path/'camera.log').open('ab')
        command = [self.setting('station.python', '/usr/bin/python3'), '-u', '-m',
            'gamepad_vla.workstation.camera_service', '--serve-camera', '--serial', self.serial,
            '--width', '1280', '--height', '720', '--remote-port', str(self.preview_port)]
        shell = 'source ' + shlex.quote(self.setting('station.ros_setup', '/opt/ros/humble/setup.bash')) + ' && exec ' + shlex.join(command)
        try:
            self.preview = self.launch(['bash', '-c', shell], self.preview_log)
            self.wait_ready(self.preview, self.preview_port, '/status')
            self.mode, self.error = 'preview', None
        except Exception:
            self._stop_preview()
            raise

    def _stop_preview(self):
        if self.preview:
            groups = owned_groups(self.preview)
            try:
                if self.preview.poll() is None:
                    try:
                        self.request(self.preview_port, '/stop', post=True, timeout=2)
                        self.preview.wait(timeout=15)
                    except (RuntimeError, subprocess.TimeoutExpired):
                        pass
                stop_group(self.preview, groups=groups)
            finally:
                if self.preview.poll() is not None:
                    self.preview = None
        if self.preview_log:
            self.preview_log.close()
            self.preview_log = None
        self.preview_port = None
        self.mode = 'idle'

    def start(self):
        with self.lock:
            if self.ego:
                raise CameraConflict('RGB-D 录像服务正在独占相机，请先停止录像。')
            self._start_preview()
        return self.status()

    def _stop_ego(self):
        if self.ego:
            groups = owned_groups(self.ego)
            try:
                if self.ego.poll() is None:
                    # stop-record restores the upstream ROS camera. Capture its
                    # new session before stop can detach/exit that launcher.
                    if self.ego_request('/api/state').get('recording'):
                        self.last_recording = self.ego_request('/api/record/stop', post=True)
                        self.last_recording_report = self.last_recording.get('recording_report')
                    groups.update(owned_groups(self.ego))
                    value = self.ego_request('/api/stop', post=True)
                    if value.get('last_recording_report') is not None:
                        self.last_recording_report = value['last_recording_report']
            finally:
                stop_group(self.ego, groups=groups)
                self.ego = None
                self.ego_port = None
                self.mode = 'idle'
        if self.ego_log:
            self.ego_log.close()
            self.ego_log = None
        self.mode = 'idle'

    def stop(self):
        with self.lock:
            self.resume_preview = False
            self._stop_ego()
            self._stop_preview()
        return self.status()

    def preview_jpeg(self):
        return self.request(self.preview_port, '/preview.jpg')

    def snapshot(self, label=''):
        if not isinstance(label, str):
            raise ValueError('标签必须是文字。')
        with self.lock:
            if self.mode != 'preview':
                raise CameraConflict('请先启动预览，录像时不能拍标定照片。')
            safe = re.sub(r'[^a-zA-Z0-9_-]', '_', label)[:50] or 'capture'
            name = safe + '_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
            with zipfile.ZipFile(io.BytesIO(self.request(self.preview_port, '/snapshot', post=True))) as archive:
                pixels = archive.read('image.png')
                metadata = json.loads(archive.read('camera.json'))
            metadata.update(label=label, saved_locally_at_utc=datetime.now(timezone.utc).isoformat(), image_file=name+'.png')
            path = self.data_root/'calibration'/self.ensure_session()
            (path/(name+'.png')).write_bytes(pixels)
            (path/(name+'.json')).write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
        return dict(name=name, path=str(path/(name+'.png')), metadata=metadata)

    def photos(self):
        if not self.session:
            return []
        path = self.data_root/'calibration'/self.session
        return [dict(name=p.stem, **json.loads(p.read_text())) for p in sorted(path.glob('*.json'), reverse=True)]

    def photo(self, name):
        if not self.session or Path(name).name != name or Path(name).suffix not in ('.png', '.json'):
            raise FileNotFoundError(name)
        path = self.data_root/'calibration'/self.session/name
        if not path.is_file():
            raise FileNotFoundError(name)
        return path

    def record_start(self):
        with self.lock:
            if self.ego:
                raise CameraConflict('录像已经启动，请先查询状态。')
            script = Path(self.setting('camera.ego_web_script') or '').expanduser()
            if not script.is_file():
                raise ValueError('请在站点配置 camera.ego_web_script 指定已有的 ego 拍照录像网页。')
            self.resume_preview = self.mode == 'preview'
            self._stop_preview()
            self.check_available()
            path = self.data_root/'camera'/self.ensure_session()
            path.mkdir(parents=True, exist_ok=True)
            self.mode = 'starting_recording'
            self.ego_port = free_port()
            self.ego_log = (path/'ego-web.log').open('ab')
            command = [self.setting('station.python', '/usr/bin/python3'), '-u', str(script),
                '--host', '127.0.0.1', '--port', str(self.ego_port), '--no-open',
                '--output-dir', str(path/'photos'), '--video-output-dir', str(path/'videos')]
            try:
                self.ego = self.launch(command, self.ego_log)
                self.wait_ready(self.ego, self.ego_port, '/api/state', timeout=10)
                self.ego_request('/api/start', post=True)
                result = self.ego_request('/api/record/start', post=True)
                if not result.get('recording'):
                    raise RuntimeError('站点 ego 网页没有进入录像状态。')
                self.mode, self.error = 'recording', None
                self.last_recording = result
                return result
            except Exception:
                self._stop_ego()
                if self.resume_preview:
                    self._start_preview()
                raise

    def record_stop(self):
        with self.lock:
            if not self.ego:
                return self.last_recording or {}
            self.mode = 'stopping_recording'
            try:
                result = self.ego_request('/api/record/stop', post=True)
                self.last_recording = result
                self.last_recording_report = result.get('recording_report')
                return result
            finally:
                # The upstream stop-record restores its own ROS camera; stop it too.
                self._stop_ego()
                if self.resume_preview:
                    self._start_preview()

    def record_state(self):
        if self.ego and self.ego.poll() is None:
            return self.ego_request('/api/state')
        return self.last_recording or {}

    def status(self):
        result = dict(mode=self.mode, session=self.session, serial=self.setting('camera.camera_3') or self.setting('camera.serials.camera_3'),
                      width=1280, height=720, age_s=None, fps=0., error=self.error,
                      output_dir=str(self.data_root/'calibration'/self.session) if self.session else None,
                      recording=self.last_recording, recording_report=self.last_recording_report)
        if self.preview and self.mode == 'preview':
            try:
                if self.preview.poll() is not None:
                    raise RuntimeError('预览相机进程已经退出。')
                result.update(json.loads(self.request(self.preview_port, '/status', timeout=1)))
            except RuntimeError as exc:
                result.update(error=str(exc))
        if self.ego and self.ego.poll() is not None:
            result.update(mode='failed', error='站点 ego 网页已经退出，请停止相机并检查日志。')
        return result
