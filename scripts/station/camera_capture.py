#!/usr/bin/env python3
"""Camera-only ROS preview over SSH, with lossless captures saved locally."""

import argparse
from collections import OrderedDict
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import threading
import time
from urllib.parse import unquote, urlsplit
from urllib.request import build_opener, ProxyHandler, Request
import zipfile


PAGE = """<!doctype html>
<html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LinkerHand 相机 · 标定拍照</title>
<style>
body{margin:0;background:#172126;color:#edf3f5;font:16px system-ui,sans-serif}
main{max-width:1280px;margin:auto;padding:20px}h1{font-size:23px;margin:0 0 8px}
p{color:#bccbd1;line-height:1.6}#view{display:block;width:100%;background:#090d10;min-height:180px}
.bar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:16px 0}
input,button{font:inherit;padding:10px 14px;border-radius:6px;border:0}
button{cursor:pointer;background:#91e4bc;color:#14231c}button:disabled{opacity:.45;cursor:wait}
#stop{background:#d7dfe3}#status,#result{white-space:pre-wrap;overflow-wrap:anywhere}
a{color:#91e4bc}li{margin:10px 0}small{color:#bccbd1}
</style><main>
<h1>LinkerHand 相机 · 标定拍照</h1>
<p>保持相机固定。摆好标定板后点击拍照，PNG 原图与对应内参 JSON 会自动保存到本地。</p>
<div id="status">正在连接相机…</div>
<div class="bar"><input id="label" placeholder="位置备注，例如 position_1" value="position_1">
<button id="shot" disabled>拍照并保存原图</button><button id="stop">停止相机</button></div>
<div id="result"></div><img id="view" alt="实时相机画面">
<p><small>预览为 JPEG；保存的 PNG 保留相机原始分辨率，不缩放、不裁剪、不去畸变。</small></p>
<h2 style="font-size:18px">本次拍摄</h2><ul id="files"></ul></main>
<script>
const $=id=>document.getElementById(id);let running=true,objectURL=null,busy=false;
async function api(path,body){let r=await fetch(path,body===undefined?{}:
 {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
 let d=await r.json();if(!r.ok)throw Error(d.error||r.statusText);return d}
async function preview(){if(!running)return;try{
 let r=await fetch('/preview.jpg?t='+Date.now());if(!r.ok)throw Error('等待相机画面');
 let next=URL.createObjectURL(await r.blob());$('view').src=next;
 if(objectURL)URL.revokeObjectURL(objectURL);objectURL=next;
}catch(e){$('result').textContent=e.message}if(running)setTimeout(preview,140)}
async function status(){if(!running)return;try{let s=await api('/status');
 $('status').textContent=`相机 ${s.serial} · ${s.width} × ${s.height} · 帧龄 ${s.age_s.toFixed(2)} 秒\\n保存目录：${s.output_dir}`;
 $('shot').disabled=busy||s.age_s>2;
 let list=$('files');list.replaceChildren();for(let f of s.captures){
 let li=document.createElement('li');for(let ext of ['png','json']){
 let a=document.createElement('a');a.href='/files/'+encodeURIComponent(f+'.'+ext);
 a.target='_blank';a.textContent=ext==='png'?f+'.png':'  内参 JSON';li.append(a)}
 list.append(li)}
}catch(e){$('shot').disabled=true;$('status').textContent=e.message}
if(running)setTimeout(status,1500)}
$('shot').onclick=async()=>{busy=true;$('shot').disabled=true;
 $('result').textContent='正在保存原图…';try{let d=await api('/capture',{label:$('label').value});
 $('result').textContent='已保存：'+d.path;
}catch(e){$('result').textContent=e.message}finally{busy=false;$('shot').disabled=false}};
$('stop').onclick=async()=>{try{await api('/stop',{});running=false;
 $('shot').disabled=true;$('stop').disabled=true;$('status').textContent='相机已停止，照片已保留。';
}catch(e){$('result').textContent=e.message}};
preview();status();
</script></html>"""


def now():
    return datetime.now(timezone.utc).isoformat()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, data, content_type="application/json", status=200):
        if isinstance(data, dict):
            data = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        self.dispatch("GET")

    def do_POST(self):
        self.dispatch("POST")

    def dispatch(self, method):
        path = unquote(urlsplit(self.path).path)
        try:
            if method == "POST" and path == "/stop":
                self.reply({"stopped": True})
                threading.Thread(target=self.server.shutdown, daemon=True).start()
                return
            if method == "GET" and path == "/" and not self.server.remote:
                self.reply(PAGE.encode(), "text/html; charset=utf-8")
                return
            self.server.route(self, method, path)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as error:
            self.reply({"error": str(error)}, status=503)


def server(port, remote, route):
    result = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    result.daemon_threads = True
    result.remote = remote
    result.route = route
    return result


def camera_main(args):
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    lock = threading.Lock()
    pending = [OrderedDict(), OrderedDict()]
    latest = None

    def receive(message, index):
        nonlocal latest
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        with lock:
            pending[index][key] = message
            if key in pending[1 - index]:
                latest = (pending[0].pop(key), pending[1].pop(key), time.monotonic())
            while len(pending[index]) > 8:
                pending[index].popitem(last=False)

    def frame():
        with lock:
            pair = latest
        if pair is None:
            raise RuntimeError("等待相机图像及同帧 camera_info")
        image, info, received = pair
        age = time.monotonic() - received
        if age > 2:
            raise RuntimeError(f"相机画面已过期（{age:.1f} 秒），未保存照片")
        metadata = {
            "serial": args.serial, "width": image.width, "height": image.height,
            "K": list(info.k), "D": list(info.d), "R": list(info.r), "P": list(info.p),
            "distortion_model": info.distortion_model, "frame_id": image.header.frame_id,
            "image_stamp": {"sec": image.header.stamp.sec, "nanosec": image.header.stamp.nanosec},
            "image_encoding": image.encoding, "age_s": age, "captured_at_utc": now(),
            "topic": "/calib_capture/camera/color/image_raw",
            "processing": "PNG from image_raw; no crop, resize, or undistortion",
        }
        return image, metadata

    bridge = CvBridge()

    def route(handler, method, path):
        image, metadata = frame()
        if method == "GET" and path == "/status":
            handler.reply(metadata)
        elif method == "GET" and path == "/preview.jpg":
            pixels = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            ok, jpeg = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise RuntimeError("JPEG 编码失败")
            handler.reply(jpeg.tobytes(), "image/jpeg")
        elif method == "POST" and path == "/snapshot":
            pixels = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            ok, png = cv2.imencode(".png", pixels, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            if not ok:
                raise RuntimeError("PNG 编码失败")
            result = io.BytesIO()
            with zipfile.ZipFile(result, "w") as archive:
                archive.writestr("image.png", png.tobytes())
                archive.writestr("camera.json", json.dumps(metadata, indent=2))
            handler.reply(result.getvalue(), "application/zip")
        else:
            handler.reply({"error": "not found"}, status=404)

    http = server(args.remote_port, True, route)
    rclpy.init()
    node = Node("gamepad_vla_calibration_preview")
    node.create_subscription(Image, "/calib_capture/camera/color/image_raw",
                             lambda msg: receive(msg, 0), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "/calib_capture/camera/color/camera_info",
                             lambda msg: receive(msg, 1), qos_profile_sensor_data)
    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    command = [
        "ros2", "launch", "realsense2_camera", "rs_launch.py",
        "camera_namespace:=calib_capture", "camera_name:=camera",
        f"serial_no:=_{args.serial}", f"rgb_camera.color_profile:={args.width},{args.height},30",
        "rgb_camera.color_format:=RGB8", "enable_color:=true", "enable_depth:=false",
        "enable_infra:=false", "enable_infra1:=false", "enable_infra2:=false",
        "enable_gyro:=false", "enable_accel:=false", "pointcloud.enable:=false",
        "align_depth.enable:=false", "publish_tf:=false",
    ]
    camera = subprocess.Popen(command, start_new_session=True)
    try:
        print("CAMERA_HTTP_READY", flush=True)
        http.serve_forever()
    finally:
        http.server_close()
        if camera.poll() is None:
            os.killpg(camera.pid, signal.SIGINT)
            try:
                camera.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(camera.pid, signal.SIGTERM)
                camera.wait(timeout=5)
        rclpy.try_shutdown()
        spin_thread.join(timeout=3)
        node.destroy_node()


def local_main(args):
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    opener = build_opener(ProxyHandler({}))
    upstream = f"http://127.0.0.1:{args.tunnel_port}"

    def request(path, post=False, timeout=10):
        req = Request(upstream + path, data=b"" if post else None)
        with opener.open(req, timeout=timeout) as response:
            return response.read()

    def route(handler, method, path):
        if method == "GET" and path == "/preview.jpg":
            handler.reply(request(path), "image/jpeg")
        elif method == "GET" and path == "/status":
            data = json.loads(request(path))
            data.update(output_dir=str(output),
                        captures=sorted((p.stem for p in output.glob("*.png")), reverse=True))
            handler.reply(data)
        elif method == "POST" and path == "/capture":
            body = json.loads(handler.rfile.read(int(handler.headers.get("Content-Length", 0))) or b"{}")
            label = re.sub(r"[^a-zA-Z0-9_-]", "_", str(body.get("label", "capture")))[:50] or "capture"
            name = label + "_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            with zipfile.ZipFile(io.BytesIO(request("/snapshot", post=True))) as archive:
                pixels = archive.read("image.png")
                metadata = json.loads(archive.read("camera.json"))
            metadata.update(label=body.get("label", ""), saved_locally_at_utc=now(),
                            image_file=name + ".png")
            (output / (name + ".png")).write_bytes(pixels)
            (output / (name + ".json")).write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
            handler.reply({"path": str(output / (name + ".png")), "name": name})
        elif method == "GET" and path.startswith("/files/"):
            name = path.removeprefix("/files/")
            if Path(name).name != name or Path(name).suffix not in (".png", ".json"):
                handler.reply({"error": "not found"}, status=404)
                return
            content_type = "image/png" if name.endswith(".png") else "application/json"
            handler.reply((output / name).read_bytes(), content_type)
        else:
            handler.reply({"error": "not found"}, status=404)

    http = server(args.port, False, route)
    profile = str(args.profile.expanduser())
    ssh = ["bash", profile, "-o", "BatchMode=yes", "-o", "ConnectTimeout=10"]
    session = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    remote_dir = args.remote_root.rstrip("/") + "/" + session
    remote_file = remote_dir + "/camera_capture.py"
    install = f"mkdir -p {shlex.quote(remote_dir)} && cat > {shlex.quote(remote_file)}"
    subprocess.run(ssh + [install], input=Path(__file__).read_bytes(), check=True, timeout=30)
    launch = (
        f"source {shlex.quote(args.ros_setup)} && "
        f"export ROS_DOMAIN_ID={args.ros_domain_id} ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR={shlex.quote(remote_dir + '/ros_logs')} && "
        "exec " + shlex.quote(args.remote_python) + " -u " + shlex.quote(remote_file) +
        f" --serve-camera --serial {shlex.quote(args.serial)} --width {args.width} --height {args.height}"
        f" --remote-port {args.remote_port}"
    )
    log = (output / "camera.log").open("ab")
    process = subprocess.Popen(
        ssh + ["-o", "ExitOnForwardFailure=yes",
               "-L", f"127.0.0.1:{args.tunnel_port}:127.0.0.1:{args.remote_port}", launch],
        stdout=log, stderr=subprocess.STDOUT,
    )
    try:
        deadline = time.monotonic() + 45
        while True:
            if process.poll() is not None:
                raise RuntimeError(f"相机连接退出，见 {output / 'camera.log'}")
            try:
                data = json.loads(request("/status", timeout=1))
                break
            except (OSError, ValueError):
                if time.monotonic() > deadline:
                    raise RuntimeError(f"相机 45 秒内未就绪，见 {output / 'camera.log'}")
                time.sleep(.3)
        (output / "session.json").write_text(json.dumps({
            "started_at_utc": now(), "local_pid": os.getpid(), "ssh_pid": process.pid,
            "local_url": f"http://127.0.0.1:{args.port}", "remote_script": remote_file,
            "source_serial": args.serial, "initial_camera_info": data,
        }, indent=2) + "\n")
        print(f"READY http://127.0.0.1:{args.port} -> {output}", flush=True)
        http.serve_forever()
    finally:
        http.server_close()
        if process.poll() is None:
            try:
                request("/stop", post=True, timeout=3)
            except OSError:
                pass
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--remote-root")
    parser.add_argument("--output", type=Path,
                        default=Path("build/camera_capture") / datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--port", type=int)
    parser.add_argument("--tunnel-port", type=int)
    parser.add_argument("--remote-port", type=int)
    parser.add_argument("--serve-camera", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not args.serve_camera:
        # The uploaded camera worker remains standalone and receives its settings via CLI.
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
        from gamepad_vla.site import site_value
        for name in ("width", "height", "port", "tunnel_port", "remote_port"):
            if getattr(args, name) is None:
                setattr(args, name, site_value("camera." + name, required=True))
        args.serial = args.serial or site_value("camera.serials.camera_3", required=True)
        args.profile = args.profile or Path(site_value("station.ssh_profile", required=True))
        args.remote_root = args.remote_root or site_value("station.camera_root", required=True)
        args.ros_setup = site_value("station.ros_setup", required=True)
        args.remote_python = site_value("station.python", required=True)
        args.ros_domain_id = int(site_value("camera.ros_domain_id", required=True))
    elif not args.serial or not args.width or not args.height or not args.remote_port:
        parser.error("camera worker requires serial, width, height and remote-port")

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupted)
    try:
        (camera_main if args.serve_camera else local_main)(args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
