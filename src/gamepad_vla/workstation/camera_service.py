"""Station-local ROS camera worker, migrated from the calibration capture tool.

Only the collection server exposes this loopback HTTP worker to the browser.
Same-frame Image/CameraInfo pairing, expiry checks and PNG/JSON are preserved.
"""
import argparse
from collections import OrderedDict, deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import signal
import subprocess
import threading
import time
from urllib.parse import unquote, urlsplit
import zipfile


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
    received_times = deque(maxlen=90)

    def receive(message, index):
        nonlocal latest
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        with lock:
            pending[index][key] = message
            if key in pending[1 - index]:
                latest = (pending[0].pop(key), pending[1].pop(key), time.monotonic())
                received_times.append(latest[2])
            while len(pending[index]) > 8:
                pending[index].popitem(last=False)

    def frame():
        with lock:
            pair = latest
            times = list(received_times)
        if pair is None:
            raise RuntimeError("Waiting for a camera image and matching camera_info")
        image, info, received = pair
        age = time.monotonic() - received
        if age > 2:
            raise RuntimeError(f"Camera frame is stale ({age:.1f} s). Photo was not saved")
        metadata = {
            "serial": args.serial, "width": image.width, "height": image.height,
            "fps": (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.,
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
                raise RuntimeError("JPEG encoding failed")
            handler.reply(jpeg.tobytes(), "image/jpeg")
        elif method == "POST" and path == "/snapshot":
            pixels = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            ok, png = cv2.imencode(".png", pixels, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            if not ok:
                raise RuntimeError("PNG encoding failed")
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
    camera = subprocess.Popen(command)
    try:
        print("CAMERA_HTTP_READY", flush=True)
        http.serve_forever()
    finally:
        http.server_close()
        if camera.poll() is None:
            camera.send_signal(signal.SIGINT)
            try:
                camera.wait(timeout=10)
            except subprocess.TimeoutExpired:
                camera.terminate()
                camera.wait(timeout=5)
        rclpy.try_shutdown()
        spin_thread.join(timeout=3)
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serve-camera", action="store_true")
    parser.add_argument("--serial", required=True)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--remote-port", type=int, required=True)
    args = parser.parse_args()
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    for signum in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupted)
    try:
        camera_main(args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
