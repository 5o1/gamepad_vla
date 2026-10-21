#!/usr/bin/env python3
"""Dual-hand/dual-arm simulation collection. Never constructs a hardware backend."""
import argparse
import json
import queue
import sys
import threading
import time
from pathlib import Path

from gamepad_vla.site import configure_runtime, site_value
from gamepad_vla.workstation.arm_mapping import ArmMapper
from gamepad_vla.workstation.collection import Session, create_backend
from gamepad_vla.workstation.collection_sources import CollectionReplay, Ros2Source
from gamepad_vla.workstation.sources import UdpSource
from gamepad_vla.workstation.spec import load_spec


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--backend", choices=("sapien", "mujoco"), required=True)
    p.add_argument("--scene-variant", choices=("full", "free-space"), default="full",
                   help="free-space disables environment collision in code, leaving v4.4 unchanged")
    p.add_argument("--mujoco-workers", type=int, default=1,
                   help="native CPU worker threads for MuJoCo; changes no solver parameters")
    p.add_argument("--spec", type=Path, default=Path(__file__).resolve().parents[1] / "configs/workcell/station_g20_v4_4.json")
    p.add_argument("--source", choices=("ros2", "udp", "replay"), required=True)
    p.add_argument("--replay", type=Path)
    p.add_argument("--ros-topic-prefix", default="", help="optional ROS subscription topic prefix")
    p.add_argument("--loop-replay", action="store_true")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--resource-root", type=Path)
    p.add_argument("--duration", type=float, required=True)
    p.add_argument("--rate-hz", type=int, default=30)
    p.add_argument("--realtime", action="store_true")
    p.add_argument("--record", action="store_true", help="start recording immediately; otherwise keyboard/browser start")
    p.add_argument("--note", default="")
    p.add_argument("--view", action="store_true")
    p.add_argument("--host", default=site_value("operator.bind_host", "127.0.0.1"))
    p.add_argument("--port", type=int, default=site_value("operator.port", 8780))
    p.add_argument("--udp-host", default=site_value("udp.bind_host", "127.0.0.1"))
    p.add_argument("--udp-port", type=int, default=site_value("udp.port", 50200))
    a = p.parse_args()
    if a.duration <= 0 or a.rate_hz <= 0:
        p.error("duration and rate must be positive")
    if a.mujoco_workers < 1 or (a.backend != "mujoco" and a.mujoco_workers != 1):
        p.error("--mujoco-workers must be positive and applies only to MuJoCo")
    if a.source == "replay" and a.replay is None:
        p.error("--replay is required")
    configure_runtime()
    spec = load_spec(a.spec)
    mapper = ArmMapper({s: getattr(spec, s + "_arm").home_q for s in ("left", "right")})
    backend = source = session = view = None
    controls = queue.Queue()
    if sys.stdin.isatty():
        def keyboard():
            for line in sys.stdin:
                text = line.strip()
                controls.put({"action": text if text in {"start", "stop"} else "note", "text": text})
        threading.Thread(target=keyboard, daemon=True).start()
    success = False
    try:
        if a.source == "ros2":
            source = Ros2Source(mapper, topic_prefix=a.ros_topic_prefix)
        elif a.source == "udp":
            source = UdpSource(a.udp_host, a.udp_port, stale_after_s=.25,
                               allow_source=site_value("udp.allow_sources", []) or None, arm_mapper=mapper)
        else:
            source = CollectionReplay(a.replay, loop=a.loop_replay)
        # Session creates output first, so keep the backend's generated assets in
        # a sibling directory rather than pre-creating the recording destination.
        backend = create_backend(a.backend, spec, work_dir=a.output.with_name(a.output.name + "-physics"),
                                 render=False, resource_root=a.resource_root,
                                 environment_collisions=a.scene_variant == "full")
        backend.reset()
        if a.backend == "mujoco" and a.mujoco_workers > 1:
            backend.mj.mju_threadpool(backend.data, a.mujoco_workers)
        session = Session(a.output, spec, backend, dict(kind=a.source, replay=None if a.replay is None else str(a.replay),
                                                     loop=a.loop_replay), mapper=mapper, fps=a.rate_hz)
        session.meta["native_cpu_workers"] = a.mujoco_workers
        if a.note:
            session.control(dict(action="note", text=a.note))
        if a.record:
            session.control(dict(action="start"))
        if a.view:
            from gamepad_vla.workstation.collection_stream import OperatorStream
            view = OperatorStream(a.backend, a.spec, a.resource_root, a.output / "view", a.host, a.port,
                                  environment_collisions=a.scene_variant == "full")
        begin = time.monotonic()
        n = max(1, round(a.duration * a.rate_hz))
        for i in range(n):
            if a.realtime:
                time.sleep(max(0., begin + i / a.rate_hz - time.monotonic()))
            if hasattr(source, "drain_events"):
                session.record_inputs(source.drain_events())
            while True:
                try:
                    session.control(controls.get_nowait())
                except queue.Empty:
                    break
            if view:
                for event in view.events():
                    session.control(event)
            commands = source.poll(time.monotonic() if a.source != "replay" else i / a.rate_hz)
            backend.apply(commands)
            applied_ns = time.monotonic_ns()
            backend.step(1 / a.rate_hz)
            row = session.record(backend.observe(), commands, applied_monotonic_ns=applied_ns,
                                 input_sequences=getattr(source, "polled_sequences", None))
            if view:
                view.publish(row)
        result = dict(ticks=n, sim_time_s=row["sim_time_s"], wall_elapsed_s=time.monotonic() - begin,
                      decode_errors=source.decode_errors, source_cycles=getattr(source, "cycles", 0))
        (a.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))
        success = True
    except RuntimeError as exc:
        p.exit(2, str(exc) + "\n")
    except KeyboardInterrupt:
        pass
    finally:
        if source:
            source.close()
            if session and hasattr(source, "drain_events"):
                session.record_inputs(source.drain_events())
        if session:
            session.close("complete" if success else "interrupted")
        if view:
            view.close()
        if backend:
            backend.close()


if __name__ == "__main__":
    main()
