#!/usr/bin/env python3
"""Run the pure-Python collection workstation with registered backends."""

import argparse
from contextlib import ExitStack
import json
import time
from datetime import datetime, timezone
from pathlib import Path

from gamepad_vla import targets
from gamepad_vla.provenance import runtime_provenance
from gamepad_vla.site import configure_runtime, site_value
from gamepad_vla.workstation.backends import FanoutBackend
from gamepad_vla.workstation.backends_registry import BACKENDS
from gamepad_vla.workstation.commands import arm_command_from_packet, hand_command_from_packet
from gamepad_vla.workstation.loop import run_loop
from gamepad_vla.workstation.protocol import ArmCommandPacket, HandCommandPacket, decode
from gamepad_vla.workstation.recorder import EpisodeRecorder
from gamepad_vla.workstation.sources import ReplaySource, ScriptedSource, UdpSource
from gamepad_vla.workstation.spec import load_spec


def parser():
    result = argparse.ArgumentParser(description="Run a backend-neutral gamepad collection workstation.")
    result.add_argument("--spec", default=str(
        Path(__file__).resolve().parents[1] / "configs/workcell/station_g20_v4_4.json"),
        help="Workcell JSON (default: adopted dual-0.8R v4.4 scene)")
    result.add_argument("--source", choices=("udp", "replay", "scripted"), required=True)
    result.add_argument("--udp-bind-host", default=site_value("udp.bind_host", "127.0.0.1"))
    result.add_argument("--udp-port", type=int, default=site_value("udp.port", 50200))
    result.add_argument("--udp-allow-source", default=",".join(site_value("udp.allow_sources", [])) or None,
                        help="Comma-separated IPv4 source allowlist")
    result.add_argument("--stale-after-s", type=float, default=0.25)
    result.add_argument("--episode-dir")
    result.add_argument("--schedule", help="JSON list of {t, packet} entries for scripted input")
    result.add_argument("--backend", default="null", help="Comma-separated backend names")
    result.add_argument("--enable-real", action="store_true")
    result.add_argument("--arm-units", choices=("rad", "deg"))
    result.add_argument("--real-sides", help="Comma-separated sides the real backend may publish to")
    result.add_argument("--record-root")
    result.add_argument("--rate-hz", type=float, default=60.0)
    result.add_argument("--duration", type=float, required=True)
    result.add_argument("--target-timeline")
    result.add_argument("--operator-view", type=Path, help="Operator camera/stream YAML (sapien only)")
    result.add_argument("--operator-run", type=Path, help="New node output directory for stream and timing")
    result.add_argument("--operator-port", type=int, default=site_value("operator.port", 8780))
    return result


def _source(args):
    if args.source == "udp":
        allow = None if args.udp_allow_source is None else [ip.strip() for ip in args.udp_allow_source.split(",")]
        return UdpSource(args.udp_bind_host, args.udp_port, stale_after_s=args.stale_after_s,
                         allow_source=allow)
    if args.source == "replay":
        if not args.episode_dir:
            raise ValueError("--episode-dir is required for replay source")
        return ReplaySource(args.episode_dir)
    schedule = []
    if args.schedule:
        for entry in json.loads(Path(args.schedule).read_text()):
            packet = decode(json.dumps(entry["packet"], separators=(",", ":")).encode())
            if isinstance(packet, HandCommandPacket):
                command = hand_command_from_packet(packet)
            elif isinstance(packet, ArmCommandPacket):
                command = arm_command_from_packet(packet)
            else:
                raise ValueError("scripted packets must be command or arm_command")
            schedule.append((float(entry["t"]), command))
    return ScriptedSource(schedule)


def _backend(args, spec):
    names = [name.strip() for name in args.backend.split(",") if name.strip()]
    unknown = [name for name in names if name not in BACKENDS]
    if unknown:
        raise ValueError(f"unknown backends {unknown}; available: {sorted(BACKENDS)}")
    instances = []
    for name in names:
        if name == "real" and (args.arm_units is None or not args.real_sides):
            raise ValueError("--arm-units and --real-sides are required for the real backend")
        instances.append(BACKENDS[name](
            spec, enable_real=args.enable_real, arm_units=args.arm_units,
            real_sides=tuple((args.real_sides or "").split(","))))
    if not instances:
        raise ValueError("at least one backend is required")
    backend = instances[0] if len(instances) == 1 else FanoutBackend(instances[0], *instances[1:])
    return backend, names


def main(argv=None):
    args = parser().parse_args(argv)
    configure_runtime()
    if args.operator_view and (args.backend != "sapien" or args.operator_run is None):
        raise ValueError("--operator-view requires --backend sapien and --operator-run")
    if args.operator_view:
        import socket
        from gamepad_vla.workstation.operator_view import load_config
        load_config(args.operator_view)
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind((site_value("operator.bind_host", "127.0.0.1"), args.operator_port))
    spec = load_spec(args.spec)
    source = _source(args)
    backend, backend_names = _backend(args, spec)
    if args.operator_view:
        from gamepad_vla.workstation.operator_view import OperatorView
        backend = OperatorView(backend, args.operator_view, args.operator_run, args.spec, args.operator_port)
    target_sequence = targets.load_timeline(args.target_timeline) if args.target_timeline else None
    recorder = None
    if args.record_root:
        episode_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        recorder = EpisodeRecorder(
            args.record_root, episode_id, spec=spec, backend_names=backend_names,
            source_description=args.source, rates={"loop_hz": args.rate_hz},
            provenance=runtime_provenance(Path(__file__).resolve().parents[1], args.spec))
    summary = None
    try:
        summary = run_loop(source, backend, recorder, rate_hz=args.rate_hz,
                           duration_s=args.duration, clock=time, target_sequence=target_sequence)
        summary.update({"backends": backend_names, "source": args.source,
                        "episode": None if recorder is None else str(recorder.path)})
        if args.source == "udp":
            summary.update(decode_errors=source.decode_errors, received_packets=source.received_packets,
                           rejected_packets=source.rejected_packets)
    finally:
        # A failed export must not prevent transport/backend cleanup.
        with ExitStack() as cleanup:
            cleanup.callback(backend.close)
            cleanup.callback(source.close)
            if recorder is not None:
                cleanup.callback(recorder.close)
    print(json.dumps(summary, separators=(",", ":")))


if __name__ == "__main__":
    main()
