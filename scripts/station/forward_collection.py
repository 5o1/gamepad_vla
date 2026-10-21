#!/usr/bin/env python3
"""Subscribe to glove/LinkerTA commands and forward UDP; no ROS publishers."""
import argparse
import socket
import time

from gamepad_vla.site import site_value
from gamepad_vla.workstation.arm_mapping import ArmMapper
from gamepad_vla.workstation.collection_sources import Ros2Source
from gamepad_vla.workstation.protocol import encode


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default=site_value("udp.peer_host"))
    p.add_argument("--port", type=int, default=site_value("udp.port", 50200))
    a = p.parse_args()
    if not a.host:
        p.error("configure udp.peer_host privately or pass --host")
    # Mapping at this subscriber is only validation; transmit master-deg so the
    # simulator maps against its own explicit startup pose exactly once.
    mapper = ArmMapper({s: [0.] * 7 for s in ("left", "right")})
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            source = Ros2Source(mapper, packet_sink=lambda packet: sock.sendto(encode(packet), (a.host, a.port)))
        except RuntimeError as exc:
            p.exit(2, str(exc) + "\n")
        try:
            while True:
                source.poll(time.monotonic())
                source.drain_events()  # forwarding has no local recorder
                time.sleep(.001)
        except KeyboardInterrupt:
            pass
        finally:
            source.close()


if __name__ == "__main__":
    main()
