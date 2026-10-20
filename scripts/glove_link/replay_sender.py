#!/usr/bin/env python3
"""Replay recorded left motor actions as relay-compatible UDP, without ROS/hardware."""
import argparse
import json
from pathlib import Path
import socket
import time

import numpy as np

from gamepad_vla.workstation.protocol import HandCommandPacket, encode
from gamepad_vla.site import site_value


def load_actions(path):
    with np.load(path, allow_pickle=False) as z:
        names, units = z['names'].tolist(), z['units'].tolist()
        indices = [names.index(f'left_hand_motor_{i}') for i in range(20)]
        if any(units[i] != 'motor_0_255' for i in indices):
            raise ValueError('left hand action units must be motor_0_255')
        actions = z['action'][:, indices]
        times = z['timestamp'].astype(float)
    if len(times) < 2 or not np.all(np.diff(times) > 0):
        raise ValueError('expected one episode with increasing timestamps')
    if not np.isfinite(actions).all() or actions.min() < 0 or actions.max() > 255:
        raise ValueError('invalid motor actions')
    return times-times[0], actions


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--npz', type=Path, required=True)
    p.add_argument('--host', default=site_value('udp.peer_host', '127.0.0.1'))
    p.add_argument('--port', type=int, default=site_value('udp.port', 50200))
    p.add_argument('--source-host', default=site_value('udp.source_host') or None,
                   help='Explicit local source IPv4 (for allowlist validation)')
    p.add_argument('--rate-hz', type=float, default=60)
    p.add_argument('--duration', type=float, default=320)
    p.add_argument('--log', type=Path, required=True)
    a = p.parse_args(argv)
    if a.rate_hz <= 0 or a.duration <= 0:
        p.error('rate and duration must be positive')
    times, actions = load_actions(a.npz)
    cycle = times[-1] + float(np.median(np.diff(times)))
    a.log.parent.mkdir(parents=True, exist_ok=True)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, a.log.open('x', buffering=1) as log:
        if a.source_host:
            sock.bind((a.source_host, 0))
        start = time.monotonic()
        for seq in range(int(np.ceil(a.duration*a.rate_hz))):
            deadline = start + seq/a.rate_hz
            time.sleep(max(0, deadline-time.monotonic()))
            phase = (seq/a.rate_hz) % cycle
            index = max(0, int(np.searchsorted(times, phase, side='right'))-1)
            # Same-host monotonic stamp only for this replay experiment. Live
            # relay stamps are ROS time and must NOT be subtracted from this clock.
            stamp = time.monotonic()
            payload = encode(HandCommandPacket('left', 'motor', [], actions[index].tolist(), [], [], stamp))
            sock.sendto(payload, (a.host, a.port))
            log.write(json.dumps(dict(seq=seq, stamp=stamp, sent_ns=time.monotonic_ns(),
                                      row=index))+'\n')
    print(json.dumps(dict(packets=seq+1, elapsed_s=time.monotonic()-start,
                          cycle_s=cycle, original_rows=len(times), log=str(a.log))))


if __name__ == '__main__':
    main()
