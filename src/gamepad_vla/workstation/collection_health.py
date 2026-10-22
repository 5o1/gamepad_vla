"""Measured input health and read-only Linux process safety inspection."""
from collections import deque
from pathlib import Path
import ipaddress
import re
import threading
import time

CHANNELS = tuple(s + '_' + k for s in ('left', 'right') for k in ('hand', 'arm'))
SAFETY_INTERVAL_S = 1.0
STALE_S = .25
FPS_MIN = 30
LATENCY_P95_MAX_MS = 50
DISK_MIN_BYTES = 10 * 1024**3


def require_loopback(host):
    if host != 'localhost' and not ipaddress.ip_address(host).is_loopback:
        raise ValueError('collection web must bind a loopback address')
    return host


class InputHealth:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.channels = {k: dict(count=0, decode_errors=0, times=deque(), last=None) for k in CHANNELS}

    def receive(self, channel, *, error=False):
        if channel not in self.channels:
            return
        now = self.clock()
        with self.lock:
            c = self.channels[channel]
            c['count'] += 1
            c['decode_errors'] += int(error)
            c['last'] = now
            c['times'].append(now)
            while c['times'] and c['times'][0] < now - 2:
                c['times'].popleft()

    def snapshot(self):
        now = self.clock()
        with self.lock:
            result = {}
            for key, c in self.channels.items():
                times = [t for t in c['times'] if now - t <= 2]
                age = None if c['last'] is None else max(0., now - c['last'])
                # Include silence in the denominator: stopped inputs decay to 0 Hz.
                hz = (len(times)-1) / (now-times[0]) if len(times)>1 and now>times[0] else 0.
                result[key] = dict(count=c['count'], decode_errors=c['decode_errors'], hz=hz,
                                   age_s=age, warning=age is None or age > STALE_S)
            return result



def hardware_process_identity(argv):
    """Return a matching program identity, never search arbitrary arguments.

    /proc cmdline is NUL-separated argv. Shell/terminal wrapper payloads are
    excluded: their real node children have independent process identities.
    """
    if not argv or not argv[0]:
        return None
    program = Path(argv[0]).name
    ignored = {'bash', 'sh', 'zsh', 'dash', 'fish', 'ksh', 'grep', 'rg',
               'bwrap', 'bubblewrap', 'firejail', 'sandbox-exec'}
    if program in ignored or program.endswith('sandbox'):
        return None
    identity, rest = argv[0], list(argv[1:])
    module = False
    if re.fullmatch(r'python(?:[0-9]+(?:\.[0-9]+)*)?', program):
        identity = None
        i = 0
        while i < len(rest):
            arg = rest[i]
            if arg in ('-c', '-'):
                return None
            if arg == '-m':
                identity = rest[i+1] if i+1 < len(rest) else None
                module = True
                rest = []
                break
            if arg in ('-W', '-X', '--check-hash-based-pycs'):
                i += 2
                continue
            if arg == '--':
                i += 1
                if i >= len(rest):return None
                arg = rest[i]
            if not arg.startswith('-'):
                identity, rest = arg, rest[i+1:]
                break
            i += 1
    if identity is None:
        return None
    if Path(identity).name == 'ros2' and len(rest) >= 2 and rest[0] in ('run', 'launch'):
        identity = rest[1]
    prefixes = ('linker_hand_ros2_sdk', 'rokae_')
    # Dotted module names are package components, just as slashes delimit paths.
    parts = str(identity).split('.') if module else Path(identity).parts
    return str(identity) if any(p.startswith(prefixes) for p in parts) else None


def detect_safety(proc_root=Path('/proc')):
    """Read identities only. Never sends a signal or opens a device."""
    matches, errors = [], []
    try:
        processes = list(Path(proc_root).iterdir())
    except OSError as exc:
        return dict(checked_wall_time_ns=time.time_ns(), safe=False, matches=[], errors=[str(exc)])
    for p in processes:
        if not p.name.isdigit():continue
        try:
            argv = [a.decode(errors='replace') for a in (p/'cmdline').read_bytes().split(b'\0') if a]
        except FileNotFoundError:
            continue
        except OSError as exc:
            errors.append(f'{p.name}: {exc.strerror}')
            continue
        identity = hardware_process_identity(argv)
        if identity is not None:
            # Full shell payloads are not useful node identities or banner text.
            matches.append(dict(pid=int(p.name), identity=identity, command=identity))
    return dict(checked_wall_time_ns=time.time_ns(), safe=not matches and not errors, matches=matches, errors=errors)
