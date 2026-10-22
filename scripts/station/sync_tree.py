#!/usr/bin/env python3
"""Sync this repository's tracked and new (non-ignored) files to a remote code copy.

The remote shell command comes from the site configuration (``station.remote_shell``),
for example a wrapper that runs ``ssh <host>``. It must accept a command string
and pass stdin through. Files deleted from git are removed remotely, and every
synced file is checked by SHA-256 afterwards.
"""
import argparse
import hashlib
import io
import shlex
import subprocess
import sys
import tarfile
from pathlib import Path

from gamepad_vla.site import site_value

ROOT = Path(__file__).resolve().parents[2]


def files():
    out = subprocess.check_output(["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"], cwd=ROOT)
    return sorted(p for p in out.decode().split("\0") if p and (ROOT / p).is_file())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    profile = site_value("station.ssh_profile", None)
    p.add_argument("--remote-shell", default=site_value("station.remote_shell", None) or (f"bash {profile}" if profile else None),
                   help="command prefix that runs a shell command on the station")
    p.add_argument("--remote-dir", default=site_value("station.code_dir", "~/gamepad_vla/code"))
    p.add_argument("--dry-run", action="store_true")
    a = p.parse_args()
    if not a.remote_shell:
        sys.exit("set station.remote_shell in the site configuration or pass --remote-shell")
    names = files()
    digests = {n: hashlib.sha256((ROOT / n).read_bytes()).hexdigest() for n in names}
    if a.dry_run:
        print("\n".join(names)); return
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tar:
        for n in names:
            tar.add(ROOT / n, arcname=n)
        manifest = "\n".join(f"{d}  {n}" for n, d in digests.items()).encode() + b"\n"
        info = tarfile.TarInfo(".sync-manifest.sha256"); info.size = len(manifest)
        tar.addfile(info, io.BytesIO(manifest))
    d = a.remote_dir
    remote = (f"mkdir -p {d} && cd {d} && tar xzf - && "
              # remove previously synced files that are no longer part of the tree
              f"if [ -f .sync-manifest.prev ]; then comm -23 <(awk '{{print $2}}' .sync-manifest.prev | sort) "
              f"<(awk '{{print $2}}' .sync-manifest.sha256 | sort) | xargs -r rm -f --; fi; "
              f"sha256sum --quiet -c .sync-manifest.sha256 && cp .sync-manifest.sha256 .sync-manifest.prev && "
              f"echo synced $(wc -l < .sync-manifest.sha256) files")
    cmd = shlex.split(a.remote_shell) + [f"bash -c {shlex.quote(remote)}"]
    r = subprocess.run(cmd, input=buf.getvalue())
    sys.exit(r.returncode)


if __name__ == "__main__":
    main()
