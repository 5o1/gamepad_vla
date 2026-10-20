#!/usr/bin/env python3
"""Check that a built wheel carries the operator UI and recovery entry point."""
import argparse
from pathlib import Path
import zipfile


def check(path):
    with zipfile.ZipFile(path) as archive:
        if any("site.local.json" in name or name.startswith(".local/") for name in archive.namelist()):
            raise ValueError("private configuration or archive is present")
        for filename in ("index.html", "app.js"):
            archive.getinfo(f"gamepad_vla/workstation/operator_web/{filename}")
        entry = next(name for name in archive.namelist() if name.endswith("/entry_points.txt"))
        if b"gamepad-vla-recover" not in archive.read(entry):
            raise ValueError("recovery CLI is missing")
    print(f"distribution OK: {path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wheel", type=Path)
    check(parser.parse_args().wheel)
