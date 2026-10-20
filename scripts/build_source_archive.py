#!/usr/bin/env python3
"""Archive Git-selected working-tree files and their code provenance."""
import argparse
import io
import json
from pathlib import Path
import sys
import tarfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from gamepad_vla.provenance import code_provenance, source_files


def build_archive(root, output):
    root, output = Path(root).resolve(), Path(output).resolve()
    files = source_files(root)
    provenance = code_provenance(root)
    with tarfile.open(output, "w:gz") as archive:
        for name in files:
            if (root / name).resolve() != output:
                archive.add(root / name, arcname=name, recursive=False)
        content = (json.dumps(provenance, indent=2) + "\n").encode()
        info = tarfile.TarInfo(".code-provenance.json")
        info.size = len(content)
        archive.addfile(info, io.BytesIO(content))
    return provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build_archive(Path(__file__).resolve().parents[1], args.output)))


if __name__ == "__main__":
    main()
