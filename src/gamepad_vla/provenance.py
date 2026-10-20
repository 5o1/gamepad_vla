"""Code identity shared by deployment archives and recorded episodes."""
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys


def source_files(root):
    """Working-tree files selected by Git, including non-ignored new files."""
    result = subprocess.run(
        ["git", "-C", str(root), "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        check=True, stdout=subprocess.PIPE)
    return sorted({p for p in result.stdout.decode().split("\0")
                   if p and (root / p).is_file()})


def source_digest(root, files):
    digest = hashlib.sha256()
    for name in files:
        # Data/report updates do not change executable code identity.
        if (name.split("/")[0] in {"src", "scripts", "configs", "constraints"}
                or name == "pyproject.toml"):
            digest.update(name.encode() + b"\0")
            digest.update((root / name).read_bytes())
    return digest.hexdigest()


def code_provenance(root):
    root = Path(root).resolve()
    if (root / ".git").exists():
        commit = subprocess.run(["git", "-C", str(root), "rev-parse", "--verify", "HEAD"],
                                capture_output=True, text=True)
        status = subprocess.run(["git", "-C", str(root), "status", "--porcelain"],
                                check=True, capture_output=True, text=True)
        return {"git_commit": commit.stdout.strip() or None, "dirty": bool(status.stdout),
                "source_sha256": source_digest(root, source_files(root))}
    snapshot = root / ".code-provenance.json"
    if snapshot.is_file():
        result = json.loads(snapshot.read_text())
        # Detect edits made to a deployed code copy after synchronization.
        files = [str(p.relative_to(root)) for folder in ("src", "scripts", "configs", "constraints")
                 for p in (root / folder).rglob("*")
                 if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"
                 and p != root / "configs/site.local.json"
                 and not any(part.endswith(".egg-info") for part in p.parts)]
        if (root / "pyproject.toml").is_file():
            files.append("pyproject.toml")
        digest = source_digest(root, sorted(files))
        result["deployment_modified"] = digest != result["source_sha256"]
        result["source_sha256"] = digest
        return result
    return {"git_commit": None, "dirty": None, "source_sha256": None}


def runtime_provenance(root, spec_path):
    result = code_provenance(root)
    result.update(python=sys.version, command=list(sys.argv),
                  spec_sha256=hashlib.sha256(Path(spec_path).read_bytes()).hexdigest())
    result["packages"] = {}
    for name in ("gamepad-vla", "numpy", "sapien", "xbox-gamepad-physics"):
        try:
            result["packages"][name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            continue
    return result
