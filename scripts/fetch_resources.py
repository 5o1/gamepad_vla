#!/usr/bin/env python3
"""Fetch pinned public resources and verify them against the tracked manifest."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import urllib.error
import urllib.request

from gamepad_vla.site import site_value, workspace_root


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fetch_file(resource, entry, workspace, proxy=None):
    target = workspace / resource["destination"] / entry["path"]
    if target.is_file() and sha256(target) == entry["sha256"]:
        return {"resource": resource["id"], "file": entry["path"], "status": "verified"}
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".part")
    for attempt in range(3):
        opener = urllib.request.build_opener(urllib.request.ProxyHandler(
            {} if not proxy else {"http": proxy, "https": proxy}))
        try:
            request = urllib.request.Request(entry["url"], headers={"User-Agent": "gamepad-vla-resource-fetcher"})
            with opener.open(request, timeout=30) as response, temporary.open("wb") as stream:
                for block in iter(lambda: response.read(1024 * 1024), b""):
                    stream.write(block)
            if temporary.stat().st_size != entry["bytes"] or sha256(temporary) != entry["sha256"]:
                raise ValueError(f"checksum/size mismatch: {resource['id']}/{entry['path']}")
            temporary.replace(target)
            return {"resource": resource["id"], "file": entry["path"], "status": "downloaded"}
        except (OSError, TimeoutError) as error:
            if attempt == 2 or (isinstance(error, urllib.error.HTTPError) and error.code < 500):
                raise
            time.sleep(attempt + 1)
        finally:
            temporary.unlink(missing_ok=True)


def fetch_git(resource, workspace, proxy=None):
    target = workspace / resource["destination"]
    if (target / ".git").exists():
        head = subprocess.run(["git", "-C", str(target), "rev-parse", "HEAD"], capture_output=True, text=True)
        if head.returncode == 0:
            dirty = subprocess.check_output(["git", "-C", str(target), "status", "--porcelain"], text=True)
            if head.stdout.strip() != resource["revision"] or dirty:
                raise ValueError(f"{resource['id']}: existing checkout differs from the pinned clean source")
            return {"resource": resource["id"], "status": "verified", "revision": resource["revision"]}
    elif target.exists() and any(target.iterdir()):
        raise ValueError(f"{resource['id']}: destination is not an empty Git checkout")
    target.mkdir(parents=True, exist_ok=True)
    subprocess.run(["git", "init", "-q", str(target)], check=True)
    env = {k: v for k, v in os.environ.items() if k.lower() not in ("http_proxy", "https_proxy", "all_proxy")}
    env["GIT_TERMINAL_PROMPT"] = "0"
    result = subprocess.run(["git", "-c", "http.proxy=" + (proxy or ""), "-C", str(target), "fetch", "--depth=1",
                             resource["repository"], resource["revision"]], env=env, capture_output=True)
    if result.returncode:
        raise RuntimeError(f"unable to fetch pinned source: {resource['id']}")
    subprocess.run(["git", "-C", str(target), "checkout", "-q", "--detach", "FETCH_HEAD"], check=True)
    actual = subprocess.check_output(["git", "-C", str(target), "rev-parse", "HEAD"], text=True).strip()
    if actual != resource["revision"]:
        raise ValueError(f"unexpected revision: {resource['id']}")
    return {"resource": resource["id"], "status": "downloaded", "revision": actual}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=Path(__file__).resolve().parents[1] / "references/external_resources.json")
    parser.add_argument("--group", choices=("simulation", "calibration", "all"), default="all")
    parser.add_argument("--workspace", type=Path)
    parser.add_argument("--proxy", default=site_value("network.proxy_url"), help="Explicit proxy to use after a direct-access failure")
    args = parser.parse_args()
    workspace = args.workspace.expanduser().resolve() if args.workspace else workspace_root()
    manifest = json.loads(args.manifest.read_text())
    resources = [r for r in manifest["resources"] if args.group == "all" or r["group"] == args.group]
    jobs = [(r, f) for r in resources if r["kind"] == "files" for f in r["files"]]
    results = []
    pool = ThreadPoolExecutor(max_workers=4)
    try:
        for result in pool.map(lambda job: fetch_file(*job, workspace, args.proxy), jobs):
            results.append(result)
            print(f"{result['status']}: {result['resource']}/{result['file']}", flush=True)
    finally:
        pool.shutdown(wait=True, cancel_futures=True)
    for resource in resources:
        if resource["kind"] == "git":
            results.append(fetch_git(resource, workspace, args.proxy))
    report = workspace / "assets" / ("fetch_" + args.group + ".json")
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps({"manifest_sha256": sha256(args.manifest), "resources": results}, indent=2) + "\n")
    print(f"Verified {len(results)} resource entries")


if __name__ == "__main__":
    main()
