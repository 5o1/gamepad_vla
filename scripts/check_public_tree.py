#!/usr/bin/env python3
"""Check publishable working-tree files without printing matched private values."""
import argparse
from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from gamepad_vla.provenance import source_files


PATTERNS = {
    "private key": re.compile(r"-----BEGIN (?:RSA |OPENSSH |EC |ENCRYPTED )?PRIVATE KEY-----"),
    "provider token": re.compile(r"\b(?:gh[pousr]_[A-Za-z0-9_]{30,}|github_pat_[A-Za-z0-9_]{40,}|sk-(?:proj-|ant-api\d+-)?[A-Za-z0-9_-]{28,}|AKIA[0-9A-Z]{16})\b"),
    "documented password": re.compile(r"(?:超级管理员密码|管理员密码|密码|口令)\s*[:：=]?\s*[A-Za-z0-9!@#$%^&*._+-]{4,}"),
    "credential URL": re.compile(r"\b(?:https?|postgres(?:ql)?|mysql|redis|mongodb)://[^\s/:\"<>]+:[^\s/@\"<>]+@"),
    "private IPv4": re.compile(r"\b(?:192\.168(?:\.\d{1,3}){2}|10(?:\.\d{1,3}){3}|172\.(?:1[6-9]|2\d|3[01])(?:\.\d{1,3}){2}|100\.(?:6[4-9]|[7-9]\d|1[01]\d|12[0-7])(?:\.\d{1,3}){2})\b"),
    "machine-specific path": re.compile(r"(?<![A-Za-z0-9_/.:-])/(?:home|data)/[A-Za-z0-9_.-]+/|/mnt/[a-z]/Users/[^/\s]+/|[A-Z]:\\Users\\"),
    "private document link": re.compile(r"https?://(?:www\.)?(?:kdocs\.cn/l|docs\.qq\.com/(?:doc|sheet))/[^\s\"<>]+"),
    "device serial": re.compile(r"[\"']\d{12}[\"']"),
}


def findings(root):
    for name in source_files(root):
        if name.startswith(".local/") or name == "configs/site.local.json":
            yield name, 1, "private file tracked by Git"
            continue
        data = (root / name).read_bytes()
        if b"\0" in data:
            continue
        try:
            text = data.decode("utf-8")
        except UnicodeDecodeError:
            continue
        for number, line in enumerate(text.splitlines(), 1):
            for category, pattern in PATTERNS.items():
                if pattern.search(line):
                    yield name, number, category


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    matches = list(findings(args.root.resolve()))
    for name, number, category in matches:
        print(f"{name}:{number}: {category} [value omitted]")
    print(f"Public tree check: {len(matches)} findings (patterns do not replace manual review)")
    return bool(matches)


if __name__ == "__main__":
    raise SystemExit(main())
