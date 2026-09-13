#!/usr/bin/env python3
"""Reject whitespace regressions in maintained PicoTracker text files."""

from __future__ import annotations

import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TEXT_SUFFIXES = {".c", ".h", ".md", ".py", ".yml", ".yaml", ".json", ".toml", ".txt"}
SPECIAL_NAMES = {
    "Makefile",
    "Dockerfile",
    ".gitignore",
    ".gitattributes",
    ".dockerignore",
    "VERSION",
}
EXCLUDED_PREFIXES = ("hardware/", "test_firmware/", "cad/")
EXCLUDED_PATHS = {"tools/with_pips_data.txt"}


def tracked_files() -> list[Path]:
    output = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT)
    return [ROOT / item.decode() for item in output.split(b"\0") if item]


def should_check(path: Path) -> bool:
    relative = path.relative_to(ROOT).as_posix()
    if relative in EXCLUDED_PATHS or relative.startswith(EXCLUDED_PREFIXES):
        return False
    return path.suffix.lower() in TEXT_SUFFIXES or path.name in SPECIAL_NAMES


def main() -> int:
    problems: list[str] = []
    for path in tracked_files():
        if not should_check(path):
            continue
        data = path.read_bytes()
        if b"\0" in data:
            continue
        relative = path.relative_to(ROOT).as_posix()
        for number, line in enumerate(data.split(b"\n"), start=1):
            if line.endswith((b" ", b"\t", b"\r")):
                problems.append(f"{relative}:{number}: trailing whitespace")
        if data and not data.endswith(b"\n"):
            problems.append(f"{relative}: missing final newline")

    if problems:
        print("Whitespace check failed:")
        print("\n".join(problems))
        return 1
    print("whitespace check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
