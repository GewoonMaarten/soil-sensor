"""Flash firmware to a device via `uv run west flash`.

Must be run from the repository root after a successful build. Passes
arguments straight through to `west flash`; e.g. `--runner` or `--dev-id`.

Examples:
    uv run tools/scripts/west_flash.py
    uv run tools/scripts/west_flash.py --build-dir build
    uv run tools/scripts/west_flash.py -n
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_BUILD_DIR = "build"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Flash built firmware with west.",
        epilog="Extra arguments after `--` are forwarded verbatim to `west flash`.",
    )
    parser.add_argument(
        "--build-dir",
        default=DEFAULT_BUILD_DIR,
        help=f"Build directory relative to the repo root (default: {DEFAULT_BUILD_DIR}).",
    )
    parser.add_argument(
        "-n",
        "--dry-run",
        action="store_true",
        help="Print the command without running it.",
    )
    parser.add_argument(
        "west_args",
        nargs=argparse.REMAINDER,
        help="Additional arguments forwarded to `west flash` after `--`.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    build_dir = REPO_ROOT / args.build_dir
    if not build_dir.exists() and not args.dry_run:
        print(f"Build directory not found: {build_dir}", file=sys.stderr)
        print("Run tools/scripts/west_build.py first.", file=sys.stderr)
        return 1

    command = [
        "uv",
        "run",
        "west",
        "flash",
        f"--build-dir={args.build_dir}",
        *args.west_args,
    ]

    print(" ".join(command), file=sys.stderr)
    if args.dry_run:
        return 0

    result = subprocess.run(command, cwd=REPO_ROOT, env=os.environ.copy())
    if result.returncode == 0:
        print("Flash succeeded.", file=sys.stderr)
    else:
        print(f"Flash failed with exit code {result.returncode}", file=sys.stderr)
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
