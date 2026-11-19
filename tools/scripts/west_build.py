#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = []
# ///
"""Build firmware for a device via `uv run west build`.

Runs from the repository root and wraps the usual build invocation:

    uv run west build -b ubx_evknorab10/nrf5340/cpuapp devices/soil-sensor/firmware/v1

Examples:
    uv run tools/scripts/west_build.py
    uv run tools/scripts/west_build.py --device central
    uv run tools/scripts/west_build.py --pristine -- cmake -DCONF_FILE=prj_release.conf
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

DEVICES = {
    "soil-sensor": {
        "board": "ubx_evknorab10/nrf5340/cpuapp",
        "source": "devices/soil-sensor/firmware/v1",
    },
    "central": {
        "board": "ubx_evknorab10/nrf5340/cpuapp",
        "source": "devices/central/firmware/v1",
    },
}

DEFAULT_DEVICE = "soil-sensor"
DEFAULT_BUILD_DIR = "build"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build device firmware with west.",
        epilog="Extra arguments after `--` are forwarded verbatim to `west build`.",
    )
    parser.add_argument(
        "--device",
        choices=sorted(DEVICES),
        default=DEFAULT_DEVICE,
        help=f"Device firmware to build (default: {DEFAULT_DEVICE}).",
    )
    parser.add_argument(
        "--board",
        help="Override the west board target.",
    )
    parser.add_argument(
        "--build-dir",
        default=DEFAULT_BUILD_DIR,
        help=f"Build directory relative to the repo root (default: {DEFAULT_BUILD_DIR}).",
    )
    parser.add_argument(
        "-p",
        "--pristine",
        action="store_true",
        help="Force a pristine build (equivalent to --pristine=always).",
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
        help="Additional arguments forwarded to `west build` after `--`.",
    )
    return parser.parse_args()


def build_command(args: argparse.Namespace) -> list[str]:
    device = DEVICES[args.device]
    board = args.board or device["board"]
    command = [
        "uv",
        "run",
        "west",
        "build",
        "-b",
        board,
        device["source"],
        f"--build-dir={args.build_dir}",
    ]
    if args.pristine:
        command.append("--pristine=always")
    command.extend(args.west_args)
    return command


def main() -> int:
    args = parse_args()
    command = build_command(args)

    print(" ".join(command), file=sys.stderr)
    if args.dry_run:
        return 0

    result = subprocess.run(command, cwd=REPO_ROOT, env=os.environ.copy())
    if result.returncode == 0:
        print(f"Build succeeded: {REPO_ROOT / args.build_dir}", file=sys.stderr)
    else:
        print(f"Build failed with exit code {result.returncode}", file=sys.stderr)
    return result.returncode


if __name__ == "__main__":
    sys.exit(main())
