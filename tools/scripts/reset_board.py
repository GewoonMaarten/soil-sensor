#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Reset the nRF5340 soil-sensor board via SEGGER J-Link so it re-advertises.

The firmware advertises its BLE beacon exactly once on boot, then idles. To
re-read the beacon without power-cycling by hand, reset the app core through
the connected J-Link debug probe.

Examples:
    uv run tools/scripts/reset_board.py
    uv run tools/scripts/reset_board.py --serial 000960171887
    uv run tools/scripts/reset_board.py --device nRF5340_XXAA_NET
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys

DEFAULT_SERIAL = "000960171887"
DEFAULT_DEVICE = "nRF5340_XXAA_APP"
DEFAULT_SPEED = 4000
DEFAULT_IF = "SWD"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reset the board via SEGGER J-Link.")
    parser.add_argument(
        "--serial",
        default=DEFAULT_SERIAL,
        help=f"J-Link serial number (default: {DEFAULT_SERIAL}).",
    )
    parser.add_argument(
        "--device",
        default=DEFAULT_DEVICE,
        help=f"J-Link target device (default: {DEFAULT_DEVICE}; use nRF5340_XXAA_NET for the network core).",
    )
    parser.add_argument(
        "--interface",
        default=DEFAULT_IF,
        help=f"Debug interface (default: {DEFAULT_IF}).",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=DEFAULT_SPEED,
        help=f"J-Link speed in kHz (default: {DEFAULT_SPEED}).",
    )
    parser.add_argument(
        "--halt",
        action="store_true",
        help="Halt the core after reset instead of letting it run.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not shutil.which("JLinkExe"):
        print("JLinkExe not found on PATH.", file=sys.stderr)
        return 1

    commands = "r\n" + ("" if not args.halt else "h\n") + "g\nq\n"
    cmd = [
        "JLinkExe",
        "-device",
        args.device,
        "-if",
        args.interface,
        "-speed",
        str(args.speed),
        "-SelectEmuBySN",
        args.serial,
        "-AutoConnect",
        "1",
    ]
    print(" ".join(cmd), file=sys.stderr)
    result = subprocess.run(
        cmd,
        input=commands,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=30,
    )
    # JLinkExe prints a lot; surface the reset outcome lines.
    for line in result.stdout.splitlines():
        if any(token in line for token in ("Reset", "Cortex", "Error", "error", "Cannot", "Failed")):
            print(line)
    return result.returncode if result.returncode != 0 else 0


if __name__ == "__main__":
    sys.exit(main())
