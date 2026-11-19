"""Monitor device serial output for debugging.

Reads a serial port and prints timestamped lines. Designed for agent use:
non-interactive, bounded by --duration, with optional pattern filtering and
capture to a log file.

Examples:
    uv run tools/scripts/serial_monitor.py --list-ports
    uv run tools/scripts/serial_monitor.py
    uv run tools/scripts/serial_monitor.py --port /dev/ttyACM0 --duration 30
    uv run tools/scripts/serial_monitor.py --match "soil" --log-file output.log
"""

from __future__ import annotations

import argparse
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

import serial
from serial.tools import list_ports

DEFAULT_BAUDRATE = 115200
DEFAULT_DURATION = 10.0
VID_PIDS = {
    (0x1366, 0x1055): "SEGGER J-Link CDC (nRF DW/VCOM)",
    (0x1366, 0x0101): "SEGGER J-Link CDC (nRF52)",
    (0x1915, 0x521F): "Nordic Semiconductor CDC",
    (0x1915, 0x5320): "Nordic Semiconductor CDC",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor device serial output.")
    parser.add_argument(
        "--port",
        help="Serial port (default: auto-detect the first known debug probe).",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"Baud rate (default: {DEFAULT_BAUDRATE}).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION,
        help=f"Seconds to monitor before exiting (default: {DEFAULT_DURATION}; 0 = until Ctrl-C).",
    )
    parser.add_argument(
        "--match",
        help="Only print lines matching this regular expression.",
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        help="Write all received lines to this file.",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List available serial ports and exit.",
    )
    return parser.parse_args()


def detect_port() -> str | None:
    ports = list_ports.comports()
    for known in VID_PIDS:
        for port in ports:
            if (port.vid, port.pid) == known:
                return port.device
    return ports[0].device if ports else None


def print_ports() -> int:
    ports = list_ports.comports()
    if not ports:
        print("No serial ports found.", file=sys.stderr)
        return 1
    for port in ports:
        known = VID_PIDS.get((port.vid, port.pid), "")
        extras = " ".join(
            f"{key}={value}" for key, value in (known, port.description) if value
        )
        print(f"{port.device}\t{extras}")
    return 0


def read_serial(args: argparse.Namespace) -> int:
    port_name = args.port or detect_port()
    if not port_name:
        print("No serial port found. Use --list-ports.", file=sys.stderr)
        return 1

    pattern = re.compile(args.match) if args.match else None
    log = args.log_file.open("a", encoding="utf-8") if args.log_file else None

    print(f"Opening {port_name} at {args.baudrate} baud.", file=sys.stderr)
    deadline = None
    if args.duration > 0:
        deadline = datetime.now(timezone.utc).timestamp() + args.duration

    try:
        with serial.Serial(port_name, args.baudrate, timeout=0.1) as port:
            while True:
                if deadline is not None and datetime.now(timezone.utc).timestamp() > deadline:
                    break
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").rstrip("\r\n")
                timestamp = datetime.now(timezone.utc).strftime("%H:%M:%S.%f")[:-3]
                if pattern is None or pattern.search(line):
                    print(f"[{timestamp}] {line}", flush=True)
                if log:
                    log.write(f"[{timestamp}] {line}\n")
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1
    finally:
        if log:
            log.close()
    return 0


def main() -> int:
    args = parse_args()
    if args.list_ports:
        return print_ports()
    return read_serial(args)


if __name__ == "__main__":
    sys.exit(main())
