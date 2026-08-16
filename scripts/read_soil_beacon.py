#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "bleak",
# ]
# ///
"""Read raw soil sensor values from the BLE manufacturer payload.

The firmware advertises authenticated manufacturer data with:
- company_id (optional in parser; scanner may expose it separately)
- device_id
- boot_nonce
- sample_counter
- three little-endian signed 32-bit raw measurements
- truncated HMAC-SHA256 authentication tag
"""

from __future__ import annotations

import argparse
import asyncio
import hmac
import hashlib
import struct
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime

from bleak import BleakScanner

DEFAULT_COMPANY_ID = 0xFFFF
DEFAULT_AUTH_KEY_HEX = "2a5df318c74091e653ac743910bd86cf"
FULL_PAYLOAD_FMT = "<HHIH3iI"
NO_COMPANY_PAYLOAD_FMT = "<HIH3iI"
TAG_SIZE = 4
MAX_RECENT_BOOT_NONCES = 16


@dataclass(slots=True)
class BeaconPacket:
    company_id: int
    device_id: int
    boot_nonce: int
    sample_counter: int
    raw_values: tuple[int, int, int]
    auth_tag: bytes
    signed_payload: bytes


@dataclass(slots=True)
class FreshnessState:
    current_boot_nonce: int
    last_counter: int
    seen_boot_nonces: deque[int] = field(default_factory=lambda: deque(maxlen=MAX_RECENT_BOOT_NONCES))


def parse_auth_key_hex(value: str) -> bytes:
    try:
        parsed = bytes.fromhex(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("auth key must be a valid hex string") from exc

    if len(parsed) != 16:
        raise argparse.ArgumentTypeError("auth key must be exactly 16 bytes (32 hex chars)")

    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Scan for the soil sensor BLE beacon and print raw readings."
    )
    parser.add_argument(
        "--address",
        help="Optional BLE address filter for a specific sensor.",
    )
    parser.add_argument(
        "--company-id",
        type=lambda value: int(value, 0),
        default=DEFAULT_COMPANY_ID,
        help="Manufacturer company ID in decimal or hex (default: 0xFFFF).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Scanner callback lifetime in seconds per run loop iteration.",
    )
    parser.add_argument(
        "--auth-key-hex",
        type=parse_auth_key_hex,
        default=parse_auth_key_hex(DEFAULT_AUTH_KEY_HEX),
        help="HMAC key as hex string (must match firmware key).",
    )
    return parser.parse_args()


def decode_payload(company_id: int, payload: bytes) -> BeaconPacket | None:
    if len(payload) == struct.calcsize(FULL_PAYLOAD_FMT):
        parsed = struct.unpack(FULL_PAYLOAD_FMT, payload)
        embedded_company_id = parsed[0]
        if embedded_company_id != company_id:
            return None
        signed_payload = payload[:-TAG_SIZE]
        auth_tag = payload[-TAG_SIZE:]
        return BeaconPacket(
            company_id=embedded_company_id,
            device_id=parsed[1],
            boot_nonce=parsed[2],
            sample_counter=parsed[3],
            raw_values=(parsed[4], parsed[5], parsed[6]),
            auth_tag=auth_tag,
            signed_payload=signed_payload,
        )

    if len(payload) == struct.calcsize(NO_COMPANY_PAYLOAD_FMT):
        parsed = struct.unpack(NO_COMPANY_PAYLOAD_FMT, payload)
        signed_payload = struct.pack("<H", company_id) + payload[:-TAG_SIZE]
        auth_tag = payload[-TAG_SIZE:]
        return BeaconPacket(
            company_id=company_id,
            device_id=parsed[0],
            boot_nonce=parsed[1],
            sample_counter=parsed[2],
            raw_values=(parsed[3], parsed[4], parsed[5]),
            auth_tag=auth_tag,
            signed_payload=signed_payload,
        )

    return None


def has_valid_auth_tag(packet: BeaconPacket, auth_key: bytes) -> bool:
    expected_tag = hmac.new(auth_key, packet.signed_payload, hashlib.sha256).digest()[:TAG_SIZE]
    return hmac.compare_digest(expected_tag, packet.auth_tag)


def freshness_status(packet: BeaconPacket, states: dict[int, FreshnessState]) -> str:
    state = states.get(packet.device_id)
    if state is None:
        states[packet.device_id] = FreshnessState(packet.boot_nonce, packet.sample_counter)
        return "fresh"

    if packet.boot_nonce == state.current_boot_nonce:
        if packet.sample_counter > state.last_counter:
            state.last_counter = packet.sample_counter
            return "fresh"
        if packet.sample_counter == state.last_counter:
            return "duplicate"
        return "stale"

    if packet.boot_nonce in state.seen_boot_nonces:
        return "replayed_boot_nonce"

    state.seen_boot_nonces.append(state.current_boot_nonce)
    state.current_boot_nonce = packet.boot_nonce
    state.last_counter = packet.sample_counter
    return "fresh_new_boot"


async def main() -> None:
    args = parse_args()
    auth_key: bytes = args.auth_key_hex


    freshness_states: dict[int, FreshnessState] = {}

    def detection_callback(device, advertisement_data) -> None:
        if args.address and device.address.lower() != args.address.lower():
            return

        payload = advertisement_data.manufacturer_data.get(args.company_id)
        if payload is None:
            return

        packet = decode_payload(args.company_id, payload)
        if packet is None:
            print(
                f"{datetime.now().isoformat(timespec='seconds')} "
                f"{device.address} unexpected payload length={len(payload)} bytes"
            )
            return

        if not has_valid_auth_tag(packet, auth_key):
            print(
                f"{datetime.now().isoformat(timespec='seconds')} "
                f"{device.address} invalid authentication tag"
            )
            return

        freshness = freshness_status(packet, freshness_states)
        if freshness in ("duplicate", "stale", "replayed_boot_nonce"):
            return

        print(
            f"{datetime.now().isoformat(timespec='seconds')} "
            f"{device.address} rssi={advertisement_data.rssi} "
            f"dev=0x{packet.device_id:04X} "
            f"boot=0x{packet.boot_nonce:08X} "
            f"ctr={packet.sample_counter} "
            f"freshness={freshness} "
            f"raw0={packet.raw_values[0]} raw1={packet.raw_values[1]} raw2={packet.raw_values[2]}"
        )

    scanner = BleakScanner(detection_callback=detection_callback)

    while True:
        await scanner.start()
        await asyncio.sleep(args.timeout)
        await scanner.stop()


if __name__ == "__main__":
    asyncio.run(main())
