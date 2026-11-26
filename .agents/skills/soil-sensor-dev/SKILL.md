---
name: soil-sensor-dev
description: Build, flash, monitor over serial, and read the BLE beacon of the nRF5340 soil-sensor firmware. Use when developing, flashing, or debugging the soil sensor or central devices.
---

# Soil sensor development tools

This repo ships helper scripts under `tools/scripts/` that wrap the common
build/flash/monitor/scan workflow. Run them with `uv run` from the repository
root so the project environment is used.

The firmware targets the u-blox EVK-NORA-B10 (nRF5340). Default device is
`soil-sensor`; `central` is the gateway device. Both share the board
`ubx_evknorab10/nrf5340/cpuapp`.

## Build

```
uv run tools/scripts/west_build.py
uv run tools/scripts/west_build.py --device central
uv run tools/scripts/west_build.py --pristine            # force clean build
uv run tools/scripts/west_build.py --pristine -- cmake -DCONF_FILE=prj_release.conf
```

Options:
- `--device {soil-sensor,central}` (default `soil-sensor`)
- `--board <target>` override the west board target
- `--build-dir <dir>` (default `build`)
- `-p` / `--pristine` force a pristine build
- `-n` / `--dry-run` print the command without running it
- Anything after `--` is forwarded verbatim to `west build` (e.g. cmake args)

Always check the exit code and full output before flashing. A successful
build prints `Build succeeded: <repo>/build` on stderr.

## Flash

Run after a successful build. The device must be connected via the J-Link
debug probe.

```
uv run tools/scripts/west_flash.py
uv run tools/scripts/west_flash.py --build-dir build
```

Options:
- `--build-dir <dir>` (default `build`)
- `-n` / `--dry-run` print the command without running it
- Anything after `--` is forwarded to `west flash` (e.g. `--runner`,
  `--dev-id`)

## Serial monitor (pyserial)

Reads serial output and prints timestamped lines. Non-interactive and bounded
by `--duration` so it is suitable for agent use. Auto-detects the first known
debug probe by VID/PID, falling back to the first USB CDC device.

```
uv run tools/scripts/serial_monitor.py --list-ports
uv run tools/scripts/serial_monitor.py
uv run tools/scripts/serial_monitor.py --port /dev/ttyACM0 --duration 30
uv run tools/scripts/serial_monitor.py --match "soil" --log-file output.log
```

Options:
- `--port <device>` (default: auto-detect)
- `--baudrate <bps>` (default 115200)
- `--duration <sec>` seconds to monitor before exiting (default 10; 0 = until
  Ctrl-C)
- `--match <regex>` only print lines matching this pattern
- `--log-file <path>` append all received lines to this file
- `--list-ports` list available serial ports and exit

Use this to capture boot logs and runtime logs for debugging. If no port is
detected, run with `--list-ports` and pass `--port` explicitly.

## Read BLE beacon

Scans for the soil sensor BLE manufacturer beacon, validates its truncated
HMAC-SHA256 tag, and prints authenticated raw readings. Use this to verify the
sensor advertises and its measurements validate without touching the device.

The firmware advertises **exactly once** on boot for ~5 seconds, then idles.
To re-read it, reset the board (see "Reset board" below) while the scanner is
running. Start the scanner first, then reset so the advertisement window
overlaps the scan. Multiple resets may be needed — BLE passive scanning can
miss a short non-connectable advertisement, and `freshness=stale` or
`replayed_boot_nonce` lines are suppressed, so only the first fresh packet per
boot prints.

```sh
# Terminal A: scanner (auto-exits after --timeout seconds)
uv run tools/scripts/read_soil_beacon.py --timeout 20

# Terminal B (while A is scanning): reset to re-advertise
uv run tools/scripts/reset_board.py
```

Options:
- `--address <BLE address>` only report a specific sensor
- `--company-id <dec or hex>` (default `0xFFFF`)
- `--timeout <sec>` seconds to scan before exiting (default 5; 0 = until Ctrl-C)
- `--auth-key-hex <hex>` HMAC key, must match the firmware key (default
  `2a5df318c74091e653ac743910bd86cf`, 16 bytes / 32 hex chars)

Each line shows `address rssi dev boot freshness raw0 raw1 raw2`. Lines with
`invalid authentication tag` or `unexpected payload length` indicate a
misconfigured key or a non-matching advertiser.

## Reset board

Reset the nRF5340 app core via the SEGGER J-Link debug probe. The firmware
advertises its BLE beacon only once on boot, so reset to re-trigger it without
power-cycling by hand.

```sh
uv run tools/scripts/reset_board.py
uv run tools/scripts/reset_board.py --serial 000960171887
uv run tools/scripts/reset_board.py --device nRF5340_XXAA_NET
```

Options:
- `--serial <SN>` J-Link serial number (default `000960171887`)
- `--device <target>` J-Link target (default `nRF5340_XXAA_APP`)
- `--interface <if>` (default `SWD`)
- `--speed <kHz>` (default 4000)
- `--halt` halt the core after reset instead of letting it run

Requires `JLinkExe` on PATH.

## Typical workflow

1. `uv run tools/scripts/west_build.py --pristine` — clean build
2. `uv run tools/scripts/west_flash.py` — flash to the connected device
3. `uv run tools/scripts/serial_monitor.py --duration 15` — capture boot +
   runtime logs (console is on the J-Link CDC, e.g. `/dev/ttyACM3`)
4. Start `uv run tools/scripts/read_soil_beacon.py --timeout 20`, then run
   `uv run tools/scripts/reset_board.py` in another terminal to re-advertise
   and capture an authenticated reading
