# TROPIC01 TVLA capture firmware

Host-driven ESP32 firmware for side-channel trace collection against a TROPIC01 secure element.
It does nothing on its own: it waits for line commands on the serial port, performs one signature
per command, and raises `TVLA_TRIGGER_PIN` (GPIO4 by default) for exactly the duration of the
libtropic sign call so an oscilloscope can capture that window.

Driven by [`../tvla_capture.py`](../tvla_capture.py). Full wiring, the fixed-vs-random campaign
design, and the capture tooling are documented in [`../README.md`](../README.md).

## Running it on your own machine (PlatformIO)

Everything is in the repo — the firmware project is this `firmware/` directory, and PlatformIO
pulls [libtropic-arduino](https://github.com/tropicsquare/libtropic-arduino) in automatically
(`lib_deps` in `platformio.ini`). Nothing has to be placed by hand.

```
firmware/
  platformio.ini        env esp32dev_tvla: board esp32dev, Arduino framework, pulls libtropic
  src/tvla_target.cpp   the firmware (builds standalone; no other sources)
```

`../tvla_capture.py` builds from this directory automatically. (It only prefers an external
`~/Documents/PlatformIO/Projects/Tropic test` when that exists — the lab machine's layout; on a
fresh clone it does not, so `firmware/` is used. `--project-dir` overrides either way.)

**1. Install PlatformIO** — the VS Code "PlatformIO IDE" extension, or the CLI: `pip install
platformio` (gives the `pio` command).

**2. Clone the repo** and find the ESP32's serial port:
* Linux: `/dev/ttyUSB0` (or `/dev/ttyACM0`) — add yourself to the port group once:
  `sudo usermod -aG dialout $USER`, then re-login.
* macOS: `/dev/cu.usbserial-*` or `/dev/cu.SLAB_USBtoUART`.
* Windows: `COM3`, `COM4`, …

**3. Build and flash** (set the port to yours):

```sh
pio run -d firmware -e esp32dev_tvla -t upload --upload-port /dev/ttyUSB0
```

The first build downloads the ESP32 toolchain and libtropic (a few minutes); later builds are
fast. You can also skip this step — `tvla_capture.py` flashes the firmware itself each run unless
`--no-flash` is given. Set the default port in `platformio.ini` (`upload_port`) to avoid passing
it every time.

**4. Verify** the board is up:

```sh
pio device monitor -d firmware -b 115200
```

Type `v` → `+VERSION tvla-target 3`, `p` → `+PONG`. Ctrl-C to exit. (Only one program can hold the
port — close the monitor before running a capture.)

**5. Wire it up** — TROPIC01 shield on the ESP32 (SPI), `GPIO4` → scope trigger, shunt/EM probe →
scope channel. Full wiring and the capture workflow are in [`../README.md`](../README.md).

**Troubleshooting.** "port busy / permission denied" → wrong port, monitor still open, or not in
`dialout`. `-ERR ...` at boot instead of a prompt → `Tropic01.begin()` failed: check the shield's
SPI wiring and power. `CMakeCache.txt directory is different` (after moving the clone) →
`find firmware/.pio/libdeps -maxdepth 3 -type d \( -name libtropic_build -o -name hal_cal_vars_build \) -exec rm -rf {} +`.

## Serial protocol

ASCII, newline terminated. Replies: `+…` success, `-ERR …` failure, `#…` informational.

| Command | Meaning |
| --- | --- |
| `p` | ping → `+PONG` |
| `v` | version → `+VERSION tvla-target 3` |
| `g ed` / `g ec` | erase slot + generate Ed25519 / P256 key → `+OK <pubkey hex>` |
| `k ed <hex>` / `k ec <hex>` | erase slot + store the given 32-byte private key → `+OK` |
| `s <hex>` | sign payload with the trigger asserted → `+OK <signature hex> <l3 nonce hex>` |
| `e` | erase the active slot → `+OK` |

## Trigger

`TVLA_TRIGGER_PIN` (GPIO4, build-time configurable) is a digital output, driven directly around
the sign call:

```c
digitalWrite(TVLA_TRIGGER_PIN, HIGH);
ret = tropic01.eddsaSign(activeSlot, msgBuf, msgLen, sigBuf);
digitalWrite(TVLA_TRIGGER_PIN, LOW);
```

The rising edge is asserted immediately before the driver dispatches the L3 Sign command over SPI;
the falling edge follows the blocking call's return, after the chip has computed and the signature
has been read back. The window therefore brackets the whole host-side sign transaction.
