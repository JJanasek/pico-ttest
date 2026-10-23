# TROPIC01 TVLA capture (ChipWhisperer-Husky or PicoScope + ESP32)

Fixed-vs-random TVLA trace collection against a TROPIC01 secure element driven by an ESP32.
Default scope is a ChipWhisperer-Husky (`--scope husky`); a PicoScope path is under
`--scope pico` (3000 Series by default; `--pico-series 6000` for a 6404D, `2000` for the ps2000a models).

## Layout

| File | Role |
| --- | --- |
| [tvla_capture.py](tvla_capture.py) | main script: flashes the target, arms the scope, writes a `.trs` |
| [tropic_target.py](tropic_target.py) | firmware build/upload + serial client for the target |
| [husky.py](husky.py) / [pico.py](pico.py) | scope wrappers (Husky / PicoScope) |
| [trigger_check.py](trigger_check.py) | diagnostic: what the trigger input sees during one signature |
| [sign_loop.py](sign_loop.py) | sign in a loop without capturing, to set the scope up by hand in its GUI |
| [merge_trs.py](merge_trs.py) | reassemble `--traces-per-file` parts into one set |
| [trs_info.py](trs_info.py) | print a set's metadata (key, scope setup, per-trace fields) |
| [firmware/](firmware/) | PlatformIO project (`esp32dev_tvla`) that builds the capture firmware |

## Quick start

```sh
source ~/.cwvenv/bin/activate          # chipwhisperer env, for --scope husky
python trigger_check.py --trigger-pin tio4        # verify the trigger first
python sign_loop.py                               # or: keep signing (Ctrl-C stops) and watch in the GUI
python tvla_capture.py -n 3000 -c ed -m message   # collect (flashes firmware unless --no-flash)
python trs_info.py traces/*.trs                    # inspect what was recorded
```

## Wiring

### Husky (default)

Trigger is a **logic pin**, so the 3.3 V GPIO drives it directly (no threshold, no probe attenuation):

| ESP32 | Husky 20-pin header |
| --- | --- |
| `GPIO4` (`TVLA_TRIGGER_PIN`) | pin 16 — TIO4 (`--trigger-pin` for others) |
| `GND` | pin 17 or 19 — GND |

Leave Husky's power pins (+3.3 V, +5 V) unconnected — the ESP32 has its own supply. GPIO4 is only
free while `LT_USE_INT_PIN` is off (else it is TROPIC01's INT pin and the build fails).

**Measurement:** the two MEASURE SMAs feed one AC-coupled LNA (gain −6.5…55 dB via `--gain-db`).
Measure the **TROPIC01 side of the shunt, referenced to board 0 V** — single-ended, one MEASURE
port, short-circuit cap on the other. Never land a shield/ground clip on the supply side of a
high-side shunt or the chip side of a low-side one: the 20-pin cable already ties Husky ground to
board ground, so that shorts the shunt (and, high-side, the 3.3 V rail). Using both MEASURE ports
**differentially** across the shunt rejects rail noise (needs a second SMA cable).

### PicoScope (`--scope pico`)

GPIO4 → **Ext** on a D model such as the 3406D (`--trigger-source ext`, the default). Ext leaves
only the measured channel enabled, so it can run at 1 GS/s; a channel trigger halves that. A/B
models have no Ext - use a channel there, `--trigger-source B`. Shunt/EM
probe → channel A, AC coupled (`--coupling AC --volt-div 0.01`). Mind ×10 probe attenuation on both
the trigger (drop `--trigger-level`) and the measurement (use ×1).

**6404D** (`--pico-series 6000`, ps6000 driver; the newer 6000E models use ps6000a and are not
covered). Here "ext" is the **AUX** BNC: 50 Ω, with a fixed ±1 V threshold range. Don't wire the GPIO
straight to it, because 3.3 V into 50 Ω asks the ESP32 pin for ~66 mA. Put a ~470 Ω resistor in series
instead (≈0.3 V at the input); `--trigger-level` defaults to 0.15 V on AUX. Alternatively, trigger on
a 1 MΩ channel (`--trigger-source B`, the usual 1.5 V). Other differences:

* AC coupling exists only at 1 MΩ, so keep the shunt on `--coupling AC`. `--coupling DC50` is for
  an amplifier output, never a shunt that carries DC.
* The smallest range is ±50 mV, which is the `--volt-div 0.01` default.
* `--bw-limit` is 25 MHz (not 20).
* The timebase grid is 156.25 MHz / n, so 12.5 MS/s lands on 13.02 MS/s. The achieved rate is the
  one recorded; `--sample-rate 15.625e6` hits the grid exactly.

### Serial

ESP32 is `/dev/ttyUSB0` (`--port`). A plugged-in Husky claims a `/dev/ttyACM*` — that is not the target.

## Capture options

* **Gain** — set from the run's own warnings, not calculation. It flags >0.1% clipped (too high)
  and <¼ ADC range used (too low). A residual ~0.006% always clips: ESP32 SPI-polls `CHIP_STATUS`
  every 25 ms, spiking the shared supply; host-induced, equal in both classes, mask it in analysis.
* **`--skip-ms`** discards the trigger edge (which couples in and clips the ADC) so the gain can be
  set for the signature.
* **Depth vs bandwidth** — Husky holds 131,070 samples regardless of rate (traced window ~101 ms):
  | goal | how |
  | --- | --- |
  | full signature, moderate BW | `--stream` (default) 10 MS/s, `--samples 1.05e6` = 105 ms, decimate 1 |
  | short window, full BW | `--no-stream --sample-rate 200e6 --samples 131070` = 0.66 ms |
  | wide span **and** full BW | `--tiles N` |

  `--tiles N` captures N consecutive windows and concatenates them (stepping `adc.offset`), one Sign
  per tile with the same key+message. Tiles are separate executions — per-sample TVLA is fine, but
  analysis must not cross a tile boundary. Use **100 MS/s / 39 tiles** for campaigns (200 MS/s has
  no FIFO drain margin and wedges the device after a few hundred windows; only a replug clears it).
* **`--traces-per-file N`** rolls to a new file every N traces (a `.trs` is only readable once
  closed); rejoin with `merge_trs.py`.

## Firmware

`tvla_capture.py` flashes `esp32dev_tvla` each run unless `--no-flash`. It builds from
`~/Documents/PlatformIO/Projects/Tropic test` when that project exists (the lab machine's), else
from the in-repo `firmware/`; `--project-dir` overrides both. By hand:

```sh
pio run -d firmware -e esp32dev_tvla -t upload
```

**Serial protocol** — ASCII, newline terminated (`+…` ok, `-ERR …` fail, `#…` info):

| Command | Meaning |
| --- | --- |
| `p` / `v` | ping → `+PONG` / version → `+VERSION tvla-target 3` |
| `g ed` / `g ec` | erase slot + generate Ed25519 / P256 key → `+OK <pubkey hex>` |
| `k ed <hex>` / `k ec <hex>` | erase slot + store the given 32-byte private key → `+OK` |
| `s <hex>` | sign with the trigger asserted → `+OK <signature hex> <l3 nonce hex>` |
| `e` | erase the active slot → `+OK` |

The trigger is raised for exactly the libtropic sign call, bracketing the whole L3 Sign transaction
(~101 ms pulse; the host-measured ~123 ms includes serial round-trip).

## What each trace file records

* **Campaign metadata** (`TRACE_SET_PARAMETERS`, once per file): target, curve, mode, public key
  (message mode) or fixed scalar (scalar mode), the full scope setup, firmware version, this code's
  git commit, a UTC timestamp, and the exact command line.
* **Per-trace fields**: `ttest` (0 = fixed / 1 = random), `msg` (bytes signed), `sig` (signature),
  `nonce` (the 12-byte secure-channel L3 IV for that Sign — differs per trace even for the same
  message), and in `--mode scalar` also `key` (the scalar written that trace).

Message mode generates the key on-chip, so only the **public** key is knowable. Pass
`--priv-key <64 hex>` to store a known key instead (recorded as `private_key`) — ground truth for
validating a recovered key. `trs_info.py` reads all of this back (structured params survive a
viewer re-saving the file, unlike the free-text description).

## Two modes

* **`-m message`** (default): one key for the whole run; the signed payload is `FIXED_PAYLOAD` or
  random. For Ed25519 the nonce `r = H(h_b ‖ M)` follows the message, so the classes differ in the
  nonce too.
* **`-m scalar`**: message fixed; the stored key is `FIXED_SCALAR` or random, rewritten per trace
  (outside the trigger window). Targets the scalar multiplication. ⚠️ one slot write per trace —
  check the ECC-slot write-endurance figure before long runs.

## Troubleshooting

**`Tropic01.begin()` / `sign failed` `ret=10 (LT_L1_CHIP_ALARM_MODE)`** — TROPIC01 tripped its tamper
detection and refuses to operate. **Only a full power cycle clears it** (unplug the ESP32's USB;
an EN reset leaves the shield powered). Alarms follow bursts of rapid signing, so `--tiles`
provokes them most; `--sign-delay` paces the run. If it re-alarms immediately, the shunt is
perturbing the supply — shrink it, move it to the ground return, or use an EM probe. Never rewire
the measurement with the rail live.

**`ret=11 (LT_L1_CHIP_BUSY)`** — no SPI response. Check CS on GPIO5, SCK/MISO/MOSI, shield power,
common grounds.

**"scope did not trigger"** — run `trigger_check.py --trigger-pin tio4`. A flat line = GPIO not
driving / wrong pin / grounds not common / pin contended. `LIBUSB_ERROR_OVERFLOW` means the Husky
desynced and needs a replug.

**PlatformIO "CMakeCache.txt directory is different"** — CMake baked an old absolute path. From
`firmware/`, delete the generated build dirs and let them regenerate:

```sh
find .pio/libdeps -maxdepth 3 -type d \( -name libtropic_build -o -name hal_cal_vars_build \) -exec rm -rf {} +
```

## Requirements

`numpy`, `pyserial`, `trsfile`, PlatformIO CLI. For `--scope husky`: `chipwhisperer` (in `~/.cwvenv`).
For `--scope pico`: PicoSDK + `picosdk` bindings, with the driver for your series (`libps3000a`,
`libps2000a` or `libps6000`). Only the selected series' library is loaded; if the SDK sits outside
the linker path, point `LD_LIBRARY_PATH` at its `lib/`.
