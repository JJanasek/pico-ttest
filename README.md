# TROPIC01 TVLA capture (PicoScope 3000 + ESP32)

Fixed-vs-random TVLA trace collection against a TROPIC01 secure element driven by an ESP32,
captured with a PicoScope 3000. Replaces the previous ChipWhisperer-based flow.

## Layout

| File | Role |
| --- | --- |
| [tvla_capture.py](tvla_capture.py) | main script: flashes the target, arms the scope, collects traces into a `.trs` file |
| [tropic_target.py](tropic_target.py) | PlatformIO build/upload wrapper + serial client for the capture firmware |
| [pico.py](pico.py) | PicoScope block-mode wrappers (`pico3000` / `pico5000` / `pico6000`) |
| [pico3000.py](pico3000.py) | old ChipWhisperer + STM32 script, kept for reference only |
| `../../Documents/PlatformIO/Projects/tropic/src/tvla_target.cpp` | capture firmware (env `esp32dev_tvla`) |

## Hardware setup

* ESP32 `GPIO4` → PicoScope **EXT** trigger input (change via `-DTVLA_TRIGGER_PIN=` in `platformio.ini`).
  GPIO4 is free only while `LT_USE_INT_PIN` is off; with it on, that pin is TROPIC01's interrupt
  input and the build fails with an explicit error.
* EM probe / shunt amplifier → PicoScope **channel A**.
* ESP32 serial port on `/dev/ttyACM0` (override with `--port`).

The firmware raises the trigger pin for exactly the duration of the libtropic sign call, so the
captured window contains one signature and nothing else.

## Firmware

The PlatformIO project now has two environments:

* `esp32dev` — the original standalone EdDSA example plus its self-contained software (cycle-count)
  TVLA demo. Unchanged.
* `esp32dev_tvla` — the host-driven capture target used here. Does nothing on its own; waits for
  serial commands.

`tvla_capture.py` builds and flashes `esp32dev_tvla` on every run unless you pass `--no-flash`.
To do it by hand:

```sh
pio run -d ~/Documents/PlatformIO/Projects/tropic -e esp32dev_tvla -t upload
```

### Serial protocol (`tvla_target.cpp`)

ASCII, newline terminated. Replies: `+…` success, `-ERR …` failure, `#…` informational.

| Command | Meaning |
| --- | --- |
| `p` | ping → `+PONG` |
| `v` | version → `+VERSION tvla-target 2` |
| `g ed` / `g ec` | erase slot + generate Ed25519 / P256 key → `+OK <pubkey hex>` |
| `k ed <hex>` / `k ec <hex>` | erase slot + store the given 32-byte private key → `+OK` |
| `s <hex>` | sign payload with the trigger asserted → `+OK <signature hex>` |
| `e` | erase the active slot → `+OK` |

## Collecting traces

```sh
# EdDSA, fixed vs random MESSAGE (key generated once, stays in the slot)
python tvla_capture.py -n 3000 -c ed -m message

# EdDSA, fixed vs random SECRET SCALAR (message stays fixed, key rewritten per trace)
python tvla_capture.py -n 3000 -c ed -m scalar --no-flash

# no scope attached - drive the target only, e.g. while setting trigger/gain up in the Pico GUI
python tvla_capture.py --no-scope --no-flash -n 20
```

`--samples / --sample-rate` define the capture window (default 1 M samples at 12.5 MS/s ≈ 80 ms).
Check with a single trace that the whole signature fits inside it before starting a long campaign.
The two classes are interleaved at random per trace so slow drift affects both equally.

### What each mode varies

`-m message` — the key is generated once and stays in the slot; the signed 32-byte payload is either
`FIXED_PAYLOAD` or fresh random. For Ed25519 the nonce `r = H(h_b ‖ M)` is derived deterministically
from the message, so the two classes differ in the nonce as well as in the hashed message — this
targets the message-dependent part of the signature. For P256/ECDSA the payload is a message hash and
the nonce `k` is drawn internally at random regardless of class, so `k` is symmetric noise in both
classes: the split isolates hash-dependent leakage in `s = k⁻¹(z + r·d) mod n` at the cost of
statistical power.

`-m scalar` — the message stays `FIXED_PAYLOAD`; before every trace the host erases the slot and
writes a private key (`eccKeyStore`), either `FIXED_SCALAR` or fresh random. This targets
key-dependent leakage in the scalar multiplication. The store happens **before** the scope is armed,
so it stays outside the trigger window.

> ⚠️ `-m scalar` performs one slot erase + key write per trace (3000 traces = 3000 write cycles on
> the ECC slot). Check the TROPIC01 datasheet's write-endurance figure for ECC key slots before
> running long campaigns, and consider spreading them across slots or capping trace counts.

Each trace is tagged with the `ttest` byte parameter (`0` = fixed, `1` = random) in both modes.

## Requirements

`numpy`, `pyserial`, `trsfile`, PicoSDK + the `picosdk` Python bindings, and the PlatformIO CLI
(auto-detected at `~/.platformio/penv/bin/pio` when not on `PATH`).
