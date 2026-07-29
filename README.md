# TROPIC01 TVLA capture (PicoScope 3000 + ESP32)

Fixed-vs-random TVLA trace collection against a TROPIC01 secure element driven by an ESP32,
captured with a PicoScope 3000. Replaces the previous ChipWhisperer-based flow.

## Layout

| File | Role |
| --- | --- |
| [tvla_capture.py](tvla_capture.py) | main script: flashes the target, arms the scope, collects traces into a `.trs` file |
| [tropic_target.py](tropic_target.py) | PlatformIO build/upload wrapper + serial client for the capture firmware |
| [pico.py](pico.py) | PicoScope block-mode wrappers (`pico3000` / `pico5000` / `pico6000`) |
| [trigger_check.py](trigger_check.py) | diagnostic: captures the trigger line untriggered during one signature |
| [pico3000.py](pico3000.py) | old ChipWhisperer + STM32 script, kept for reference only |
| `../../Documents/PlatformIO/Projects/tropic/src/tvla_target.cpp` | capture firmware (env `esp32dev_tvla`) |

## Hardware setup

* ESP32 `GPIO4` → PicoScope trigger input (change via `-DTVLA_TRIGGER_PIN=` in `platformio.ini`).
  GPIO4 is free only while `LT_USE_INT_PIN` is off; with it on, that pin is TROPIC01's interrupt
  input and the build fails with an explicit error.
  * **Prefer an analog channel over Ext.** Only 3000 Series D models have an Ext input at all, and
    even on a 3406D (which has one) `ps3000aRunBlock` rejected it with `PICO_TRIGGER_ERROR` —
    `ps3000aSetSimpleTrigger` accepts an unusable source without complaint, so the failure only
    surfaces at arm time. Wire the trigger to a channel and pass `--trigger-source B`; that channel
    is enabled at ±5 V automatically. The scope prints its variant on connect.
  * **Mind the probe attenuation.** A ×10 probe turns the 3.3 V GPIO pulse into ~0.33 V at the
    scope, well under a 1.5 V threshold. Either switch the probe to ×1 or drop `--trigger-level`
    to about half the observed peak. The same applies to the measurement channel: a ×10 probe
    throws away 20 dB of the signal you are trying to analyse.
* EM probe / shunt amplifier → PicoScope **channel A**, AC coupled at ±50 mV
  (`--coupling AC --volt-div 0.01`, the defaults). AC coupling drops the DC bias so the small
  range is usable on the trace itself.
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

# trigger from channel B; 0.21V suits a x10 probe on the 3.3V GPIO (x1 probe: use 1.5)
python tvla_capture.py -n 3000 -c ed -m message --trigger-source B --trigger-level 0.21

# explicit acquisition settings (these are the defaults): AC coupled, +-50mV on the probe
python tvla_capture.py -n 3000 -c ed -m message --coupling AC --volt-div 0.01

# no scope attached - drive the target only, e.g. while setting trigger/gain up in the Pico GUI
python tvla_capture.py --no-scope --no-flash -n 20
```

`--samples / --sample-rate` define the capture window. The default is 2 M samples at 12.5 MS/s
= 160 ms, sized around a measured TROPIC01 Ed25519 signature of ~123 ms — at 2 MB per trace that
is ~6 GB for 3000 traces, so trade `--sample-rate` against resolution if that is too much. The
first captured trace prints the measured signature duration and warns if it exceeds the window
(truncated traces are otherwise invisible in the `.trs` file).

The two classes are interleaved at random per trace so slow drift affects both equally.

### "scope did not trigger"

The capture never accepts an untriggered trace (the driver's auto-trigger is disabled), so a
timeout means no edge crossed the level. To see what the scope actually gets on the trigger line:

```sh
python trigger_check.py --channel B --level 1.5
```

It captures untriggered while the target signs once and reports the min/max level, whether a pulse
crossed the threshold, its width and where it starts — plus the host-measured signature duration,
which is what `--samples / --sample-rate` have to cover.

With `--pre-trigger`, the scope cannot honour the trigger until it has collected that many samples
first. The capture therefore holds each sign command back for the fill duration — without that, the
edge lands during the fill and is dropped, and because the line then stays high for the whole
signature no second edge ever arrives. A level that works in the PicoScope GUI but never fires here
is this, not the level.

If a pulse is there but too small, it is almost always probe attenuation (a ×10 probe shows a 3.3 V
GPIO as ~0.33 V) — the script suggests a level to use. A line that is completely flat means the GPIO
is not driving, is not connected to that channel, the probe ground is missing, or the pin is
contended by something else driving it (TROPIC01's INT line sits on GPIO4).

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
