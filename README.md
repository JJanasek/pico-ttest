# TROPIC01 TVLA capture (ChipWhisperer-Husky or PicoScope 3000 + ESP32)

Fixed-vs-random TVLA trace collection against a TROPIC01 secure element driven by an ESP32.
Capture runs on a ChipWhisperer-Husky by default (`--scope husky`); the PicoScope 3000 path is
still there under `--scope pico`.

## Layout

| File | Role |
| --- | --- |
| [tvla_capture.py](tvla_capture.py) | main script: flashes the target, arms the scope, collects traces into a `.trs` file |
| [tropic_target.py](tropic_target.py) | PlatformIO build/upload wrapper + serial client for the capture firmware |
| [husky.py](husky.py) | ChipWhisperer-Husky capture wrapper, same shape as the PicoScope ones |
| [pico.py](pico.py) | PicoScope block-mode wrappers (`pico3000` / `pico5000` / `pico6000`) |
| [trigger_check.py](trigger_check.py) | diagnostic: reports what the trigger input sees during one signature |
| [pico3000.py](pico3000.py) | old ChipWhisperer + STM32 script, kept for reference only |
| `~/Documents/PlatformIO/Projects/Tropic test/src/tvla_target.cpp` | capture firmware (env `esp32dev_tvla`) |

## Hardware setup

### ChipWhisperer-Husky (default)

**Trigger.** The Husky triggers on a logic pin, not on an analog channel, so a 3.3 V GPIO drives
it directly - there is no threshold and no probe attenuation to compensate for.

| ESP32 | Husky 20-pin header |
| --- | --- |
| `GPIO4` (`TVLA_TRIGGER_PIN`) | pin 16 - TIO4 |
| `GND` | pin 17 or 19 - GND |

Leave Husky's +3.3 V (pins 3/18) and +5 V (pins 1/20) unconnected: the ESP32 has its own supply.
`--trigger-pin` selects a different input (`tio1`-`tio3`, `nrst`, or `aux` for the AUX MCX).
GPIO4 is free only while `LT_USE_INT_PIN` is off; with it on, that pin is TROPIC01's interrupt
input and the build fails with an explicit error.

**Measurement.** The two MEASURE SMAs feed one AC-coupled low-noise amplifier, with gain from
-6.5 dB to 55 dB set by `--gain-db`. There is no channel and no volts/division.

Either way the measured point is **the TROPIC01 side of the shunt, referenced to board 0 V** -
single-ended, one MEASURE port, with the short-circuit cap fitted on the other one (it lowers
noise and improves the gain/frequency response).

* *High-side shunt* (in the 3.3 V line into TROPIC01): the chip's supply node is
  `V_chip = 3V3 - I*R`, and the 3.3 V rail is stiff, so the AC part of that node *is* the shunt
  drop, inverted. The input's AC coupling blocks the 3.3 V DC. Signal to the chip side of the
  resistor, shield to board 0 V.
* *Low-side shunt* (in TROPIC01's ground return): signal to the chip side of the resistor,
  shield to board 0 V - i.e. the far side of the shunt from the chip.

**Never land a shield or a probe ground clip on the supply side of a high-side shunt, or on the
chip side of a low-side one.** The 20-pin cable already ties Husky ground to board ground, so
either mistake shorts the shunt out - and on the high-side one it shorts the 3.3 V rail.

Using the two MEASURE ports **differentially** across the shunt instead is the upgrade: it takes
a second SMA cable, and it rejects noise on the 3.3 V rail that the single-ended connection
picks up along with the signal. Polarity does not matter, only the sign of the trace.

Whichever way it is connected, TROPIC01's own decoupling capacitor sits across the point being
measured and low-passes the current signature. That caps how much detail is recoverable without
lifting it.

**Gain.** Set it from measurement, not calculation. `--gain-db 39` is the default because a
sweep on this rig put the 99.99th percentile of the samples at 120 of 127 - the most amplitude
available before real signal starts hitting the rail:

| `--gain-db` | std | 99.9th pct | clipped |
| --- | --- | --- | --- |
| 37 | 8.1 | 69 | 0.004% |
| **39** | **9.7** | **87** | **0.006%** |
| 41 | 12.2 | 114 | 0.060% |
| 43 | 14.7 | 127 (railed) | 0.136% |

Re-derive it after any change to the probe or the shunt. The capture warns in both directions:
above 0.1% of samples clipped, and when the trace uses under a quarter of the ADC range.

A residual ~0.006% of samples clip at any gain. Those are the ESP32 polling TROPIC01's
`CHIP_STATUS` over SPI every 25 ms (libtropic's `LT_L1_READ_RETRY_DELAY_MS`), each poll putting a
narrow spike on the shared supply. They are host-induced, hit both classes equally, and are
easy to mask in analysis - which is why the capture only warns above 0.1%.

**Capture depth.** Husky's sample buffer is 131,070 samples, full stop, while the traced window
is ~101 ms. Nothing covers all of it at full resolution, so the two modes trade differently:

* **`--stream` (default)** keeps `decimate` at 1 - the ADC runs at the sample rate and every
  sample is read out, so there is no aliasing - at up to ~10 MS/s. The buffer limit is lifted,
  but a partial window is the price: the default 500,000 samples is the **first 50 ms** at
  10 MS/s. No pre-trigger samples.
* **`--no-stream`** fills the buffer and decimates from the `--adc-freq` ADC clock to reach a
  lower rate. That covers the whole signature in one window (131,070 at 800 kS/s = 164 ms), but
  decimation on Husky is plain sample-dropping with **no anti-alias filter**, so it folds
  everything up to `--adc-freq`/2 back into the band. It costs real SNR; take it only when the
  tail of the operation actually matters.

**`--skip-ms` (default 2).** The trigger GPIO's own edge couples into the measurement and is the
largest thing in the window - large enough to clip the ADC and force the gain down, starving the
signal that is wanted. `scope.adc.offset` steps past it so the gain can be set for the signature.

### PicoScope 3000 (`--scope pico`)

* ESP32 `GPIO4` -> PicoScope trigger input.
  * **Prefer an analog channel over Ext.** Only 3000 Series D models have an Ext input at all, and
    even on a 3406D (which has one) `ps3000aRunBlock` rejected it with `PICO_TRIGGER_ERROR` -
    `ps3000aSetSimpleTrigger` accepts an unusable source without complaint, so the failure only
    surfaces at arm time. Wire the trigger to a channel and pass `--trigger-source B`; that channel
    is enabled at +-5 V automatically. The scope prints its variant on connect.
  * **Mind the probe attenuation.** A x10 probe turns the 3.3 V GPIO pulse into ~0.33 V at the
    scope, well under a 1.5 V threshold. Either switch the probe to x1 or drop `--trigger-level`
    to about half the observed peak. The same applies to the measurement channel: a x10 probe
    throws away 20 dB of the signal you are trying to analyse.
* EM probe / shunt amplifier -> PicoScope **channel A**, AC coupled at +-50 mV
  (`--coupling AC --volt-div 0.01`, the defaults). AC coupling drops the DC bias so the small
  range is usable on the trace itself.

### Serial

The ESP32 enumerates on `/dev/ttyUSB0` (override with `--port`). Note that a plugged-in Husky
claims a `/dev/ttyACM*` of its own - that is not the target.

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
pio run -d "~/Documents/PlatformIO/Projects/Tropic test" -e esp32dev_tvla -t upload
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

# turn the gain up if the run warns that the trace uses too little of the ADC range
python tvla_capture.py -n 3000 -c ed -m message --gain-db 40

# the whole signature in one window, at the cost of aliasing from the decimator
python tvla_capture.py -n 3000 -c ed -m message --no-stream --sample-rate 800e3 --samples 131070

# a short slice at high bandwidth: 2.6 ms at 50 MS/s, block mode, decimate 1
python tvla_capture.py -n 3000 -c ed -m message --no-stream --sample-rate 50e6 --samples 131070

# trigger wired to TIO3 (20-pin pin 14) instead of TIO4
python tvla_capture.py -n 3000 -c ed -m message --trigger-pin tio3

# PicoScope instead: trigger from channel B, 0.21V suits a x10 probe on the 3.3V GPIO
python tvla_capture.py --scope pico -n 3000 -c ed --trigger-source B --trigger-level 0.21

# no scope attached - drive the target only, e.g. while checking the wiring
python tvla_capture.py --no-scope --no-flash -n 20
```

The trigger pulse is ~101 ms wide, while the host measures the sign call at ~123 ms - the
difference is serial round-trip, and only the ~101 ms inside the trigger has to be captured.
`trigger_check.py` reports the pulse width from Husky's own counter.

`--samples / --sample-rate` define the capture window; the defaults differ per scope because
the capture depths do. Husky takes 500,000 samples at 10 MS/s (the first 50 ms), a PicoScope
2 M at 12.5 MS/s (160 ms, the whole signature). The first captured trace reports the measured
duration and what fraction of it the window holds.

Measured throughput on this rig, at the Husky defaults: **0.39 s per trace and 500 kB on disk**,
so 1000 traces take about **6.5 minutes** and **477 MiB**, and 3000 traces about 20 minutes and
1.4 GiB. The floor is the 123 ms signature itself; the rest is arming and USB transfer.

On Husky the samples are stored as a fraction of ADC full scale rather than in volts - what full
scale is in volts depends on `--gain-db` and on the probe - so the `.trs` Y label reads
`ADC full scale`. A t-test does not care, but a `.trs` viewer will not show millivolts.

The two classes are interleaved at random per trace so slow drift affects both equally.

### Target won't come up

`tvla_capture.py` resets the board explicitly on connect so the firmware's boot banner - and any
`-ERR` in it - is never flushed before the host starts listening. Read that line first; it names
the fault. To see the raw line by hand:

```sh
python -c "from tropic_target import TropicTarget; TropicTarget(verbose=True).open()"
```

* `-ERR Tropic01.begin() failed ret=11 (LT_L1_CHIP_BUSY)` - libtropic polled `CHIP_STATUS` until
  it ran out of retries and never saw *ready*. TROPIC01 is not talking: check the SPI wiring
  (CS on GPIO5, SCK/MISO/MOSI per the libtropic ESP32 tutorial), that the shield is powered, and
  that the grounds are common.
* `-ERR Tropic01.begin() failed ret=10 (LT_L1_CHIP_ALARM_MODE)` - the opposite: SPI works and the
  chip answered, but its `CHIP_STATUS` has the ALARM bit set. TROPIC01 has tripped its own tamper
  detection and refuses to operate. **Only a full power cycle clears it** - an ESP32 EN reset is
  not enough, because the shield stays powered through it. Unplug the ESP32's USB, wait, plug it
  back in.

  If it goes straight back into alarm, the measurement setup itself is tripping the detector.
  A shunt in the supply rail is the usual cause: shrink the resistor, add decoupling on the chip
  side of it (which costs signal bandwidth), move the shunt to the ground return, or drop the
  shunt for an EM probe, which does not perturb the supply at all. TROPIC01 is a secure element
  with active countermeasures - resisting this measurement is a feature, not a fault.

  Never change the measurement wiring with the rail live; hot-plugging a probe onto a powered
  supply is itself enough of a transient to trip the alarm.

### PlatformIO: "CMakeCache.txt directory is different"

libtropic builds through CMake, and CMake bakes absolute paths into its cache. A project tree
that has moved (or came from another machine) fails the build with a path that no longer exists.
Delete the generated build directories - not the whole `.pio`, so the downloaded dependencies
survive - and let them regenerate:

```sh
cd "~/Documents/PlatformIO/Projects/Tropic test"
find .pio/libdeps -maxdepth 3 -type d \( -name libtropic_build -o -name hal_cal_vars_build \) -exec rm -rf {} +
```

### "scope did not trigger"

The capture never accepts an untriggered trace, so a timeout means no rising edge arrived. To see
what the trigger input actually gets:

```sh
# Husky: polls the trigger pin during one signature, then arms for real and reports whether it fired
python trigger_check.py --trigger-pin tio4

# PicoScope: captures the trigger channel untriggered and reports the pulse it sees
python trigger_check.py --scope pico --channel B --level 1.5
```

On Husky a flat line means the GPIO is not driving, is not on the pin you named, the grounds are
not common (the 20-pin header's pin 17/19), or the pin is contended - TROPIC01's INT line sits on
GPIO4 when `LT_USE_INT_PIN` is on. A line that reads high the whole time never produces an edge.

If `LIBUSB_ERROR_OVERFLOW` appears, the Husky's USB read has desynced; it will then fail to
identify itself (`Unknown ChipWhisperer:`) until it is unplugged and plugged back in.

On a PicoScope, a pulse that is present but too small is almost always probe attenuation (a x10
probe shows a 3.3 V GPIO as ~0.33 V) - the script suggests a level to use. With `--pre-trigger`,
the scope cannot honour the trigger until it has collected that many samples first, so the capture
holds each sign command back for the fill duration; a level that works in the PicoScope GUI but
never fires here is this, not the level.

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

`numpy`, `pyserial`, `trsfile`, and the PlatformIO CLI (auto-detected at
`~/.platformio/penv/bin/pio` when not on `PATH`).

For `--scope husky`, the `chipwhisperer` package - already set up in `~/.cwvenv`:

```sh
source ~/.cwvenv/bin/activate
```

For `--scope pico`, PicoSDK plus the `picosdk` Python bindings. Neither scope's library is
imported unless that scope is selected, so only the one you use has to be installed.
