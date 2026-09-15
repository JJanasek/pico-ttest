# TROPIC01 TVLA capture firmware

Host-driven ESP32 firmware for side-channel trace collection against a TROPIC01 secure element.
It does nothing on its own: it waits for line commands on the serial port, performs one signature
per command, and raises `TVLA_TRIGGER_PIN` (GPIO4 by default) for exactly the duration of the
libtropic sign call so an oscilloscope can capture that window.

Driven by [`../tvla_capture.py`](../tvla_capture.py). Full wiring, the fixed-vs-random campaign
design, and the capture tooling are documented in [`../README.md`](../README.md).

## Build and flash

```sh
pio run -e esp32dev_tvla -t upload --upload-port /dev/ttyUSB0
```

Depends on [libtropic-arduino](https://github.com/tropicsquare/libtropic-arduino) (pulled in by
`platformio.ini`).

## Serial protocol

ASCII, newline terminated. Replies: `+…` success, `-ERR …` failure, `#…` informational.

| Command | Meaning |
| --- | --- |
| `p` | ping → `+PONG` |
| `v` | version → `+VERSION tvla-target 2` |
| `g ed` / `g ec` | erase slot + generate Ed25519 / P256 key → `+OK <pubkey hex>` |
| `k ed <hex>` / `k ec <hex>` | erase slot + store the given 32-byte private key → `+OK` |
| `s <hex>` | sign payload with the trigger asserted → `+OK <signature hex>` |
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
