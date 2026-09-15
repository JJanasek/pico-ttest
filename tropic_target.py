## tropic_target.py
# Host side of the TROPIC01 TVLA capture target (see tropic/src/tvla_target.cpp):
#  * PlatformIOProject - builds and flashes the firmware with PlatformIO.
#  * TropicTarget      - drives the firmware over a plain serial line.
# This replaces the ChipWhisperer target/programmer that the old pico3000.py used.
import collections
import os
import shlex
import shutil
import subprocess
import time

import serial

# The PlatformIO project holding tvla_target.cpp. The in-repo `firmware/` builds it standalone,
# so a fresh clone works out of the box; override with --project-dir to build from an external
# project (e.g. one that also has the standalone esp32dev demo).
_REPO_FIRMWARE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "firmware")
_EXTERNAL = os.path.expanduser("~/Documents/PlatformIO/Projects/Tropic test")
DEFAULT_PROJECT_DIR = _REPO_FIRMWARE if os.path.isdir(_REPO_FIRMWARE) else _EXTERNAL
DEFAULT_ENV = "esp32dev_tvla"
# The ESP32's USB-serial bridge. Note that a plugged-in Husky also claims a /dev/ttyACM*, so
# do not reach for the first ACM port you see - open() rejects it explicitly.
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200

# TR01_CURVE_PRIVKEY_LEN: length of a private key / secret scalar accepted by eccKeyStore().
PRIVKEY_LEN = 32


class TargetError(Exception):
    """Raised when the target rejects a command or answers with '-ERR ...'."""


def find_pio():
    """Locates the `pio` executable, which is often not on PATH after a VSCode install."""
    candidates = [
        shutil.which("pio"),
        shutil.which("platformio"),
        os.path.expanduser("~/.platformio/penv/bin/pio"),
        os.path.expanduser("~/.platformio/penv/Scripts/pio.exe"),
    ]
    for candidate in candidates:
        if candidate and os.path.exists(candidate):
            return candidate
    raise FileNotFoundError(
        "PlatformIO CLI not found - install it or add ~/.platformio/penv/bin to PATH"
    )


class PlatformIOProject:
    """Thin wrapper around `pio run` for building/flashing the capture firmware."""

    def __init__(self, project_dir=DEFAULT_PROJECT_DIR, env=DEFAULT_ENV, port=None, pio=None):
        self.project_dir = os.path.expanduser(project_dir)
        self.env = env
        self.port = port
        self.pio = pio or find_pio()

        if not os.path.isdir(self.project_dir):
            raise FileNotFoundError(f"PlatformIO project not found: {self.project_dir}")
        if not os.path.isfile(os.path.join(self.project_dir, "platformio.ini")):
            raise FileNotFoundError(f"No platformio.ini in {self.project_dir}")

    def _run(self, targets):
        cmd = [self.pio, "run", "-d", self.project_dir, "-e", self.env]
        for target in targets:
            cmd += ["-t", target]
        if self.port and "upload" in targets:
            cmd += ["--upload-port", self.port]

        # shlex.join, not " ".join: the project path has a space in it, and the echoed command
        # is there to be copy-pasted when a build needs re-running by hand.
        print("[*] " + shlex.join(cmd))
        subprocess.run(cmd, check=True)

    def build(self):
        self._run(["build"])

    def upload(self, settle=2.0):
        """Builds and flashes the firmware, then waits for the board to re-enumerate."""
        self._run(["upload"])
        print(f"[*] Firmware flashed, waiting {settle:.1f}s for the board to come up")
        time.sleep(settle)


class TropicTarget:
    """Line protocol client for tvla_target.cpp.

    Replies are '+...' on success and '-ERR ...' on failure; '#...' lines are informational
    and only echoed when verbose is set.
    """

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, timeout=15.0, verbose=False):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.verbose = verbose
        self.ser = None
        # Rolling log of raw lines, so a failed handshake can report what the board actually sent.
        self._recent = collections.deque(maxlen=8)

    # ------------------------------------------------------------------ lifecycle
    def _reset_board(self):
        """Pulses EN so the boot banner lands after we have started listening.

        Opening the port already resets most ESP32 boards, but the banner - which carries the
        firmware's '-ERR' when Tropic01.begin() fails - is emitted while the driver is still
        settling and gets flushed. Losing it turns a one-line hardware fault into a silent
        30 second timeout, so the reset is driven explicitly instead. DTR low keeps IO0 high,
        which boots the application rather than the ROM bootloader.
        """
        try:
            self.ser.setDTR(False)
            self.ser.setRTS(True)   # EN low: hold in reset
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.ser.setRTS(False)  # release
            time.sleep(0.05)
        except (OSError, serial.SerialException):
            # Not every USB-serial bridge exposes the auto-reset lines; the open() itself has
            # usually reset the board anyway, so this is not worth failing over.
            pass

    def open(self, ready_timeout=30.0):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.5)
        self._reset_board()
        # Alternate between listening for '+READY' and pinging until one of them answers.
        deadline = time.time() + ready_timeout
        while time.time() < deadline:
            # A '-ERR' here means the firmware booted and reported a real failure (e.g. the secure
            # channel handshake) - it is never worth retrying, so let it propagate.
            try:
                if self._read_reply(timeout=2.0).startswith("READY"):
                    print("[*] Target READY")
                    return self
            except TimeoutError:
                pass
            try:
                self.ping()
                print("[*] Target alive (answered ping)")
                return self
            except TimeoutError:
                pass

        # Include whatever did come over the line - boot messages, a truncated banner or nothing
        # at all all point at different problems.
        seen = " | ".join(self._recent) if self._recent else "nothing received"
        raise TimeoutError(f"target on {self.port} did not come up within {ready_timeout}s. "
                           f"Last lines seen: {seen}")

    def close(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc, tb):
        self.close()

    # ------------------------------------------------------------------- protocol
    def _read_reply(self, timeout=None, accept_ready=True):
        """Reads lines until a '+'/'-' reply arrives. Returns the reply without its prefix.

        '+READY' is the firmware's unsolicited boot banner, not an answer to anything. If the
        board is still booting when a command goes out, that banner arrives first and would be
        returned as the command's reply - keygen then tries to parse "READY" as a public key.
        Commands therefore skip it; only open(), which is waiting for it, accepts it.
        """
        deadline = time.time() + (self.timeout if timeout is None else timeout)
        while time.time() < deadline:
            line = self.ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue
            self._recent.append(line)
            if line.startswith("#"):
                if self.verbose:
                    print("    " + line)
                continue
            if not accept_ready and line.startswith("+") and line[1:].strip().startswith("READY"):
                if self.verbose:
                    print("    (late boot banner ignored: " + line + ")")
                continue
            if line.startswith("-"):
                raise TargetError(line)
            if line.startswith("+"):
                return line[1:].strip()
            if self.verbose:
                print("    (unparsed: " + line + ")")
        raise TimeoutError(f"no reply from target on {self.port} within {timeout or self.timeout}s")

    def command(self, line, timeout=None):
        if self.ser is None:
            raise RuntimeError("target is not open")
        self.ser.reset_input_buffer()
        self.ser.write((line + "\n").encode("ascii"))
        self.ser.flush()
        return self._read_reply(timeout=timeout, accept_ready=False)

    # ------------------------------------------------------------------- commands
    def ping(self):
        return self.command("p", timeout=5.0)

    def version(self):
        return self.command("v", timeout=5.0)

    def keygen(self, curve):
        """Erases the curve's slot and generates a fresh key. curve is 'ed' or 'ec'."""
        if curve not in ("ed", "ec"):
            raise ValueError("curve must be 'ed' (Ed25519) or 'ec' (P256)")
        reply = self.command(f"g {curve}", timeout=30.0)
        return bytes.fromhex(reply.split()[-1])

    def store_key(self, curve, key):
        """Erases the curve's slot and writes the given 32 byte private key (secret scalar)."""
        if curve not in ("ed", "ec"):
            raise ValueError("curve must be 'ed' (Ed25519) or 'ec' (P256)")
        if len(key) != PRIVKEY_LEN:
            raise ValueError(f"private key must be {PRIVKEY_LEN} bytes, got {len(key)}")
        self.command(f"k {curve} " + key.hex(), timeout=30.0)

    def sign(self, payload, timeout=None):
        """Signs payload with the provisioned key; the target triggers the scope around the call."""
        reply = self.command("s " + payload.hex(), timeout=timeout)
        return bytes.fromhex(reply.split()[-1])

    def erase(self):
        self.command("e", timeout=30.0)
