## husky.py
# ChipWhisperer-Husky capture wrapper, shaped like the pico.py block-mode wrappers so
# tvla_capture.py can drive either scope with the same arm() / getNativeSignalBytes() loop.
#
# Husky differs from a PicoScope in three ways that matter here:
#
#  * One measurement input. There are no channels to pick and no volts/division: the MEASURE
#    SMA feeds a low-noise amplifier whose gain is set in dB, and the input is AC coupled in
#    hardware. Samples come back as a fraction of full scale, not as volts.
#  * The trigger is a logic pin, not an analog channel. It comes off the 20-pin connector
#    (tio1-4, nRST) or the AUX MCX, and a plain 3.3 V GPIO drives it directly - there is no
#    threshold to set and no probe attenuation to compensate for.
#  * The sample buffer is 131070 samples, full stop. A TROPIC01 signature takes ~123 ms, so
#    covering one needs either decimation (block mode, the default) or streaming.
import numpy as np

# Husky records either no pre-trigger samples at all or at least this many, and never more
# than 32767 - and never any while decimating.
MIN_PRESAMPLES = 8

# Husky's ADC sample buffer. Reading back 8 bits per sample instead of 12 does *not* enlarge it;
# it only raises the achievable streaming rate.
MAX_BLOCK_SAMPLES = 131070

# Clock generator limits (Husky; Husky Plus goes to 250 MHz).
MIN_CLKGEN_FREQ = 5e6
MAX_CLKGEN_FREQ = 200e6

# What the USB link sustains in stream mode, matching ChipWhisperer's own "keep ADC Freq
# < 10 MHz". A four-trace test at 15/20/25 MS/s passed cleanly and this was briefly raised to
# 25e6 on that basis - which was wrong. A real campaign at 20 MS/s hit FIFO errors within a few
# hundred traces, and overrunning the link does not just corrupt the trace: it wedges the Husky
# into LIBUSB_ERROR_IO / "Unknown ChipWhisperer" until it is physically replugged. 10 MS/s ran a
# 10,000 trace campaign with no FIFO errors at all. Do not raise this without a sustained test
# of at least several hundred traces.
MAX_STREAM_RATE = 10e6

# ADC clock used in block mode. Everything above the effective rate is thrown away by the
# decimator without an anti-alias filter, so a lower ADC clock folds less noise into the band
# that is kept - but it must stay a sane clock for the ADC. 20 MHz is a good compromise.
DEFAULT_ADC_FREQ = 20e6

# Measured on this rig (shunt in TROPIC01's 3.3 V rail, passive probe on MEASURE): a gain sweep
# put the 99.99th percentile at 120 of 127 with 0.006% of samples clipped - the most amplitude
# available before real signal starts hitting the rail. Re-derive it after any probe change; the
# capture warns in both directions.
DEFAULT_GAIN_DB = 39.0

# Streaming at 10 MS/s over the first 50 ms of the ~101 ms signature: decimate stays 1, so no
# aliasing, and 500k samples is 500 kB per trace.
DEFAULT_SAMPLE_RATE = 10e6
DEFAULT_SAMPLES = 500_000

# The trigger GPIO's own edge is the largest thing in the window and clips the ADC; skipping it
# is what lets the gain be set for the signature.
DEFAULT_SKIP_MS = 2.0
DEFAULT_TRIGGER_PIN = "tio4"
TRIGGER_PINS = ("tio1", "tio2", "tio3", "tio4", "nrst", "aux")

# A handful of clipped samples is normal and expected: the ESP32 polls TROPIC01's CHIP_STATUS
# over SPI every 25 ms (libtropic's LT_L1_READ_RETRY_DELAY_MS) and each poll puts a narrow spike
# on the shared supply. Warning on those would mean warning on every capture, so only a fraction
# large enough to be eating real signal is worth reporting.
CLIP_WARN_FRACTION = 1e-3

# How long a single armed capture may take to trigger and transfer before we give up, in seconds.
CAPTURE_TIMEOUT = 10.0

DEBUG_MODE = True


class FifoError(IOError):
    """Raised when a capture overran Husky's FIFOs and lost samples.

    Distinct from the TimeoutError a missed trigger raises, because the two need opposite
    responses: a timeout is retried as-is, while an overrun wedges the capture datapath and
    every later arm fails until the scope is rebuilt.
    """


class HuskyScope():
    """Block-mode (or streaming) capture on a ChipWhisperer-Husky.

    Interface deliberately mirrors pico.pico3000: connect / arm / getNativeSignalBytes /
    disconnect, with samples returned as signed bytes for the .trs BYTE sample coding.
    """

    def __init__(self, gain_db=DEFAULT_GAIN_DB, gain_mode="high",
                 trigger_pin=DEFAULT_TRIGGER_PIN, adc_freq=DEFAULT_ADC_FREQ,
                 stream=True, skip_ms=DEFAULT_SKIP_MS, timeout=CAPTURE_TIMEOUT):
        if trigger_pin not in TRIGGER_PINS:
            raise ValueError("trigger pin must be one of {}".format(", ".join(TRIGGER_PINS)))
        self.gain_db = gain_db
        self.gain_mode = gain_mode
        self.trigger_pin = trigger_pin
        self.adc_freq = adc_freq
        self.stream = stream
        self.skip_ms = skip_ms
        self.timeout = timeout

        self.scope = None
        self.sampleRate = None
        self.decimate = 1
        self.n_points = 0
        self.presamples = 0
        self.skip_samples = 0
        self.tile = 0
        # Named for symmetry with pico3000, which prints it after setChannel(). Husky's input
        # range is set by the LNA gain, so what a full-scale sample corresponds to in volts
        # depends on gain_db and on the probe - the traces are stored normalised instead.
        self.voltRange = "full scale (gain {:.1f} dB)".format(gain_db)
        self._clip_warned = False
        self._gain_warned = False

    # ------------------------------------------------------------------ lifecycle
    def connect(self):
        import chipwhisperer as cw

        self.scope = cw.scope()
        if not getattr(self.scope, "_is_husky", False):
            found = self.scope._getCWType()
            self.scope.dis()
            self.scope = None
            raise RuntimeError("connected ChipWhisperer is a '{}', not a Husky".format(found))

        # verbose=True walks scope.trace, which is None when TraceWhisperer failed to load, and
        # raises out of _dict_repr() before the setup is applied.
        self.scope.default_setup(verbose=False)

        # NOTE: do not disable the HS2 clock output here. default_setup() routes clkgen to HS2
        # (pin 6 of the 20-pin header), which does put a square wave at the ADC clock frequency
        # next to the target, and switching it off looked like a free way to stop TROPIC01
        # tripping into alarm mode. It is not free: with `scope.io.hs2 = None` a 200 MS/s capture
        # fails on the very first window with 'slow FIFO underflow, fast FIFO overflow', at the
        # same rate and offset that captures cleanly with HS2 left alone. Whatever the clock
        # module does when that output is gated, the ADC does not survive it.

        if DEBUG_MODE:
            print("Connected to {} (serial {})".format(
                self.scope._getCWType(), self.scope.sn))
        return self

    def disconnect(self):
        if self.scope is not None:
            self.scope.dis()
            self.scope = None

    # ----------------------------------------------------------------- acquisition
    def setChannel(self, sampleRate, n_points, preTrigger=0):
        """Configures gain, clock and capture depth.

        Returns (voltsPerDivision, timeDiv, sampleRate) so the caller can fill in the .trs
        header exactly as it does for a PicoScope. voltsPerDivision is 0.1: samples are stored
        as a fraction of ADC full scale, which is 10 divisions of 0.1.
        """
        scope = self.scope
        if scope is None:
            raise RuntimeError("scope is not connected")

        adc_freq, decimate = self._resolve_rate(sampleRate)

        if self.stream:
            if preTrigger:
                raise ValueError("Husky cannot record pre-trigger samples in stream mode")
        else:
            if n_points > MAX_BLOCK_SAMPLES:
                raise ValueError(
                    "{:,} samples exceeds Husky's {:,} sample buffer. Lower --samples or "
                    "--sample-rate, or add --stream (up to {:.0f} MS/s, no pre-trigger)."
                    .format(n_points, MAX_BLOCK_SAMPLES, MAX_STREAM_RATE / 1e6))
            if 0 < preTrigger < MIN_PRESAMPLES:
                raise ValueError(
                    "Husky needs at least {} pre-trigger samples (or none at all), got {}"
                    .format(MIN_PRESAMPLES, preTrigger))
            if preTrigger and decimate > 1:
                raise ValueError(
                    "Husky cannot combine pre-trigger samples with decimation (the requested "
                    "{:.3f} MS/s needs a factor of {}). Use --pre-trigger 0, or raise "
                    "--sample-rate to the ADC clock ({:.3f} MS/s)."
                    .format(sampleRate / 1e6, decimate, adc_freq / 1e6))

        # Free-running internal clock: the ESP32 runs off its own crystal, so nothing here is
        # synchronous with the target - the ADC clock only sets the sample rate.
        scope.clock.clkgen_src = "system"
        scope.clock.adc_mul = 1
        scope.clock.clkgen_freq = adc_freq
        if not scope.clock.clkgen_locked:
            raise RuntimeError("Husky PLL did not lock at {:.3f} MHz".format(adc_freq / 1e6))
        adc_freq = scope.clock.adc_freq

        scope.gain.mode = self.gain_mode
        scope.gain.db = self.gain_db

        # Husky rejects *any* write to presamples while decimate > 1 - including a write of 0 -
        # so decimation has to be off before presamples can be cleared, and back on afterwards.
        scope.adc.decimate = 1
        scope.adc.presamples = 0
        # Both of these change how many bytes the host expects per sample, and rewriting them
        # with the value they already hold is enough to desync the USB read - which surfaces as
        # LIBUSB_ERROR_OVERFLOW on the next capture and wedges the SAM3U until it is replugged.
        # 8 bits per sample is what the stream link can keep up with; in block mode there is no
        # reason not to read the ADC's full 12.
        bits = 8 if self.stream else 12
        if scope.adc.stream_mode != self.stream:
            scope.adc.stream_mode = self.stream
        if scope.adc.bits_per_sample != bits:
            scope.adc.bits_per_sample = bits
        scope.adc.samples = int(n_points)
        scope.adc.decimate = decimate
        if preTrigger:
            # Only reachable with decimate == 1; _resolve_rate() has already rejected the rest.
            scope.adc.presamples = int(preTrigger)
        # Discard the first skip_ms after the trigger. The trigger GPIO's own edge couples into
        # the measurement and is by far the largest thing in the window - large enough to clip
        # the ADC and force the gain down, starving the signal that is actually wanted. Skipping
        # past it is what lets the gain be set for the signature instead of for the edge.
        self.skip_samples = int(round(self.skip_ms * 1e-3 * adc_freq))
        scope.adc.offset = self.skip_samples
        scope.adc.basic_mode = "rising_edge"
        scope.adc.timeout = self.timeout

        self.decimate = decimate
        self.n_points = int(n_points)
        self.presamples = int(preTrigger)
        self.sampleRate = adc_freq / decimate
        timeDiv = (self.n_points / self.sampleRate) / 10.0

        if DEBUG_MODE:
            print("ADC configured:")
            print("\tADC clock: {:.3f} MS/s, decimate {} -> {:.3f} MS/s effective"
                  .format(adc_freq / 1e6, decimate, self.sampleRate / 1e6))
            print("\tsamples: {:,} ({:,} before the trigger){}"
                  .format(self.n_points, self.presamples, ", streaming" if self.stream else ""))
            if self.skip_ms:
                print("\tskipping: {:.2f} ms after the trigger ({:,} ADC clocks)"
                      .format(self.skip_ms, self.skip_samples))
            print("\tgain: {:.1f} dB ({} mode)".format(scope.gain.db, scope.gain.mode))

        return 0.1, timeDiv, self.sampleRate

    def _resolve_rate(self, sampleRate):
        """Picks an ADC clock and decimation factor for the requested effective sample rate."""
        if sampleRate <= 0:
            raise ValueError("sample rate must be positive")

        if self.stream:
            # Streaming reads every sample out over USB, so the ADC clock *is* the sample rate.
            if sampleRate > MAX_STREAM_RATE:
                raise ValueError(
                    "{:.3f} MS/s is beyond what Husky can stream (~{:.0f} MS/s). Lower "
                    "--sample-rate, or drop --stream and use decimated block mode."
                    .format(sampleRate / 1e6, MAX_STREAM_RATE / 1e6))
            if sampleRate < MIN_CLKGEN_FREQ:
                raise ValueError(
                    "Husky's clock generator stops at {:.0f} MHz, so streaming below that is "
                    "not possible. Drop --stream: block mode reaches low rates by decimating."
                    .format(MIN_CLKGEN_FREQ / 1e6))
            return sampleRate, 1

        # Block mode: keep the ADC on a fixed clock and decimate down to the requested rate.
        adc_freq = max(self.adc_freq, sampleRate)
        if adc_freq > MAX_CLKGEN_FREQ:
            raise ValueError("{:.3f} MS/s is above Husky's {:.0f} MS/s maximum"
                             .format(sampleRate / 1e6, MAX_CLKGEN_FREQ / 1e6))
        adc_freq = max(adc_freq, MIN_CLKGEN_FREQ)
        return adc_freq, max(1, int(round(adc_freq / sampleRate)))

    def setTriggerChannel(self, enable=1):
        """Routes the trigger to the pin the ESP32's TVLA GPIO is wired to."""
        scope = self.scope
        if scope is None:
            raise RuntimeError("scope is not connected")

        scope.trigger.module = "basic"
        scope.trigger.triggers = self.trigger_pin
        scope.adc.basic_mode = "rising_edge"

        # Husky must not drive the line it is supposed to be watching. default_setup() puts
        # tio1/tio2 in serial mode, so a trigger on those needs them released explicitly.
        if self.trigger_pin == "aux":
            scope.io.aux_io_mcx = "high_z"
        else:
            setattr(scope.io, self.trigger_pin, "high_z")

        if DEBUG_MODE and enable:
            print("Trigger: rising edge on {}".format(self.trigger_pin))

    def set_tile(self, index):
        """Moves the capture window to tile `index`, for covering a long operation in slices.

        Husky's buffer is 131,070 samples however fast the ADC runs, so a high-rate capture only
        ever sees a short window. Stepping adc.offset by one window per tile walks that window
        along the operation; tile 0 starts at skip_ms, tile 1 one window later, and so on, so the
        tiles butt up against each other and concatenate into a continuous span.

        Only meaningful at decimate == 1: offset counts ADC clocks, which equal samples only when
        nothing is being dropped.
        """
        if self.scope is None:
            raise RuntimeError("scope is not connected")
        if self.decimate != 1:
            raise ValueError(
                "tiling needs decimate == 1 (offset counts ADC clocks, not decimated samples); "
                "the requested rate decimates by {}".format(self.decimate))
        self.tile = index
        offset = self.skip_samples + index * self.n_points
        # Only touch the register when the window actually moves. Writing OFFSET_ADDR the value
        # it already holds breaks the next capture: at 200 MS/s every tiled run failed on tile 0
        # with 'slow FIFO underflow, fast FIFO overflow', while the identical single-window
        # capture - same rate, same 400,000 clock offset, but no redundant write - ran clean.
        # Tile 0 is already where setChannel() left it, so it is the only tile that makes a
        # no-op write. Skipping it fixed tiling outright: tiles 1-76 write a *changed* offset
        # immediately before arm() and are fine, so it is specifically the redundant write that
        # upsets the FPGA, not writing the register late.
        if offset != self.scope.adc.offset:
            self.scope.adc.offset = offset

    def arm(self, preTrigger=0):
        if self.scope is None:
            raise RuntimeError("scope is not connected")
        if int(preTrigger) != self.presamples:
            raise ValueError("pre-trigger changed after setChannel(): {} vs {}"
                             .format(preTrigger, self.presamples))
        # Sticky flags, so they have to be cleared per capture for the check below to mean
        # anything about *this* trace.
        self.scope.adc.errors = 0
        self.scope.arm()

    def getNativeSignalBytes(self, timeout=None):
        """Waits for the armed capture and returns (signed bytes, raw ADC samples).

        Raises TimeoutError when the trigger never fires, matching pico3000 so the capture
        loop re-arms and retries rather than storing a bogus trace.
        """
        scope = self.scope
        if scope is None:
            raise RuntimeError("scope is not connected")
        if timeout is not None:
            scope.adc.timeout = timeout

        try:
            # poll_done=True asks Husky when the capture actually finished. The default instead
            # sleeps for a computed (offset+samples)/adc_freq, and that estimate gets fragile as
            # the offset grows - reading early shows up as 'slow FIFO underflow' with a
            # 'fast FIFO overflow' behind it, which is what tiling was hitting.
            timed_out = scope.capture(poll_done=True)
        except Exception as ex:
            # A desynced bulk read leaves the Husky unable to identify itself on the next
            # connect, and only a replug clears it - say so rather than letting the raw
            # libusb error travel up through the capture loop's generic handler.
            if "OVERFLOW" in str(ex).upper():
                raise IOError(
                    "USB read overflowed - the Husky needs to be unplugged and plugged back in "
                    "before it will enumerate correctly again ({})".format(ex)) from ex
            raise
        if timed_out:
            raise TimeoutError(
                "scope did not trigger within {:.1f}s (or returned short data)"
                .format(scope.adc.timeout))

        # get_last_trace() is scaled to [-0.5, 0.5]; the .trs BYTE coding is a signed byte, so
        # this lands on the same [-128, 127] mapping pico.rawToBytes() produces.
        trace = scope.get_last_trace()
        raw = scope.get_last_trace(as_int=True)
        out = np.clip(np.rint(np.asarray(trace, dtype=np.float32) * 255.0), -128, 127)
        # A FIFO over/underflow means the stream could not keep up and samples were dropped or
        # duplicated. The data still has the right length and looks plausible - corrupted samples
        # mostly show up as extreme values, which reads as "clipping" - so nothing downstream
        # would notice. Reject the trace instead of writing it to the .trs.
        errors = str(scope.adc.errors or "")
        if "overflow" in errors or "underflow" in errors:
            raise FifoError(
                "Husky FIFO error at {:.1f} MS/s, tile {}, offset {:,} ADC clocks ({:.2f} ms) "
                "({}) - samples were lost, so this trace is not trustworthy. Note that block "
                "mode at 200 MS/s with only the 2 ms skip has run a clean 100 trace campaign, "
                "so a high rate on its own is not enough to cause this; a large offset is the "
                "other thing that provokes it. Bisect by sweeping --skip-ms at a fixed rate."
                .format(self.sampleRate / 1e6, self.tile, self.scope.adc.offset,
                        1e3 * self.scope.adc.offset / self.sampleRate,
                        errors.strip().rstrip(",")))

        self._report_errors(float(np.mean((out >= 127) | (out <= -128))))
        return out.astype(np.int8).tobytes(), raw

    def _report_errors(self, clipped_fraction):
        """Reports gain problems once each.

        Husky's own 'ADC clipped' flag is set by a single railed sample, which the SPI-poll
        spikes guarantee on every capture, so the measured fraction is used instead.
        """
        if clipped_fraction > CLIP_WARN_FRACTION and not self._clip_warned:
            self._clip_warned = True
            print("[!] {:.3f}% of samples clipped at --gain-db {:.1f} - enough to be flattening "
                  "real signal, lower it".format(100 * clipped_fraction, self.gain_db))
        errors = str(self.scope.adc.errors or "")
        if "gain too low" in errors and not self._gain_warned:
            self._gain_warned = True
            print("[!] Signal uses under a quarter of the ADC range - raise --gain-db "
                  "(currently {:.1f})".format(self.gain_db))
