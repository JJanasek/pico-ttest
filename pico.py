## pico.py
# This file wraps up the different picoscope driver python library for easier use of block mode.
# Only supports drivers for ps3000a, ps5000 and ps6000, because I can only test on picoscopes models 6407,5203,3206D.
# ps2000a (pico2000) reuses the ps3000a code path and has not been tested on hardware.
# ps6000 (pico6000: 6402-6404 C/D, 6407) reuses it too; written against the ps6000 Programmer's
# Guide (ps6000pg.en-11), not yet run on a 6404D.
# WARNING: Is not optimized and might contain bugs.
import numpy as np

# Each class imports its own driver, so a machine with only one series' library installed (say
# libps6000 for a 6404D) can still import this module.
from picosdk.errors import *
import ctypes
import re
import time
from picosdk.functions import adc2mV, assert_pico_ok

DEBUG_MODE = True

# How long a single armed block may take to trigger and transfer before we give up, in seconds.
CAPTURE_TIMEOUT = 10.0

# Trigger sources. Only the 3000 Series *D* models have the Ext input (the 3406D included); on
# A/B models ps3000aSetSimpleTrigger still returns PICO_OK for it, but ps3000aRunBlock then fails
# with PICO_TRIGGER_ERROR (see the A API Programmer's Guide, "External trigger input").
PS3000A_EXTERNAL = 4
# The Ext input is fixed at +-5 V, scaled to +-32767 regardless of the channel ranges.
PS3000A_EXT_MAX_VALUE = 32767
PS3000A_EXT_RANGE_V = 5.0

# The 6000 Series (ps6000) has no Ext input: its trigger-only input is AUX, a 50 ohm BNC with a
# fixed +-1 V range, scaled like the channels to +-PS6000_MAX_VALUE (Programmer's Guide, "Voltage
# ranges"). PS6000_EXTERNAL (4) exists in the enum but is not a usable trigger source.
PS6000_TRIGGER_AUX = 5
PS6000_MAX_VALUE = 32512
PS6000_AUX_RANGE_V = 1.0

CHANNEL_NAMES = {0: "A", 1: "B", 2: "C", 3: "D", PS3000A_EXTERNAL: "EXT", PS6000_TRIGGER_AUX: "AUX"}


def channelName(channel):
    return CHANNEL_NAMES.get(channel, str(channel))


def argClosest(lst, K):
    return min(range(len(lst)), key = lambda i: abs(lst[i]-K))

def rawToBytes(buffer, minADC, maxADC, n_points=None, chunk=4_000_000):
    """Rescales a raw int16 ADC buffer to signed bytes (the .trs BYTE sample coding).

    Vectorized on purpose: the per-sample list comprehension this replaces took seconds per
    trace once traces got long enough to hold a full public-key operation. Converted in chunks
    so a 50 M sample trace does not need a 200 MB float32 copy of itself.
    """
    raw = np.frombuffer(buffer, dtype=np.int16)
    if n_points is not None:
        raw = raw[:n_points]

    out = np.empty(raw.size, dtype=np.int8)
    scale = 255.0 / (maxADC - minADC)
    for start in range(0, raw.size, chunk):
        block = raw[start:start + chunk].astype(np.float32)
        block -= minADC
        block *= scale
        block -= 255.0 / 2
        np.rint(block, out=block)
        np.clip(block, -128, 127, out=block)
        out[start:start + chunk] = block.astype(np.int8)
    return out, raw

class pico3000():
    """PicoScope 3000 Series (ps3000a driver), block mode.

    The driver calls go through _call/_enum so pico2000 can reuse everything here: the ps2000a
    API has the same functions and arguments under a different prefix. pico6000 reuses it too,
    overriding the few calls whose arguments differ (_setChannel, _attachBuffer, _adcLimits).
    """

    _prefix = "ps3000a"
    _prefix_upper = "PS3000A"
    series = "3000"

    # GetValues' downSampleRatio. With RATIO_MODE_NONE ps3000a/ps2000a take 0 (as in Pico's
    # examples); ps6000 rejects 0 with PICO_INVALID_PARAMETER ("must be greater than zero").
    _downSampleRatio = 0

    # The trigger-only input (Ext on a D model) and the fixed scale its threshold is given in.
    EXT_CHANNEL = PS3000A_EXTERNAL
    EXT_MAX_VALUE = PS3000A_EXT_MAX_VALUE
    EXT_RANGE_V = PS3000A_EXT_RANGE_V

    def _loadDriver(self):
        from picosdk.ps3000a import ps3000a
        return ps3000a

    def _call(self, name, *args):
        return getattr(self._drv, self._prefix + name)(*args)

    def _enum(self, name):
        return getattr(self._drv, self._prefix_upper + "_" + name)

    def couplingValue(self, coupling):
        """Maps 'AC'/'DC' onto the driver's coupling enum."""
        enum = self._enum("COUPLING")
        key = self._prefix_upper + "_" + str(coupling).upper()
        if key not in enum:
            raise ValueError("unknown coupling {!r}, expected one of {}".format(
                coupling, sorted(k.split("_")[-1] for k in enum)))
        return enum[key]

    def __init__(self):
        print("[*] Picoscope {} SETUP".format(self.series))
        self._drv = self._loadDriver()
        self.status = {}
        self.chandle = ctypes.c_int16()
        self.chARange = None
        self.voltRange = ''
        self.timeDiv = 0
        self.timebase = 0
        self.voltDiv = 0
        self.sampleRate = 0
        self.maxADC = 0
        self.minADC = 0
        self.channel_out = None
        self.bufferAMin = None
        self.overflow = None
        self.cmaxSamples = None
        self.n_points = 0
        # Mutable buffer: the driver writes the variant string into it (a c_char_p would point at
        # an immutable bytes literal).
        self.model = ctypes.create_string_buffer(32)
        self.variant = "Unknown"
        self.channelRanges = {}  # channel -> full scale in volts, for trigger level conversion
        self.requiredSize = ctypes.c_int16()
        self.available_ranges = (ctypes.c_int32 * 15)()
        self.number_of_ranges = ctypes.c_int32(15)
        self.RANGES = []
        self.measureChannel = 0
        self.coupling = 'DC'
        self.offset = 0
        self.bandwidthLimit = "full"
        self._rangeCache = {}  # (channel, coupling) -> ranges, for drivers that probe them
        self.capturesTotal = 0
        self.capturesClipped = 0

    def disconnect(self):
        # Stops the scope
        # Handle = chandle
        self.status["stop"] = self._call("Stop", self.chandle)
        assert_pico_ok(self.status["stop"])

        # Closes the unit
        # Handle = chandle
        self.status["close"] = self._call("CloseUnit", self.chandle)
        assert_pico_ok(self.status["close"])

    def connect(self):
        self.status["openunit"] = self._call("OpenUnit", ctypes.byref(self.chandle), None)
        powerstate = self.status["openunit"]
        if powerstate == 3:
            # PICO_NOT_FOUND. By far the usual cause when going from the GUI to a script: the
            # PicoScope application still holds the device, and only one process can open it.
            raise PicoSDKCtypesError(
                "PICO_NOT_FOUND - no {} Series scope could be opened. Close the PicoScope "
                "application first (it keeps the device open even when idle), check the USB "
                "cable, and that the scope really uses the {} driver."
                .format(self.series, self._prefix))
        if powerstate in (282, 286) and hasattr(self._drv, self._prefix + "ChangePowerSource"):
            # 282 = PICO_POWER_SUPPLY_NOT_CONNECTED (4-channel models without their DC supply:
            # only A and B work), 286 = PICO_USB3_0_DEVICE_NON_USB3_0_PORT. Both open fine once
            # the driver is told to accept the power source. ps6000 has no such call.
            self.status["ChangePowerSource"] = self._call("ChangePowerSource", self.chandle, powerstate)
            assert_pico_ok(self.status["ChangePowerSource"])
            if powerstate == 282:
                print("[!] Running on USB power: channels C and D are unavailable until the DC "
                      "supply is connected")
            else:
                print("[!] USB 3.0 scope on a USB 2.0 port: reading out long blocks is slower")
        else:
            assert_pico_ok(powerstate)
        print("[*] Picoscope CONNECTED")

        # 3 = PICO_VARIANT_INFO. Worth printing: only D models have the Ext trigger input. This
        # used to be skipped when the power source had to be changed, leaving variant "Unknown".
        self.status["getinfo"] = self._call("GetUnitInfo", self.chandle, self.model, ctypes.sizeof(self.model), ctypes.byref(self.requiredSize), 3)
        assert_pico_ok(self.status["getinfo"])
        self.variant = self.model.value.decode(errors="replace")
        print("[*] Picoscope model: PicoScope {}".format(self.variant))
        if not self.hasExtTrigger():
            print("[!] Model {} has no Ext trigger input (D models only) - trigger on an "
                  "analog channel instead".format(self.variant))
        self._disableDigitalPorts()

    def _disableDigitalPorts(self):
        """MSO models: switch the digital ports off so they take no memory or timebases."""
        setPort = getattr(self._drv, self._prefix + "SetDigitalPort", None)
        if setPort is None:
            return
        for port in (0x80, 0x81):  # DIGITAL_PORT0 / PORT1
            setPort(self.chandle, port, 0, 0)  # fails harmlessly on non-MSO models

    def setChannel(self, channel, voltsPerDivision, sampleRate, timeDiv=None, n_points=None, offset=0, coupling='DC'):
        # timeDiv is only a hint: the achievable sample rate is quantized to the scope's timebase
        # grid, so the effective timeDiv is recomputed from n_points below and returned.
        if n_points is None:
            raise ValueError("n_points is required")
        self.sampleRate = sampleRate
        self.timeDiv = timeDiv
        self.n_points = n_points
        self.measureChannel = channel
        self.coupling = coupling
        self.offset = offset
        self.RANGES = self._availableRanges(channel, coupling)
        self.chARange = list(self.RANGES.keys())[argClosest([x[1] for x in self.RANGES.items()], 5*voltsPerDivision)]
        self.voltDiv = self.RANGES[self.chARange]/5 #update selected voltDiv
        self.voltRange = list(self._enum("RANGE").keys())[self.chARange]

        self.status["setChA"] = self._setChannel(channel, 1, self.couplingValue(coupling), self.chARange, offset)# Set up channel A
        assert_pico_ok(self.status["setChA"])
        self.channelRanges[channel] = self.RANGES[self.chARange]
        # Disable other channels
        for ch in range(1,4):
            self.status["setChB"] = self._setChannel((channel + ch)%4, 0, self.couplingValue('DC'), self.chARange, 0)
            try:
                assert_pico_ok(self.status["setChB"])
            except(PicoSDKCtypesError):
                # Not all scopes have channels C and D and here will complain.
                pass

        timeIntervalns = ctypes.c_float()
        # int32 in the API: a c_int16 here let the driver write past the end of the variable.
        returnedMaxSamples = ctypes.c_int32()
        self.timebase, self.sampleRate = self._selectTimebase(sampleRate)
        self.status["GetTimebase"] = self._call("GetTimebase2", self.chandle, self.timebase, self.n_points, ctypes.byref(timeIntervalns), 1, ctypes.byref(returnedMaxSamples), 0)#get timebase information
        assert_pico_ok(self.status["GetTimebase"])
        self.timeDiv = self.n_points/(10*self.sampleRate)

        # Create buffers ready for assigning pointers for data collection
        self.channel_out = (ctypes.c_int16 * self.n_points)()
        self._attachBuffer(channel)

        # Creates a overlow location for data
        self.overflow = (ctypes.c_int16)()
        # Creates converted types maxsamples
        self.cmaxSamples = ctypes.c_int32(self.n_points)

        # Finds the max ADC count
        self.maxADC, self.minADC = self._adcLimits()

        if(DEBUG_MODE):
            print("Measure channel")
            print("\tchannel: " + channelName(channel))
            print("\tScope state:")
            print("\tsampleRate: {:e}Hz".format(sampleRate))
            print("\tn_points: {}".format(self.n_points))
            print("\tvoltDiv: {}V/div".format(self.voltDiv))
            print("\tvoltRange: {}".format(self.voltRange))
            print(self.status)
        return self.voltDiv, self.timeDiv, self.sampleRate

    def _setChannel(self, channel, enabled, couplingValue, rangeIdx, offset):
        """One SetChannel call, returning its status. ps6000 adds a bandwidth argument."""
        return self._call("SetChannel", self.chandle, channel, enabled, couplingValue, rangeIdx, offset)

    def _attachBuffer(self, channel):
        """Hands channel_out to the driver as the channel's data buffer."""
        # The second (min) buffer only carries data for aggregate downsampling, which this wrapper
        # never enables - at 50M samples it is 100 MB of waste, so use the single-buffer call and
        # keep the two-buffer one as a fallback.
        self.bufferAMin = None
        try:
            self.status["SetDataBuffers"] = self._call("SetDataBuffer", self.chandle, channel, ctypes.byref(self.channel_out), self.n_points, 0, 0)
            assert_pico_ok(self.status["SetDataBuffers"])
        except (AttributeError, PicoSDKCtypesError):
            self.bufferAMin = (ctypes.c_int16 * self.n_points)()
            self.status["SetDataBuffers"] = self._call("SetDataBuffers", self.chandle, channel, ctypes.byref(self.channel_out), ctypes.byref(self.bufferAMin), self.n_points, 0, 0)# Setting the data buffer location for data collection from channel A
            assert_pico_ok(self.status["SetDataBuffers"])

    def _adcLimits(self):
        """(max, min) ADC counts that full scale maps onto, as c_int16."""
        maxADC = ctypes.c_int16()
        self.status["maximumValue"] = self._call("MaximumValue", self.chandle, ctypes.byref(maxADC))
        assert_pico_ok(self.status["maximumValue"])
        minADC = ctypes.c_int16()
        self.status["minimumValue"] = self._call("MinimumValue", self.chandle, ctypes.byref(minADC))
        assert_pico_ok(self.status["minimumValue"])
        return maxADC, minADC

    def _availableRanges(self, channel, coupling='DC'):
        """Input ranges the channel supports, as {range enum: full scale in volts}.

        coupling only matters to drivers that have to probe for the ranges (see _probeRanges).
        """
        self.status["availableRanges"] = self._call("GetChannelInformation", self.chandle, 0, 0, ctypes.byref(self.available_ranges), ctypes.byref(self.number_of_ranges), channel)
        assert_pico_ok(self.status["availableRanges"])
        return {x: self._drv.PICO_VOLTAGE_RANGE[x] for x in self.available_ranges[:self.number_of_ranges.value]}

    def _probeRanges(self, channel, coupling, volts):
        """The ranges in volts ({range enum: full scale}) that SetChannel accepts for this channel
        and coupling, for drivers whose GetChannelInformation is missing or unusable.

        Leaves the channel enabled on the last accepted range; callers set it up right after.
        """
        key = (channel, str(coupling).upper())
        if key not in self._rangeCache:
            ranges = {}
            for idx, v in sorted(volts.items()):
                st = self._setChannel(channel, 1, self.couplingValue(coupling), idx, 0)
                if st == 0:  # PICO_OK
                    ranges[idx] = v
            if not ranges:
                raise PicoSDKCtypesError("channel {} accepted no input range with {} coupling"
                                         .format(channelName(channel), coupling))
            self._rangeCache[key] = ranges
        return self._rangeCache[key]

    def _selectTimebase(self, sampleRate):
        """Returns (timebase, achieved sample rate) closest to the requested rate.

        Asks the driver rather than using a formula: the timebase rule differs between models
        (the 3000 D/A/B with a 1 GS/s clock, the older 3204-3206 A/B at 500 MS/s, every 2000
        model) and with the number of enabled channels. The formula this replaces assumed a
        1 GS/s model and stored its own idea of the rate - on any other model the .trs time
        axis came out wrong without a word. The interval grows with the timebase, so a binary
        search finds the closest one in ~32 calls.
        """
        n_points = self.n_points

        def interval(tb):
            ns = ctypes.c_float()
            maxSamples = ctypes.c_int32()
            st = self._call("GetTimebase2", self.chandle, tb, n_points, ctypes.byref(ns), 1, ctypes.byref(maxSamples), 0)
            return ns.value if st == 0 else None

        # The fastest timebases are invalid with more than one channel on, so find the first one
        # that works rather than assuming 0.
        lo = next((tb for tb in range(32) if interval(tb) is not None), None)
        if lo is None:
            maxSamples = ctypes.c_int32()
            ns = ctypes.c_float()
            fits = self._call("GetTimebase2", self.chandle, 100, 1, ctypes.byref(ns), 1, ctypes.byref(maxSamples), 0) == 0
            raise PicoSDKCtypesError(
                "no timebase accepts {:,} samples - {}; lower --samples".format(
                    n_points, "this model holds at most {:,} with the current channels"
                    .format(maxSamples.value) if fits else "more than this model's memory holds"))
        target = 1e9 / sampleRate
        if interval(lo) >= target:
            return lo, 1e9 / interval(lo)

        hi = 2**32 - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            iv = interval(mid)
            if iv is None or iv >= target:
                hi = mid
            else:
                lo = mid
        ivLo, ivHi = interval(lo), interval(hi)
        if ivHi is None or abs(ivLo - target) <= abs(ivHi - target):
            return lo, 1e9 / ivLo
        return hi, 1e9 / ivHi

    def _extNote(self):
        return "The Ext input exists only on 3000 Series D models"

    def _extLevelNote(self):
        """Appended to the error for a trigger level outside the Ext input's range."""
        return ""

    def hasExtTrigger(self):
        """True unless the variant string clearly names a non-D model (only D models have Ext)."""
        match = re.match(r"\s*(\d{4})\s*([A-Za-z])", self.variant)
        if not match:
            return True  # unknown variant - do not second-guess the user
        return match.group(2).upper() == "D"

    def enableChannel(self, channel, rangeVolts, offset=0.0, coupling='DC'):
        """Enables an extra channel, e.g. one used only as an analog trigger source.

        No data buffer is attached: an analog trigger only requires its channel to be enabled.
        Coupling stays DC by default: AC coupling makes a long logic-high pulse droop back towards
        zero, so a level trigger on it would only see the edge.
        """
        ranges = self._availableRanges(channel, coupling)
        # Smallest range that still fits the signal (never one that would clip it).
        fitting = sorted((v, k) for k, v in ranges.items() if v >= rangeVolts)
        rangeIdx = fitting[0][1] if fitting else max((v, k) for k, v in ranges.items())[1]
        self.status["setChTrig"] = self._setChannel(channel, 1, self.couplingValue(coupling), rangeIdx, offset)
        assert_pico_ok(self.status["setChTrig"])
        self.channelRanges[channel] = ranges[rangeIdx]

        # Enabling a second channel can restrict the available timebases (on a 1 GS/s model the
        # fastest one is single-channel only, and memory is split between channels). Re-select
        # rather than failing later inside RunBlock - what the GUI does silently when a second
        # channel is switched on.
        if self.n_points:
            timeIntervalns = ctypes.c_float()
            returnedMaxSamples = ctypes.c_int32()
            self.status["GetTimebaseTrig"] = self._call("GetTimebase2", self.chandle, self.timebase, self.n_points, ctypes.byref(timeIntervalns), 1, ctypes.byref(returnedMaxSamples), 0)
            if self.status["GetTimebaseTrig"] != 0:
                before = self.sampleRate
                self.timebase, self.sampleRate = self._selectTimebase(before)
                self.timeDiv = self.n_points/(10*self.sampleRate)
                print("[!] Enabling channel {} invalidated the timebase: sample rate {:.4g} MS/s "
                      "-> {:.4g} MS/s".format(channelName(channel), before/1e6, self.sampleRate/1e6))

        if(DEBUG_MODE):
            print("Trigger channel enabled")
            print("\tchannel: {}".format(channelName(channel)))
            print("\trange: +-{}V".format(ranges[rangeIdx]))
        return ranges[rangeIdx]

    def setTriggerChannel(self, channel, enable=0, level=None, threshold=None, timeout=0):
        """Arms a simple rising-edge trigger.

        timeout is the driver's autoTrigger_ms and defaults to 0 = wait indefinitely. A non-zero
        value makes the scope capture anyway when no trigger arrives, which would quietly put
        untriggered traces into the set; CAPTURE_TIMEOUT in getNativeSignalBytes bounds the wait
        instead.

        level is in volts and is converted to ADC counts for the source: the Ext input has a fixed
        full scale (+-5V on a 3000 D, +-1V for the 6000 Series AUX), an analog channel uses
        whatever range it was enabled with. threshold still accepts raw counts for callers that
        want them.
        """
        if threshold is None:
            level = 1.5 if level is None else level
            if channel == self.EXT_CHANNEL:
                if abs(level) > self.EXT_RANGE_V:
                    raise ValueError(
                        "trigger level {}V is outside the {} input's fixed +-{}V range{}".format(
                            level, channelName(channel), self.EXT_RANGE_V, self._extLevelNote()))
                threshold = int(round(level / self.EXT_RANGE_V * self.EXT_MAX_VALUE))
            else:
                fullScale = self.channelRanges.get(channel)
                if fullScale is None:
                    raise ValueError(
                        "channel {} must be enabled before it can trigger - call enableChannel() "
                        "or setChannel() for it first".format(channelName(channel)))
                threshold = int(round(level / fullScale * self.maxADC.value))

        # ps3000aSetSimpleTrigger takes the threshold as an int16 and would silently truncate.
        if not -32767 <= threshold <= 32767:
            raise ValueError(
                "trigger threshold {} counts is out of range for channel {} - the level ({}V) "
                "exceeds the channel's range".format(threshold, channelName(channel), level))

        if channel == self.EXT_CHANNEL and not self.hasExtTrigger():
            print("[!] Arming the Ext trigger on model {}, which has no Ext input - "
                  "{}RunBlock will most likely fail with PICO_TRIGGER_ERROR"
                  .format(self.variant, self._prefix))

        self.status["trigger"] = self._call("SetSimpleTrigger", self.chandle, enable, channel, threshold, self._enum("THRESHOLD_DIRECTION")[self._prefix_upper + "_RISING"], 0, timeout)# Sets up single trigger
        assert_pico_ok(self.status["trigger"])

        if(DEBUG_MODE):
            print("Trigger channel")
            print("\tchannel: {}".format(channelName(channel)))
            print("\tthreshold: {} counts{}".format(threshold, "" if level is None else " (~{:.2f}V)".format(level)))
            print("\ttimeout: " + str(timeout))


    def arm(self, preTrigger=0):
        # Stop any capture still running from the previous block before re-arming.
        self._call("Stop", self.chandle)
        # Starts block capture
        self.status["runblock"] = self._call("RunBlock", self.chandle, preTrigger, self.n_points - preTrigger, self.timebase, 1, None, 0, None, None)
        try:
            assert_pico_ok(self.status["runblock"])
        except PicoSDKCtypesError as ex:
            if "PICO_TRIGGER_ERROR" in str(ex):
                raise PicoSDKCtypesError(
                    "{} - the trigger source is not usable on this device. {} (this one reports "
                    "'{}'); trigger on an analog channel instead."
                    .format(ex, self._extNote(), self.variant)) from ex
            raise

        if(DEBUG_MODE):
            print("Block capture started.")

    def getNativeSignalBytes(self, timeout=CAPTURE_TIMEOUT):
            """Waits for the armed block to complete and returns (bytes, raw int16 samples).

            Raises TimeoutError if the trigger never fires, so a capture loop can re-arm and
            retry instead of blocking forever on a missed trigger.
            """
            ready = ctypes.c_int16(0)
            check = ctypes.c_int16(0)
            deadline = time.time() + timeout
            while ready.value == check.value:
                self.status["isReady"] = self._call("IsReady", self.chandle, ctypes.byref(ready))
                if time.time() > deadline:
                    self.status["stop"] = self._call("Stop", self.chandle)
                    raise TimeoutError("scope did not trigger within {:.1f}s".format(timeout))
                time.sleep(1e-4)  # yield instead of hammering the USB link

            # GetValues uses cmaxSamples as an in/out parameter, so it has to be reset to the
            # buffer size before every capture - otherwise a short block shrinks all later ones.
            self.cmaxSamples = ctypes.c_int32(self.n_points)
            self.status["GetValues"] = self._call("GetValues", self.chandle, 0, ctypes.byref(self.cmaxSamples), self._downSampleRatio, 0, 0, ctypes.byref(self.overflow))
            assert_pico_ok(self.status["GetValues"])
            # Every trace in a .trs has the same length, so a short block is an error rather
            # than something to write.
            if self.cmaxSamples.value != self.n_points:
                raise PicoSDKCtypesError("scope returned {:,} of {:,} samples".format(
                    self.cmaxSamples.value, self.n_points))

            # The driver flags a channel whose input went over range in this block - the GUI's
            # red over-range marker. Clipped samples are flat at full scale and carry no leakage,
            # so say so instead of letting them into the set unnoticed.
            self.capturesTotal += 1
            if self.overflow.value & (1 << self.measureChannel):
                self.capturesClipped += 1
                if self.capturesClipped == 1 or self.capturesClipped % 100 == 0:
                    print("[!] Channel {} over range in {} of {} captures - raise --volt-div, or "
                          "check the analog offset".format(channelName(self.measureChannel),
                                                         self.capturesClipped, self.capturesTotal))

            # Converts ADC from channel A to mV
            # channel_out_interpreted =  adc2mV(channel_out, chARange, maxADC)

            # Scale the output signal to interpret as bytes
            channel_out_interpreted, raw = rawToBytes(self.channel_out, self.minADC.value, self.maxADC.value, self.cmaxSamples.value)

            return channel_out_interpreted.tobytes(), raw

    def setBandwidthLimit(self, channel, enabled=True):
        """The GUI's per-channel bandwidth limiter (20 MHz on the 3000 Series)."""
        setFilter = getattr(self._drv, self._prefix + "SetBandwidthFilter", None)
        if setFilter is None:
            raise PicoSDKCtypesError("{} has no bandwidth limiter".format(self._prefix))
        self.status["bandwidth"] = setFilter(self.chandle, channel, 1 if enabled else 0)  # BW_20MHZ / BW_FULL
        assert_pico_ok(self.status["bandwidth"])
        self.bandwidthLimit = "20MHz" if enabled else "full"

    def describe(self, triggerChannel=None, triggerLevel=None, preTrigger=0, coupling=None):
        """The setup in PicoScope GUI terms, to compare with what was tuned there by hand."""
        window = self.n_points / self.sampleRate
        lines = [
            "PicoScope GUI equivalent:",
            "\tmodel:            PicoScope {} ({} driver)".format(self.variant, self._prefix),
            "\tcollection time:  {} /div  (10 div = {:.4g} ms)".format(_siTime(window / 10), window * 1e3),
            "\tsamples:          {:,}  at {:.4g} MS/s ({} per sample)".format(
                self.n_points, self.sampleRate / 1e6, _siTime(1 / self.sampleRate)),
            "\tchannel {}:        +-{} {}".format(channelName(self.measureChannel),
                                                 _siVolt(self.channelRanges[self.measureChannel]),
                                                 coupling or ""),
        ]
        if self.bandwidthLimit != "full":
            lines.append("\tbandwidth limit:  {}".format(self.bandwidthLimit))
        if triggerChannel is not None:
            lines.append("\ttrigger:          Single, rising edge on {}{}".format(
                channelName(triggerChannel),
                "" if triggerLevel is None else " at {}".format(_siVolt(triggerLevel))))
            lines.append("\tpre-trigger:      {:.0f} %".format(100.0 * preTrigger / self.n_points))
        print("\n".join(lines))


def _siTime(seconds):
    for unit, scale in (("s", 1), ("ms", 1e-3), ("us", 1e-6), ("ns", 1e-9)):
        if seconds >= scale:
            return "{:.4g} {}".format(seconds / scale, unit)
    return "{:.4g} ns".format(seconds * 1e9)


def _siVolt(volts):
    return "{:.4g} {}".format(volts * 1e3, "mV") if volts < 1 else "{:.4g} V".format(volts)

class pico2000(pico3000):
    """PicoScope 2000 Series via the ps2000a driver (2205A MSO, 2206B/2207B/2208B, 2405A-2408B...).

    The ps2000a API mirrors ps3000a, so only what genuinely differs is overridden. The oldest
    2000 models (2204A/2205A) use the separate ps2000 driver, which this does not cover.
    """

    _prefix = "ps2000a"
    _prefix_upper = "PS2000A"
    series = "2000"

    def _loadDriver(self):
        from picosdk.ps2000a import ps2000a
        return ps2000a

    def _availableRanges(self, channel, coupling='DC'):
        # picosdk declares ps2000aGetChannelInformation's pointer arguments as plain int32, so
        # it cannot be called through the wrapper; ask SetChannel which ranges it accepts instead.
        return self._probeRanges(channel, coupling, self._drv.PICO_VOLTAGE_RANGE)

    def _extNote(self):
        return "Not every 2000 Series model has an Ext trigger input"

    def hasExtTrigger(self):
        # The D-suffix rule is 3000 Series only; for the 2000 Series let RunBlock decide.
        return True

class pico5000():

    def __init__(self):
        print("[*] Picoscope SETUP")
        # Loaded here rather than at import, like the other series' drivers.
        global ps5
        from picosdk.ps5000 import ps5000 as ps5
        self.status = {}
        self.chandle = ctypes.c_int16()
        self.chARange = None
        self.timeDiv = 0
        self.timebase = 0
        self.voltDiv = 0
        self.sampleRate = 0
        self.maxADC = 0
        self.minADC = 0
        self.channel_out = None
        self.bufferAMin = None
        self.overflow = None
        self.cmaxSamples = None
        self.n_points = 0
    

    def disconnect(self):
        # Stops the scope
        # Handle = chandle
        self.status["stop"] = ps5.ps5000Stop(self.chandle)
        assert_pico_ok(self.status["stop"])

        # Closes the unit
        # Handle = chandle
        self.status["close"] = ps5.ps5000CloseUnit(self.chandle)
        assert_pico_ok(self.status["close"])

    def connect(self):
        self.status["openunit"] = ps5.ps5000OpenUnit(ctypes.byref(self.chandle))
        try:
            assert_pico_ok(self.status["openunit"])
            print("[*] Picoscope CONNECTED")
        except:
            # powerstate becomes the status number of openunit
            powerstate = self.status["openunit"]

            # If powerstate is the same as 282 then it will run this if statement
            if powerstate == 282:
                # Changes the power input to "PICO_POWER_SUPPLY_NOT_CONNECTED"
                self.status["ChangePowerSource"] = ps5.ps5000ChangePowerSource(self.chandle, 282)
                # If the powerstate is the same as 286 then it will run this if statement
            elif powerstate == 286:
                # Changes the power input to "PICO_USB3_0_DEVICE_NON_USB3_0_PORT"
                self.status["ChangePowerSource"] = ps5.ps5000ChangePowerSource(self.chandle, 286)
            else:
                raise

            assert_pico_ok(self.status["ChangePowerSource"])

    def setChannel(self, channel, voltsPerDivision, sampleRate, timeDiv, n_points):
        self.sampleRate = sampleRate
        self.timeDiv = timeDiv
        self.n_points = n_points
        volt_ranges = [10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000]
        closest_voltDiv = argClosest(volt_ranges, 5*voltsPerDivision*1e3)
        self.voltDiv = (volt_ranges[closest_voltDiv]/5)*1e-3 #update selected voltDiv
        pico_voltDiv = ['10MV', '20MV', '50MV', '100MV', '200MV', '500MV', '1V', '2V', '5V', '10V', '20V', '50V']
        self.voltRange = 'PS5000_'+pico_voltDiv[closest_voltDiv]
        self.chARange = ps5.PS5000_RANGE['PS5000_'+pico_voltDiv[closest_voltDiv]]

        self.status["setChA"] = ps5.ps5000SetChannel(self.chandle, channel, 1, True, self.chARange, 0)# Set up channel A, True=DC, False=AC
        assert_pico_ok(self.status["setChA"])
        # Disable other channels
        for ch in range(1,4):
            self.status["setChB"] = ps5.ps5000SetChannel(self.chandle, (channel + ch)%4, 0, True, self.chARange, 0)
        try:
            assert_pico_ok(self.status["setChB"])
        except(PicoSDKCtypesError):
            pass

        timeIntervalns = ctypes.c_float()
        returnedMaxSamples = ctypes.c_int16()
        # Sample rate for picoscope3000 follows a rule: sampleRate = 1e9/(2**n) for n<3, 125e6/(n-2) for n>=3.
        if self.sampleRate>=250e6:
            self.timebase = max(0, round(np.log2(1e9/self.sampleRate)))# n = log2(1e9/sampleRate) if 1GHz>=sampleRate>=250MHz
            self.sampleRate = 1e9/(2**self.timebase)
        elif sampleRate<250e6:
            self.timebase = min(2**32-1, round(125e6/self.sampleRate+2))# n = 125e6/sampleRate + 2 if sampleRate<125MHz
            self.sampleRate = 125e6/(self.timebase-2)
        else:
            raise 
        self.status["GetTimebase"] = ps5.ps5000GetTimebase(self.chandle, self.timebase, self.n_points, ctypes.byref(timeIntervalns), 1, ctypes.byref(returnedMaxSamples), 0)#get timebase information
        assert_pico_ok(self.status["GetTimebase"])
        self.timeDiv = self.n_points/(10*self.sampleRate)

        # Create buffers ready for assigning pointers for data collection
        self.channel_out = (ctypes.c_int16 * self.n_points)()
        self.bufferAMin = (ctypes.c_int16 * self.n_points)() # used for downsampling which isn't in the scope of this example
        self.status["SetDataBuffers"] = ps5.ps5000SetDataBuffers(self.chandle, channel, ctypes.byref(self.channel_out), ctypes.byref(self.bufferAMin), self.n_points, 0, 0)# Setting the data buffer location for data collection from channel A
        assert_pico_ok(self.status["SetDataBuffers"])

        # Creates a overlow location for data
        self.overflow = (ctypes.c_int16)()
        # Creates converted types maxsamples
        self.cmaxSamples = ctypes.c_int32(self.n_points)

        # Finds the max ADC count
        self.maxADC = ctypes.c_int16(32512)
        self.status["maximumValue"] = 0
        assert_pico_ok(self.status["maximumValue"])
        self.minADC = ctypes.c_int16(-32512)
        self.status["minimumValue"] = 0
        assert_pico_ok(self.status["minimumValue"])

        if(DEBUG_MODE):
            print("Measure channel")
            print("\tchannel: " + channelName(channel))
            print("\tScope state:")
            print("\tsampleRate: {:e}Hz".format(sampleRate))
            print("\tn_points: {}".format(self.n_points))
            print("\tvoltDiv: {}V/div".format(self.voltDiv))
            print("\tvoltRange: "'PS3000A_'+pico_voltDiv[closest_voltDiv])
            print(self.status)
        return self.voltDiv, self.timeDiv, self.sampleRate

    def setTriggerChannel(self, channel, enable=0, threshold=1024, timeout=1000):
        self.status["trigger"] = ps5.ps5000SetSimpleTrigger(self.chandle, enable, channel, threshold, 2, 0, timeout)# Sets up single trigger
        assert_pico_ok(self.status["trigger"])

        if(DEBUG_MODE):
            print("Trigger channel")
            print("\tchannel: " + channelName(channel))
            print("\ttimeout: " + str(timeout))


    def arm(self):
        # Starts block capture
        self.status["runblock"] = ps5.ps5000RunBlock(self.chandle, 0, self.n_points, self.timebase, 1, None, 0, None, None)
        assert_pico_ok(self.status["runblock"])
        
        if(DEBUG_MODE):
            print("Block capture started.")

    def getNativeSignalBytes(self):
            # Checks data collection to finish the capture
            ready = ctypes.c_int16(0)
            check = ctypes.c_int16(0)
            while ready.value == check.value:
                self.status["isReady"] = ps5.ps5000IsReady(self.chandle, ctypes.byref(ready))

            self.cmaxSamples = ctypes.c_int32(self.n_points)
            self.status["GetValues"] = ps5.ps5000GetValues(self.chandle, 0, ctypes.byref(self.cmaxSamples), 0, 0, 0, ctypes.byref(self.overflow))
            assert_pico_ok(self.status["GetValues"])

            # Converts ADC from channel A to mV
            # channel_out_interpreted =  adc2mV(channel_out, chARange, maxADC)

            # Scale the output signal to interpret as bytes
            channel_out_interpreted, raw = rawToBytes(self.channel_out, self.minADC.value, self.maxADC.value, self.cmaxSamples.value)

            return channel_out_interpreted.tobytes(), raw

class pico6000(pico3000):
    """PicoScope 6000 Series via the ps6000 driver: 6402C/D, 6403C/D, 6404C/D and 6407.

    Not the 6000E Series (6403E, 6424E...), which uses the separate ps6000a driver. ps6000 runs
    block mode with the same calls as ps3000a, so only what differs is overridden: SetChannel
    takes a bandwidth limiter, SetDataBuffer has no segment index, there is no MaximumValue,
    GetChannelInformation or ChangePowerSource, and the trigger-only input is AUX.
    """

    _prefix = "ps6000"
    _prefix_upper = "PS6000"
    series = "6000"
    _downSampleRatio = 1

    EXT_CHANNEL = PS6000_TRIGGER_AUX
    EXT_MAX_VALUE = PS6000_MAX_VALUE
    EXT_RANGE_V = PS6000_AUX_RANGE_V

    # Full scale of each PS6000_RANGE value, in ps6000Api.h order; picosdk leaves
    # ps6000.PICO_VOLTAGE_RANGE empty. Which of them a channel takes is probed (_availableRanges).
    RANGE_VOLTS = (0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0)

    # This wrapper's coupling names onto the ps6000 impedance/coupling types. AC exists only at
    # 1 Mohm, 50 ohm only DC (and only up to +-5V); the 6407 is 50 ohm only.
    COUPLINGS = {"AC": "PS6000_AC", "DC": "PS6000_DC_1M", "DC50": "PS6000_DC_50R"}

    def __init__(self):
        super().__init__()
        self._bandwidth = {}  # channel -> PS6000_BANDWIDTH_LIMITER value; BW_FULL when absent
        self._lastSetup = {}  # channel -> (coupling, range, offset) it was last enabled with

    def _loadDriver(self):
        from picosdk.ps6000 import ps6000
        return ps6000

    def couplingValue(self, coupling):
        key = self.COUPLINGS.get(str(coupling).upper())
        if key is None:
            raise ValueError("unknown coupling {!r}, expected one of {}".format(
                coupling, sorted(self.COUPLINGS)))
        return self._enum("COUPLING")[key]

    def _setChannel(self, channel, enabled, couplingValue, rangeIdx, offset):
        bandwidth = self._bandwidth.get(channel, self._enum("BANDWIDTH_LIMITER")["PS6000_BW_FULL"])
        st = self._call("SetChannel", self.chandle, channel, enabled, couplingValue, rangeIdx, offset, bandwidth)
        if st == 0 and enabled:
            self._lastSetup[channel] = (couplingValue, rangeIdx, offset)
        return st

    def _attachBuffer(self, channel):
        # ps6000SetDataBuffer has no segment index: (handle, channel, buffer, length, ratio mode).
        self.bufferAMin = None
        self.status["SetDataBuffers"] = self._call("SetDataBuffer", self.chandle, channel, ctypes.byref(self.channel_out), self.n_points, 0)  # PS6000_RATIO_MODE_NONE
        assert_pico_ok(self.status["SetDataBuffers"])

    def _adcLimits(self):
        # No MaximumValue/MinimumValue in ps6000: every model codes full scale as +-32512.
        return ctypes.c_int16(PS6000_MAX_VALUE), ctypes.c_int16(-PS6000_MAX_VALUE)

    def _availableRanges(self, channel, coupling='DC'):
        # No GetChannelInformation in ps6000, and the ranges depend on the model (the 6407 has
        # +-100mV only) and on the coupling (50 ohm stops at +-5V), so ask SetChannel.
        return self._probeRanges(channel, coupling, dict(enumerate(self.RANGE_VOLTS)))

    def hasExtTrigger(self):
        return True  # every ps6000 model has the AUX input

    def _extNote(self):
        return "On the 6000 Series the trigger input is AUX"

    def _extLevelNote(self):
        return ("; AUX is a 50 ohm input, so drive it from the 3.3V GPIO through a series "
                "resistor (470 ohm puts ~0.3V on it) and trigger at about half of what arrives")

    def setBandwidthLimit(self, channel, enabled=True):
        """The limiter is a SetChannel argument on ps6000: 20 MHz on the 6402/6403, 25 MHz on
        the 6404, none on the 6407. Re-applies the channel's last setup with it."""
        if channel not in self._lastSetup:
            raise ValueError("channel {} must be enabled before its bandwidth can be limited"
                             .format(channelName(channel)))
        bw = self._enum("BANDWIDTH_LIMITER")
        options = [("20MHz", bw["PS6000_BW_20MHZ"]), ("25MHz", bw["PS6000_BW_25MHZ"])]
        if self.variant.startswith("6404"):
            options.reverse()
        if not enabled:
            options = [("full", bw["PS6000_BW_FULL"])]
        for name, value in options:
            self._bandwidth[channel] = value
            st = self._setChannel(channel, 1, *self._lastSetup[channel])
            if st == 0:
                self.status["bandwidth"] = st
                self.bandwidthLimit = name
                return
        # Put the channel back to full bandwidth rather than leave it however the rejected calls
        # left it.
        self._bandwidth.pop(channel, None)
        self._setChannel(channel, 1, *self._lastSetup[channel])
        raise PicoSDKCtypesError("PicoScope {} has no bandwidth limiter (status {})"
                                 .format(self.variant, st))


# --pico-series values onto the block-mode classes above.
PICO_SERIES = {"3000": pico3000, "2000": pico2000, "6000": pico6000}


if __name__=="__main__":
    voltDiv = 4e-3 #V/div
    timeDiv = 100e-9 #s/div
    sampleRate = 1e9 #S/s
    offset = 20e-3 #V
    n_samples=int(10*timeDiv*sampleRate)

    scope = pico3000()
    scope.connect()
    voltDiv, timeDiv, sampleRate = scope.setChannel(0, voltDiv, sampleRate, timeDiv, n_samples,offset)
    print("Scope settings:\n\tvoltDiv: {:e}\n\tvoltRange: {}\n\ttimeDiv: {:e}\n\tsampleRate: {:e}".format(voltDiv,scope.voltRange,timeDiv,sampleRate))
    scope.setTriggerChannel(4,enable=1)#4 is Ext Channel

    scope.disconnect()
