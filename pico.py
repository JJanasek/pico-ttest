## pico.py
# This file wraps up the different picoscope driver python library for easier use of block mode.
# Only supports drivers for ps3000a, ps5000 and ps6000, because I can only test on picoscopes models 6407,5203,3206D.
# WARNING: Is not optimized and might contain bugs.
import numpy as np

from picosdk.errors import *
from picosdk.ps3000a import ps3000a as ps3
from picosdk.ps5000 import ps5000 as ps5
from picosdk.ps6000 import ps6000 as ps6
import ctypes
import re
import time
from picosdk.functions import adc2mV, assert_pico_ok

DEBUG_MODE = True

# How long a single armed block may take to trigger and transfer before we give up, in seconds.
CAPTURE_TIMEOUT = 10.0

# Trigger sources. Only the 3000 Series *D* models have the Ext input; on A/B models
# ps3000aSetSimpleTrigger still returns PICO_OK for it, but ps3000aRunBlock then fails with
# PICO_TRIGGER_ERROR (see the A API Programmer's Guide, "External trigger input").
PS3000A_EXTERNAL = 4
# The Ext input is fixed at +-5 V, scaled to +-32767 regardless of the channel ranges.
PS3000A_EXT_MAX_VALUE = 32767
PS3000A_EXT_RANGE_V = 5.0

CHANNEL_NAMES = {0: "A", 1: "B", 2: "C", 3: "D", PS3000A_EXTERNAL: "EXT"}


def channelName(channel):
    return CHANNEL_NAMES.get(channel, str(channel))

def argClosest(lst, K):
    return min(range(len(lst)), key = lambda i: abs(lst[i]-K))

def rawToBytes(buffer, minADC, maxADC, n_points=None):
    """Rescales a raw int16 ADC buffer to signed bytes (the .trs BYTE sample coding).

    Vectorized on purpose: the per-sample list comprehension this replaces took seconds per
    trace once traces got long enough to hold a full public-key operation.
    """
    raw = np.frombuffer(buffer, dtype=np.int16)
    if n_points is not None:
        raw = raw[:n_points]
    scaled = 255.0 * (raw.astype(np.float32) - minADC) / (maxADC - minADC) - 255.0 / 2
    return np.clip(np.rint(scaled), -128, 127).astype(np.int8), raw

class pico3000():

    def __init__(self):
        print("[*] Picoscope SETUP")
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

    def disconnect(self):
        # Stops the scope
        # Handle = chandle
        self.status["stop"] = ps3.ps3000aStop(self.chandle)
        assert_pico_ok(self.status["stop"])

        # Closes the unit
        # Handle = chandle
        self.status["close"] = ps3.ps3000aCloseUnit(self.chandle)
        assert_pico_ok(self.status["close"])

    def connect(self):
        self.status["openunit"] = ps3.ps3000aOpenUnit(ctypes.byref(self.chandle), None)
        try:
            assert_pico_ok(self.status["openunit"])
            print("[*] Picoscope CONNECTED")
            # 3 = PICO_VARIANT_INFO. Worth printing: only D models have the Ext trigger input.
            self.status["getinfo"] = ps3.ps3000aGetUnitInfo(self.chandle, self.model, ctypes.sizeof(self.model), ctypes.byref(self.requiredSize), 3)
            assert_pico_ok(self.status["getinfo"])
            self.variant = self.model.value.decode(errors="replace")
            print("[*] Picoscope model: PicoScope {}".format(self.variant))
            if not self.hasExtTrigger():
                print("[!] Model {} has no Ext trigger input (D models only) - trigger on an "
                      "analog channel instead".format(self.variant))
        except:
            # powerstate becomes the status number of openunit
            powerstate = self.status["openunit"]

            # If powerstate is the same as 282 then it will run this if statement
            if powerstate == 282:
                # Changes the power input to "PICO_POWER_SUPPLY_NOT_CONNECTED"
                self.status["ChangePowerSource"] = ps3.ps3000aChangePowerSource(self.chandle, 282)
                # If the powerstate is the same as 286 then it will run this if statement
            elif powerstate == 286:
                # Changes the power input to "PICO_USB3_0_DEVICE_NON_USB3_0_PORT"
                self.status["ChangePowerSource"] = ps3.ps3000aChangePowerSource(self.chandle, 286)
            else:
                raise

            assert_pico_ok(self.status["ChangePowerSource"])

    def setChannel(self, channel, voltsPerDivision, sampleRate, timeDiv=None, n_points=None, offset=0):
        # timeDiv is only a hint: the achievable sample rate is quantized to the scope's timebase
        # grid, so the effective timeDiv is recomputed from n_points below and returned.
        if n_points is None:
            raise ValueError("n_points is required")
        self.sampleRate = sampleRate
        self.timeDiv = timeDiv
        self.n_points = n_points
        self.status["availableRanges"] = ps3.ps3000aGetChannelInformation(self.chandle, 0, 0, ctypes.byref(self.available_ranges), ctypes.byref(self.number_of_ranges), channel)
        assert_pico_ok(self.status["availableRanges"])
        self.RANGES = {x:ps3.PICO_VOLTAGE_RANGE[x] for x in self.available_ranges[:self.number_of_ranges.value]}
        self.chARange = list(self.RANGES.keys())[argClosest([x[1] for x in self.RANGES.items()], 5*voltsPerDivision)]
        self.voltDiv = self.RANGES[self.chARange]/5 #update selected voltDiv
        self.voltRange = list(ps3.PS3000A_RANGE.keys())[self.chARange]

        self.status["setChA"] = ps3.ps3000aSetChannel(self.chandle, channel, 1, ps3.PS3000A_COUPLING['PS3000A_DC'], self.chARange, offset)# Set up channel A
        assert_pico_ok(self.status["setChA"])
        self.channelRanges[channel] = self.RANGES[self.chARange]
        # Disable other channels
        for ch in range(1,4):
            self.status["setChB"] = ps3.ps3000aSetChannel(self.chandle, (channel + ch)%4, 0, ps3.PS3000A_COUPLING['PS3000A_DC'], self.chARange, 0)
            try:
                assert_pico_ok(self.status["setChB"])
            except(PicoSDKCtypesError):
                # Not all scopes have channels C and D and here will complain.
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
        self.status["GetTimebase"] = ps3.ps3000aGetTimebase2(self.chandle, self.timebase, self.n_points, ctypes.byref(timeIntervalns), 1, ctypes.byref(returnedMaxSamples), 0)#get timebase information
        assert_pico_ok(self.status["GetTimebase"])
        self.timeDiv = self.n_points/(10*self.sampleRate)

        # Create buffers ready for assigning pointers for data collection
        self.channel_out = (ctypes.c_int16 * self.n_points)()
        self.bufferAMin = (ctypes.c_int16 * self.n_points)() # used for downsampling which isn't in the scope of this example
        self.status["SetDataBuffers"] = ps3.ps3000aSetDataBuffers(self.chandle, channel, ctypes.byref(self.channel_out), ctypes.byref(self.bufferAMin), self.n_points, 0, 0)# Setting the data buffer location for data collection from channel A
        assert_pico_ok(self.status["SetDataBuffers"])

        # Creates a overlow location for data
        self.overflow = (ctypes.c_int16)()
        # Creates converted types maxsamples
        self.cmaxSamples = ctypes.c_int32(self.n_points)

        # Finds the max ADC count
        self.maxADC = ctypes.c_int16()
        self.status["maximumValue"] = ps3.ps3000aMaximumValue(self.chandle, ctypes.byref(self.maxADC))
        assert_pico_ok(self.status["maximumValue"])
        self.minADC = ctypes.c_int16()
        self.status["minimumValue"] = ps3.ps3000aMinimumValue(self.chandle, ctypes.byref(self.minADC))
        assert_pico_ok(self.status["minimumValue"])

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

    def hasExtTrigger(self):
        """True unless the variant string clearly names a non-D model (only D models have Ext)."""
        match = re.match(r"\s*(\d{4})\s*([A-Za-z])", self.variant)
        if not match:
            return True  # unknown variant - do not second-guess the user
        return match.group(2).upper() == "D"

    def enableChannel(self, channel, rangeVolts, offset=0.0):
        """Enables an extra channel, e.g. one used only as an analog trigger source.

        No data buffer is attached: an analog trigger only requires its channel to be enabled.
        """
        ranges = {x: ps3.PICO_VOLTAGE_RANGE[x] for x in self.available_ranges[:self.number_of_ranges.value]}
        # Smallest range that still fits the signal (never one that would clip it).
        fitting = sorted((v, k) for k, v in ranges.items() if v >= rangeVolts)
        rangeIdx = fitting[0][1] if fitting else max((v, k) for k, v in ranges.items())[1]
        self.status["setChTrig"] = ps3.ps3000aSetChannel(self.chandle, channel, 1, ps3.PS3000A_COUPLING['PS3000A_DC'], rangeIdx, offset)
        assert_pico_ok(self.status["setChTrig"])
        self.channelRanges[channel] = ranges[rangeIdx]

        # Enabling a second channel can restrict the available timebases, so re-validate the one
        # chosen in setChannel rather than failing later inside RunBlock.
        if self.n_points:
            timeIntervalns = ctypes.c_float()
            returnedMaxSamples = ctypes.c_int16()
            self.status["GetTimebaseTrig"] = ps3.ps3000aGetTimebase2(self.chandle, self.timebase, self.n_points, ctypes.byref(timeIntervalns), 1, ctypes.byref(returnedMaxSamples), 0)
            assert_pico_ok(self.status["GetTimebaseTrig"])

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

        level is in volts and is converted to ADC counts for the source: the Ext input is always
        +-5V full scale, an analog channel uses whatever range it was enabled with. threshold
        still accepts raw counts for callers that want them.
        """
        if threshold is None:
            level = 1.5 if level is None else level
            if channel == PS3000A_EXTERNAL:
                threshold = int(round(level / PS3000A_EXT_RANGE_V * PS3000A_EXT_MAX_VALUE))
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

        if channel == PS3000A_EXTERNAL and not self.hasExtTrigger():
            print("[!] Arming the Ext trigger on model {}, which has no Ext input - "
                  "ps3000aRunBlock will most likely fail with PICO_TRIGGER_ERROR"
                  .format(self.variant))

        self.status["trigger"] = ps3.ps3000aSetSimpleTrigger(self.chandle, enable, channel, threshold, ps3.PS3000A_THRESHOLD_DIRECTION['PS3000A_RISING'], 0, timeout)# Sets up single trigger
        assert_pico_ok(self.status["trigger"])

        if(DEBUG_MODE):
            print("Trigger channel")
            print("\tchannel: {}".format(channelName(channel)))
            print("\tthreshold: {} counts{}".format(threshold, "" if level is None else " (~{:.2f}V)".format(level)))
            print("\ttimeout: " + str(timeout))


    def arm(self, preTrigger=0):
        # Stop any capture still running from the previous block before re-arming.
        ps3.ps3000aStop(self.chandle)
        # Starts block capture
        self.status["runblock"] = ps3.ps3000aRunBlock(self.chandle, preTrigger, self.n_points - preTrigger, self.timebase, 1, None, 0, None, None)
        try:
            assert_pico_ok(self.status["runblock"])
        except PicoSDKCtypesError as ex:
            if "PICO_TRIGGER_ERROR" in str(ex):
                raise PicoSDKCtypesError(
                    "{} - the trigger source is not usable on this device. The Ext input exists "
                    "only on 3000 Series D models (this one reports '{}'); trigger on an analog "
                    "channel instead.".format(ex, self.variant)) from ex
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
                self.status["isReady"] = ps3.ps3000aIsReady(self.chandle, ctypes.byref(ready))
                if time.time() > deadline:
                    self.status["stop"] = ps3.ps3000aStop(self.chandle)
                    raise TimeoutError("scope did not trigger within {:.1f}s".format(timeout))
                time.sleep(1e-4)  # yield instead of hammering the USB link

            # GetValues uses cmaxSamples as an in/out parameter, so it has to be reset to the
            # buffer size before every capture - otherwise a short block shrinks all later ones.
            self.cmaxSamples = ctypes.c_int32(self.n_points)
            self.status["GetValues"] = ps3.ps3000aGetValues(self.chandle, 0, ctypes.byref(self.cmaxSamples), 0, 0, 0, ctypes.byref(self.overflow))
            assert_pico_ok(self.status["GetValues"])

            # Converts ADC from channel A to mV
            # channel_out_interpreted =  adc2mV(channel_out, chARange, maxADC)

            # Scale the output signal to interpret as bytes
            channel_out_interpreted, raw = rawToBytes(self.channel_out, self.minADC.value, self.maxADC.value, self.cmaxSamples.value)

            return channel_out_interpreted.tobytes(), raw

class pico5000():

    def __init__(self):
        print("[*] Picoscope SETUP")
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

class pico6000():

    def __init__(self):
        print("[*] Picoscope SETUP")
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
        self.status["stop"] = ps6.ps6000Stop(self.chandle)
        assert_pico_ok(self.status["stop"])

        # Closes the unit
        # Handle = chandle
        self.status["close"] = ps6.ps6000CloseUnit(self.chandle)
        assert_pico_ok(self.status["close"])

    def connect(self):
        self.status["openunit"] = ps6.ps6000OpenUnit(ctypes.byref(self.chandle), None, 0)#resolution, 0=8bits, 
        try:
            assert_pico_ok(self.status["openunit"])
            print("[*] Picoscope CONNECTED")
        except:
            # powerstate becomes the status number of openunit
            powerstate = self.status["openunit"]

            # If powerstate is the same as 282 then it will run this if statement
            if powerstate == 282:
                # Changes the power input to "PICO_POWER_SUPPLY_NOT_CONNECTED"
                self.status["ChangePowerSource"] = ps6.ps6000ChangePowerSource(self.chandle, 282)
                # If the powerstate is the same as 286 then it will run this if statement
            elif powerstate == 286:
                # Changes the power input to "PICO_USB3_0_DEVICE_NON_USB3_0_PORT"
                self.status["ChangePowerSource"] = ps6.ps6000ChangePowerSource(self.chandle, 286)
            else:
                raise

            assert_pico_ok(self.status["ChangePowerSource"])

    def setChannel(self, channel, voltsPerDivision, sampleRate, timeDiv, n_points):
        self.sampleRate = sampleRate
        self.timeDiv = timeDiv
        self.n_points = n_points
        # volt_ranges = [10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000]
        # closest_voltDiv = argClosest(volt_ranges, 5*voltsPerDivision*1e3)
        # self.voltDiv = (volt_ranges[closest_voltDiv]/5)*1e-3 #update selected voltDiv
        # pico_voltDiv = ['A_10MV', '_20MV', '_50MV', '_100MV', '_200MV', '_500MV', '_1V', '_2V', '_5V', '_10V', '_20V', '_50V']

        #Picoscope 6407 only have +-100mv range
        self.voltDiv = (1000)*1e-3 #update selected voltDiv
        self.chARange = ps6.PS6000_RANGE['PS6000_100MV']
        self.voltRange = 'PS6000_100MV'
        
        self.status["setChA"] = ps6.ps6000SetChannel(self.chandle, channel, 1, ps6.PS6000_COUPLING['PS6000_DC_50R'], self.chARange, 0, ps6.PS6000_BANDWIDTH_LIMITER["PS6000_BW_FULL"])# Set up channel A
        assert_pico_ok(self.status["setChA"])
        # Disable other channels
        for ch in range(1,4):
            self.status["setChB"] = ps6.ps6000SetChannel(self.chandle, (channel + ch)%4, 0, ps6.PS6000_COUPLING['PS6000_DC_50R'], self.chARange, 0, ps6.PS6000_BANDWIDTH_LIMITER["PS6000_BW_FULL"])
        try:
            assert_pico_ok(self.status["setChB"])
        except(PicoSDKCtypesError):
            pass

        timeIntervalns = ctypes.c_float()
        returnedMaxSamples = ctypes.c_int16()
        # Sample rate for picoscope6000 follows a rule: sampleRate = 5e9/(2**n) for n<5, 156.250e6/(n-4) for n>=5.
        if self.sampleRate>=156.25e6:
            self.timebase = max(0, round(np.log2(5e9/self.sampleRate)))# n = log2(5e9/sampleRate) if 5GHz>=sampleRate>=156.25MHz
            self.sampleRate = 5e9/(2**self.timebase)
        elif sampleRate<156.25e6:
            self.timebase = min(2**32-1, round(156.25e6/self.sampleRate+4))# n = 156.25e6/sampleRate + 4 if sampleRate<125MHz
            self.sampleRate = 156.25e6/(self.timebase-4)
        else:
            raise 
        self.status["GetTimebase"] = ps6.ps6000GetTimebase2(self.chandle, self.timebase, self.n_points, ctypes.byref(timeIntervalns), 1, ctypes.byref(returnedMaxSamples), 0)#get timebase information
        assert_pico_ok(self.status["GetTimebase"])
        self.timeDiv = self.n_points/(10*self.sampleRate)

        # Create buffers ready for assigning pointers for data collection
        self.channel_out = (ctypes.c_int16 * self.n_points)()
        self.bufferAMin = (ctypes.c_int16 * self.n_points)() # used for downsampling which isn't in the scope of this example
        self.status["SetDataBuffers"] = ps6.ps6000SetDataBuffers(self.chandle, channel, ctypes.byref(self.channel_out), ctypes.byref(self.bufferAMin), self.n_points, 0, 0)# Setting the data buffer location for data collection from channel A
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
            print("\tvoltRange: " + self.voltRange)
            print(self.status)
        return self.voltDiv, self.timeDiv, self.sampleRate

    def setTriggerChannel(self, channel, enable=0, threshold=1024, timeout=1000):
        self.status["trigger"] = ps6.ps6000SetSimpleTrigger(self.chandle, enable, channel, threshold, ps6.PS6000_THRESHOLD_DIRECTION['PS6000_RISING'], 0, timeout)# Sets up single trigger
        assert_pico_ok(self.status["trigger"])

        if(DEBUG_MODE):
            print("Trigger channel")
            print("\tchannel: " + channelName(channel))
            print("\ttimeout: " + str(timeout))


    def arm(self):
        # Starts block capture
        self.status["runblock"] = ps6.ps6000RunBlock(self.chandle, 0, self.n_points, self.timebase, 1, None, 0, None, None)
        assert_pico_ok(self.status["runblock"])
        
        if(DEBUG_MODE):
            print("Block capture started.")

    def getNativeSignalBytes(self, AS_BYTES=True):
            # Checks data collection to finish the capture
            ready = ctypes.c_int16(0)
            check = ctypes.c_int16(0)
            while ready.value == check.value:
                self.status["isReady"] = ps6.ps6000IsReady(self.chandle, ctypes.byref(ready))

            self.cmaxSamples = ctypes.c_int32(self.n_points)
            self.status["GetValues"] = ps6.ps6000GetValues(self.chandle, 0, ctypes.byref(self.cmaxSamples), 1, 0, 0, ctypes.byref(self.overflow))

            assert_pico_ok(self.status["GetValues"])

            # Converts ADC from channel A to mV
            # channel_out_interpreted =  adc2mV(channel_out, chARange, maxADC)

            if AS_BYTES:
                # Scale the output signal to interpret as bytes
                channel_out = rawToBytes(self.channel_out, self.minADC.value, self.maxADC.value, self.cmaxSamples.value)[0].tobytes()
            else:
                # Return the full resolution of the channel (16bit)
                channel_out = self.channel_out

            return channel_out


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
