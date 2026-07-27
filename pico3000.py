import numpy as np

from picosdk.errors import *
from picosdk.ps3000a import ps3000a as ps3

import ctypes
from picosdk.functions import adc2mV, assert_pico_ok


import sys
import os
import time
import numpy as np
import serial
import matplotlib.pyplot as plt
import trsfile
from ttest import ttest
import time
import chipwhisperer as cw
from pico import *
import os
import shutil
import subprocess
#traces/RSA1_20251003_150346.trs

ONLY_CW = True
COLLECT_TRACES = True
TTEST_STATUS = False
plot_refresh_rate = 100

SRC_PATH = os.path.expanduser("~/chipwhisperer/firmware/mcu/sca-rsa/")


DEBUG_MODE = True

PLATFORM = 'CW308_STM32F3'
CRYPTO_TARGET = 'NONE'
SS_VER = 'SS_VER_1_1'

DATA_LEN=128
RESP_LEN=128


# Create the full shell command string
cmd = f"""
rm -rf objdir-{PLATFORM} objdir .dep
make SS_VER={SS_VER} PLATFORM={PLATFORM} CRYPTO_TARGET={CRYPTO_TARGET}
"""

def collect_cw(scope, target, cmd,textin):
    scope.adc.offset = 0
    scope.adc.samples = 131070
    scope.adc.decimate = 1
    scope.clock.adc_src = 'clkgen_x1'
    scope.gain.db = 30
    scope.clock.clkgen_src = 'system'
    scope.clock.adc_mul = 1
    scope.adc.decimate = 1
  
   
    target.flush()
    target.simpleserial_write(cmd, textin)
    res =target.simpleserial_read('r', RESP_LEN, timeout=0)
    #print(res) 


def set_exp(scope, target, cmd, textin):
     target.simpleserial_write(cmd, textin)


if __name__=="__main__":

    # program start
    start_time = time.time()


    # number of trace to collect
    n_traces = 3000

    # Configure output files
    if ONLY_CW and COLLECT_TRACES:
        current_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())#str(int(time.time()))
        traceFileName = "traces/" + "RSA3" + "protected" + "{}_{}".format(n_traces, current_time) + ".trs"
       
    else:
        n_traces = 1000 #Set big number of traces to keep computation going when setting parameters in the picoscope GUI.

    ## Scope parameters (Consider a 10div timespace and 10div voltage space)
    voltDiv = 2e-1 #V/div # result will be voltDiv * 5
    timeDiv = 5e-3  #s/div
    sampleRate = 0.24e9 #S/s
    total_samples= int(30e7)#int(10 * timeDiv * sampleRate)#int(40e7)#int(10*timeDiv*sampleRate) #time when trigger goes off * sampling frequency (maximum is 10*timeDiv*sampleRate?)
    preTrigger = 0#int(total_samples * 0.1) #int(0.1*(sampleRate*timeDiv*10)) #number of samples to collect before the trigger
    offset=20e-2
    n_samples = total_samples - preTrigger


    if  ONLY_CW and COLLECT_TRACES:
        ## Init Scope
        scope = pico3000()
        scope.connect()
        voltDiv, timeDiv, sampleRate = scope.setChannel(0, voltDiv, sampleRate, timeDiv, n_samples,offset)
        #scope.setChannel(1, voltDiv, sampleRate, timeDiv, n_samples, offset)
        print("Scope settings:\n\tvoltDiv: {:e}\n\tvoltRange: {}\n\ttimeDiv: {:e}\n\tsampleRate: {:e}".format(voltDiv,scope.voltRange,timeDiv,sampleRate))
        print(dir(scope))
        scope.setTriggerChannel(4,enable=1)#4 is Ext Channel
        # --- DIGITAL MSO TRIGGER SETUP (D5) ---

        # 1. Enable Port 0 (Pins D0-D7) and set 1.5V threshold
        """
        # 1. Enable Digital Port 0 (D0-D7)
        status_port = ps3.ps3000aSetDigitalPort(scope.chandle, 0x80, 1, 1500)
        assert_pico_ok(status_port)

        # 2. Master Trigger Properties (CRITICAL: Sets the timeout to wait indefinitely)
        status_props = ps3.ps3000aSetTriggerChannelProperties(scope.chandle, None, 0, 0, 0)
        assert_pico_ok(status_props)

        # 3. Set D5 to Trigger on Rising Edge (With strict struct packing)
        class PS3000A_DIGITAL_CHANNEL_DIRECTIONS(ctypes.Structure):
            _pack_ = 1
            _fields_ = [("channel", ctypes.c_int32),
                        ("direction", ctypes.c_int32)]

        # 5 = D5, 3 = PS3000A_DIGITAL_DIRECTION_RISING
        dig_directions = PS3000A_DIGITAL_CHANNEL_DIRECTIONS(5, 3) 

        status_dir = ps3.ps3000aSetTriggerDigitalPortProperties(
            scope.chandle,
            ctypes.byref(dig_directions),
            1
        )
        assert_pico_ok(status_dir)

        # 4. Define Master Trigger Conditions
        class PS3000A_TRIGGER_CONDITIONS(ctypes.Structure):
            _pack_ = 1
            _fields_ = [("channelA", ctypes.c_int32),
                        ("channelB", ctypes.c_int32),
                        ("channelC", ctypes.c_int32),
                        ("channelD", ctypes.c_int32),
                        ("external", ctypes.c_int32),
                        ("aux", ctypes.c_int32),
                        ("pulseWidthQualifier", ctypes.c_int32),
                        ("digital", ctypes.c_int32)]

        # Set analog to 0 (DONT_CARE), and digital to 1 (TRUE)
        conditions = PS3000A_TRIGGER_CONDITIONS(0, 0, 0, 0, 0, 0, 0, 1)

        status_cond = ps3.ps3000aSetTriggerChannelConditions(
            scope.chandle,
            ctypes.byref(conditions),
            1
        )
        assert_pico_ok(status_cond)

        """



    ### CHIPWHISPERER SETUP

    scope_cw = cw.scope()

    target_type = cw.targets.SimpleSerial
    try:
        target = cw.target(scope_cw, target_type)
    except:
        print("INFO: Caught exception on reconnecting to target - attempting to reconnect to scope first.")
        print("INFO: This is a work-around when USB has died without Python knowing. Ignore errors above this line.")
        scope_cw = cw.scope()
        target = cw.target(scope_cw, target_type)

    print("INFO: Found ChipWhisperer😍")

    prog = cw.programmers.STM32FProgrammer

    time.sleep(0.05)
    scope_cw.default_setup()


    ### COMPILE CODE

    if not os.path.isdir(SRC_PATH):
        raise FileNotFoundError(f"SRC_PATH does not exist: {SRC_PATH}")

    os.chdir(SRC_PATH)
    print(f"Changed working directory to {SRC_PATH}")

    try:
        subprocess.run(cmd, shell=True, cwd=SRC_PATH, check=True, executable='/bin/bash')
        print("Commands executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Build failed with return code {e.returncode}")


    ### FLASH CW with hex file

    HEXFILE = SRC_PATH + "simpleserial_sca_rsa-" + PLATFORM + ".hex"
    cw.program_target(scope_cw, prog, HEXFILE)

    ### GENERARE PLAINTEXT

    textin_array = []
    textin_array = [os.urandom(DATA_LEN) for _ in range(n_traces)]

    os.chdir(os.path.dirname(__file__))

    CLOCK = scope_cw.clock.clkgen_freq
    BAUD = target.baud
    print('Clock: %f MHz\nBaud: %d bps' % (CLOCK, BAUD))






	### TRS HEADER INIT

    if ONLY_CW and COLLECT_TRACES:
        headers = {
            trsfile.Header.TRS_VERSION:2,
            trsfile.Header.DESCRIPTION: 'RSA ttest',
            trsfile.Header.NUMBER_SAMPLES:int(n_samples),
            trsfile.Header.LENGTH_DATA:1,
            trsfile.Header.SAMPLE_CODING:trsfile.SampleCoding.BYTE,
            trsfile.Header.LABEL_X:"s",
            trsfile.Header.LABEL_Y:"V",
            trsfile.Header.SCALE_X:10*timeDiv/n_samples,
            trsfile.Header.SCALE_Y:10*voltDiv/np.iinfo(np.uint8).max,
            trsfile.Header.TRACE_PARAMETER_DEFINITIONS: trsfile.parametermap.TraceParameterDefinitionMap(
                {'ttest':trsfile.traceparameter.TraceParameterDefinition(trsfile.traceparameter.ParameterType.BYTE, 1, 0)}
            ),
        }
        traceFile = trsfile.trs_open(traceFileName, mode='w', headers=headers)


    # Main loop
    numTotal = 0
    while numTotal < n_traces:
        ## Set Scope trigger

        if  ONLY_CW and COLLECT_TRACES: scope.arm(preTrigger=preTrigger)
        try:
            
        	# toss a coin to select Fixed or Random
            coin = np.random.randint(0,3)
            tmp = bytearray(os.urandom(DATA_LEN))
            ## RUN DECRYPTION VIA CHIPWHISPRER
            if( coin == 0):
                tmp[0] = 0
                set_exp(scope_cw, target, 'e', bytes(tmp))
                collect_cw(scope_cw, target, 'a',textin_array[0])
            elif coin == 1:
                tmp[0] = 0
                set_exp(scope_cw, target, 'e', bytes(tmp))
                collect_cw(scope_cw, target, 'a',textin_array[numTotal])
            else:
                tmp[0] = numTotal % 100
                set_exp(scope_cw, target, 'e',bytes(tmp))
                collect_cw(scope_cw, target, 'a',textin_array[0])

            ## Get data from Scope
            if  ONLY_CW and COLLECT_TRACES: 
                channel_out, channel_out_interpreted = scope.getNativeSignalBytes()
                ## Write data and trace in .TRS file
                traceFile.append(trsfile.Trace(
                    trsfile.SampleCoding.BYTE, 
                    channel_out, 
                    trsfile.parametermap.TraceParameterMap(
                        {'ttest':trsfile.parametermap.ByteArrayParameter([coin])}
                    )
                ))
                
            
            numTotal += 1 #Placed here, the loop will continue to run operation untill enough traces are successfully collected. If placed on top of the loop, will throw away uncollected traces.

        except Exception as ex:
            print("ERROR: ", ex)

        if numTotal % 100 == 0:
            print(numTotal)

    scope_cw.dis()
    if ONLY_CW and COLLECT_TRACES:
        print("TRSFILE: " + traceFileName)
    print("Done: " + str(numTotal))
    print("Total time: " + str(time.time() - start_time))

    if ONLY_CW and COLLECT_TRACES:
        # Close files
        traceFile.close()
        ## Close scope and serial port
        scope.disconnect()

