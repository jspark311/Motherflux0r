###########################################################################
# Makefile for Motherflux0r library support
# Author: J. Ian Lindsay
#
# Relies on exports from top-level Makefile.
###########################################################################
LIBRARY_NAME       = libmotherflux0r


###########################################################################
# Source file definitions...
###########################################################################

# Teensyduino
SOURCES_S    = $(TEENSY_CORE_PATH)/memcpy-armv7m.S
SOURCES_S   += $(TEENSY_CORE_PATH)/memset.S

SOURCES_C    = $(TEENSY_CORE_PATH)/delay.c
SOURCES_C   += $(TEENSY_CORE_PATH)/debugprintf.c
SOURCES_C   += $(TEENSY_CORE_PATH)/eeprom.c
SOURCES_C   += $(TEENSY_CORE_PATH)/extmem.c
SOURCES_C   += $(TEENSY_CORE_PATH)/fuse.c
SOURCES_C   += $(TEENSY_CORE_PATH)/libc.c
SOURCES_C   += $(TEENSY_CORE_PATH)/digital.c
SOURCES_C   += $(TEENSY_CORE_PATH)/analog.c
SOURCES_C   += $(TEENSY_CORE_PATH)/interrupt.c
SOURCES_C   += $(TEENSY_CORE_PATH)/bootdata.c
SOURCES_C   += $(TEENSY_CORE_PATH)/clockspeed.c
SOURCES_C   += $(TEENSY_CORE_PATH)/keylayouts.c
SOURCES_C   += $(TEENSY_CORE_PATH)/nonstd.c
SOURCES_C   += $(TEENSY_CORE_PATH)/pwm.c
SOURCES_C   += $(TEENSY_CORE_PATH)/rtc.c
SOURCES_C   += $(TEENSY_CORE_PATH)/startup.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_alloc_valid.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_calloc.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_free.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_hash.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_malloc.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_malloc_stats.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_pool.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_realloc.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_realloc_i.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_realloc_move.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_szalloc.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_util.c
SOURCES_C   += $(TEENSY_CORE_PATH)/sm_zalloc.c
SOURCES_C   += $(TEENSY_CORE_PATH)/tempmon.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_desc.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_joystick.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_keyboard.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_midi.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_mouse.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_mtp.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_rawhid.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_seremu.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_serial.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_serial2.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_serial3.c
SOURCES_C   += $(TEENSY_CORE_PATH)/usb_touch.c

SOURCES_CPP  = $(TEENSY_CORE_PATH)/HardwareSerial2.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/AudioStream.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/HardwareSerial4.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/DMAChannel.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/HardwareSerial.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/HardwareSerial3.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/HardwareSerial1.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/EventResponder.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/HardwareSerial5.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/HardwareSerial6.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/HardwareSerial7.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/HardwareSerial8.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/IPAddress.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/IntervalTimer.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/Print.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/main.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/new.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/serialEvent.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/serialEvent1.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/serialEvent2.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/serialEvent3.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/serialEvent4.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/serialEvent5.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/serialEvent6.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/serialEvent7.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/serialEvent8.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/serialEventUSB1.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/serialEventUSB2.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/usb_audio.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/usb_flightsim.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/usb_inst.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/Stream.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/Time.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/Tone.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/WMath.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/WString.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/yield.cpp
SOURCES_CPP += $(TEENSY_CORE_PATH)/CrashReport.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/SPI/SPI.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/SerialFlash/SerialFlashChip.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/SerialFlash/SerialFlashDirectory.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/EEPROM/EEPROM.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Time/DateStrings.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Time/Time.cpp


# Teensy Audio library
SOURCES_S   += $(ARDUINO_LIBS)/Audio/memcpy_audio.S

SOURCES_C   += $(ARDUINO_LIBS)/Audio/data_waveforms.c
SOURCES_C   += $(ARDUINO_LIBS)/Audio/data_ulaw.c
SOURCES_C   += $(ARDUINO_LIBS)/Audio/data_windows.c
SOURCES_C   += $(ARDUINO_LIBS)/Audio/data_spdif.c
SOURCES_C   += $(ARDUINO_LIBS)/Audio/utility/sqrt_integer.c

SOURCES_CPP  = $(ARDUINO_LIBS)/Audio/analyze_rms.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/analyze_fft256.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/Quantizer.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/analyze_fft1024.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/analyze_print.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/analyze_notefreq.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/Resampler.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/analyze_peak.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/analyze_tonedetect.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/async_input_spdif3.cpp
# Requires Wire shim.
#SOURCES_CPP += $(ARDUINO_LIBS)/Audio/control_ak4558.cpp
#SOURCES_CPP += $(ARDUINO_LIBS)/Audio/control_cs42448.cpp
#SOURCES_CPP += $(ARDUINO_LIBS)/Audio/control_cs4272.cpp
#SOURCES_CPP += $(ARDUINO_LIBS)/Audio/control_sgtl5000.cpp
#SOURCES_CPP += $(ARDUINO_LIBS)/Audio/control_tlv320aic3206.cpp
#SOURCES_CPP += $(ARDUINO_LIBS)/Audio/control_wm8731.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_bitcrusher.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_chorus.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_combine.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_delay.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_delay_ext.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_envelope.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_fade.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_flange.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_freeverb.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_granular.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_midside.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_multiply.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_rectifier.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_reverb.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/effect_waveshaper.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/filter_biquad.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/filter_fir.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/filter_variable.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/input_adc.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/input_adcs.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/input_i2s.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/input_i2s2.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/input_i2s_hex.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/input_i2s_oct.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/input_i2s_quad.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/input_pdm.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/input_spdif3.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/input_tdm.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/input_tdm2.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/mixer.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_adat.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_dac.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_dacs.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_i2s.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_i2s2.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_i2s_hex.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_i2s_oct.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_i2s_quad.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_mqs.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_pt8211.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_pt8211_2.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_pwm.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_spdif.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_spdif2.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_spdif3.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_tdm.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/output_tdm2.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/play_memory.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/play_queue.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/play_sd_raw.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/play_sd_wav.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/play_serialflash_raw.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/record_queue.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/spi_interrupt.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/synth_dc.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/synth_karplusstrong.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/synth_pinknoise.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/synth_pwm.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/synth_simple_drum.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/synth_sine.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/synth_tonesweep.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/synth_waveform.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/synth_wavetable.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/synth_whitenoise.cpp
SOURCES_CPP += $(ARDUINO_LIBS)/Audio/utility/imxrt_hw.cpp

# Non-blocking i2c driver
SOURCES_CPP += teensy4_i2c/src/i2c_register_slave.cpp
SOURCES_CPP += teensy4_i2c/src/i2c_driver_wire.cpp
SOURCES_CPP += teensy4_i2c/src/imx_rt1060/imx_rt1060_i2c_driver.cpp
SOURCES_CPP += teensy4_i2c/src/tests/ina260_reliability.cpp
SOURCES_CPP += teensy4_i2c/src/tests/loopback_reliability.cpp
SOURCES_CPP += teensy4_i2c/src/tests/raw_loopback.cpp
SOURCES_CPP += teensy4_i2c/src/tests/raspberry_pi_reliability.cpp

# C3P (Manuvr) platform shim
SOURCES_CPP += ManuvrArduino/src/Teensy4/ConcretePlatform.cpp
SOURCES_CPP += ManuvrArduino/src/Teensy4/I2CAdapter.cpp
SOURCES_CPP += ManuvrArduino/src/Teensy4/SPIAdapter.cpp
SOURCES_CPP += ManuvrArduino/src/Teensy4/UART.cpp

# C3P itself
SOURCES_CPP += CppPotpourri/src/AbstractPlatform.cpp
SOURCES_CPP += CppPotpourri/src/AsyncSequencer.cpp
SOURCES_CPP += CppPotpourri/src/EnumeratedTypeCodes.cpp
SOURCES_CPP += CppPotpourri/src/CppPotpourri.cpp
SOURCES_CPP += CppPotpourri/src/Quaternion.cpp
SOURCES_CPP += CppPotpourri/src/Meta/Bikeshed/SensorWrapper.cpp
SOURCES_CPP += CppPotpourri/src/TimeSeries/SensorFilter.cpp
SOURCES_CPP += CppPotpourri/src/StringBuilder.cpp
SOURCES_CPP += CppPotpourri/src/Battery.cpp
SOURCES_CPP += CppPotpourri/src/uuid.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/BusQueue.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/I2CDevice.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/I2CBusOp.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/I2CAdapter.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/SPIAdapter.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/SPIBusOp.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/UARTAdapter.cpp
SOURCES_CPP += CppPotpourri/src/M2MLink/M2MLink.cpp
SOURCES_CPP += CppPotpourri/src/M2MLink/M2MMsg.cpp
SOURCES_CPP += CppPotpourri/src/M2MLink/M2MMsgHdr.cpp
SOURCES_CPP += CppPotpourri/src/Identity/Identity.cpp
SOURCES_CPP += CppPotpourri/src/Identity/IdentityUUID.cpp
SOURCES_CPP += CppPotpourri/src/Storage/RecordTypes/ConfRecord.cpp
SOURCES_CPP += CppPotpourri/src/Storage/SimpleDataRecord.cpp
SOURCES_CPP += CppPotpourri/src/Console/C3PConsole.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/BufferAccepter/GPSWrapper/GPSWrapper.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/TripleAxisPipe/TripleAxisCompass.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/TripleAxisPipe/TripleAxisPipe.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/BufferAccepter/BufferAccepter.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/BufferAccepter/Base64/C3P_B64.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/BufferAccepter/LineCoDec/LineCoDec.cpp
SOURCES_CPP += CppPotpourri/src/C3PValue/C3PType.cpp
SOURCES_CPP += CppPotpourri/src/C3PValue/C3PValue.cpp
SOURCES_CPP += CppPotpourri/src/C3PValue/KeyValuePair.cpp
SOURCES_CPP += CppPotpourri/src/Image/Image.cpp
SOURCES_CPP += CppPotpourri/src/Image/ImageUtils/ImageUtils.cpp
SOURCES_CPP += CppPotpourri/src/Image/ImageUtils/ImageGraph.cpp
SOURCES_CPP += CppPotpourri/src/Image/ImageUtils/BlobPlotter.cpp
SOURCES_CPP += CppPotpourri/src/TimerTools/StopWatch.cpp
SOURCES_CPP += CppPotpourri/src/TimerTools/C3PScheduler.cpp
SOURCES_CPP += CppPotpourri/src/TimerTools/C3PTrace.cpp
SOURCES_CPP += CppPotpourri/src/cbor-cpp/cbor.cpp

# Drivers
SOURCES_CPP += ManuvrDrivers/src/AMG88xx/AMG88xx.cpp
SOURCES_CPP += ManuvrDrivers/src/BME280/BME280.cpp
SOURCES_CPP += ManuvrDrivers/src/BQ24155/BQ24155.cpp
SOURCES_CPP += ManuvrDrivers/src/LSM9DS1/LSM9DS1.cpp
SOURCES_CPP += ManuvrDrivers/src/LTC294x/LTC294x.cpp
SOURCES_CPP += ManuvrDrivers/src/MCP356x/MCP356x.cpp
SOURCES_CPP += ManuvrDrivers/src/MCP356x/MCP356x_Util.cpp
SOURCES_CPP += ManuvrDrivers/src/MCP356x/MCP356x_Regs.cpp
SOURCES_CPP += ManuvrDrivers/src/SSD13xx/SSD13xx.cpp
SOURCES_CPP += ManuvrDrivers/src/SSD13xx/SSD1331.cpp
SOURCES_CPP += ManuvrDrivers/src/SX1503/SX1503.cpp
SOURCES_CPP += ManuvrDrivers/src/SX8634/SX8634.cpp
SOURCES_CPP += ManuvrDrivers/src/TSL2561/TSL2561.cpp
SOURCES_CPP += ManuvrDrivers/src/VEML6075/VEML6075.cpp
SOURCES_CPP += ManuvrDrivers/src/VL53L0X/VL53L0X.cpp
SOURCES_CPP += ManuvrDrivers/src/Composites/ManuvrPMU/ManuvrPMU-r2.cpp


###########################################################################
# exports, consolidation....
###########################################################################
OBJS  = $(SOURCES_C:.c=.o) $(SOURCES_CPP:.cpp=.o) $(SOURCES_S:.S=.o)
DEP_FILES = $(SOURCES_C:.c=.d) $(SOURCES_CPP:.cpp=.d) $(SOURCES_S:.S=.d)


###########################################################################
# Rules for building the firmware follow...
###########################################################################

.PHONY:  $(OUTPUT_PATH)/$(LIBRARY_NAME).a

all:  $(OUTPUT_PATH)/$(LIBRARY_NAME).a

%.o : %.cpp
	$(CXX) -std=$(CPP_STANDARD) $(OPTIMIZATION) $(CXXFLAGS) -c -o $@ $^

%.o : %.c
	$(CC) -std=$(C_STANDARD) $(OPTIMIZATION) $(CFLAGS) -c -o $@ $^

%.o : %.S
	$(CC) -std=$(C_STANDARD) $(OPTIMIZATION) -x assembler-with-cpp $(CFLAGS) -c -o $@ $^


$(OUTPUT_PATH)/$(LIBRARY_NAME).a: $(OBJS)
	$(AR) rcs $(OUTPUT_PATH)/$(LIBRARY_NAME).a $(OBJS)

clean:
	rm -f *.d *.o *.su *~ $(OBJS) $(DEP_FILES) $(OUTPUT_PATH)/$(LIBRARY_NAME).a
