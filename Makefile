###########################################################################
# Makefile for Motherflux0r
# Author: J. Ian Lindsay
# Date:   2021.05.16
#
###########################################################################
FIRMWARE_NAME      = Motherflux0r

MCU                = cortex-m7
CPU_SPEED          = 600000000
OPTIMIZATION       = -O2 -g
C_STANDARD         = gnu99
CPP_STANDARD       = gnu++14


###########################################################################
# Environmental awareness...
###########################################################################
WHO_I_AM       = $(shell whoami)
ARDUINO_PATH   = /opt/Arduino
export TEENSY_PATH    = $(ARDUINO_PATH)/hardware/teensy/avr

# This is where we will store compiled libs and the final output.
export BUILD_ROOT   = $(shell pwd)
export OUTPUT_PATH  = $(BUILD_ROOT)/build

TOOLCHAIN          = $(ARDUINO_PATH)/hardware/tools/arm
TEENSY_LOADER_PATH = $(BUILD_ROOT)/tools/teensy_loader_cli/teensy_loader_cli
FORMAT             = ihex

export CC      = $(TOOLCHAIN)/bin/arm-none-eabi-gcc
export CXX     = $(TOOLCHAIN)/bin/arm-none-eabi-g++
export AR      = $(TOOLCHAIN)/bin/arm-none-eabi-ar
export AS      = $(TOOLCHAIN)/bin/arm-none-eabi-as
export OBJCOPY = $(TOOLCHAIN)/bin/arm-none-eabi-objcopy
export SZ      = $(TOOLCHAIN)/bin/arm-none-eabi-size
export MAKE    = $(shell which make)


###########################################################################
# Source files, includes, and linker directives...
###########################################################################
CXXFLAGS     = -felide-constructors -fno-exceptions -fno-rtti -MMD
CXXFLAGS    += -fpermissive -fno-threadsafe-statics

CFLAGS       = -Wall -nostdlib
CFLAGS      += -DF_CPU=$(CPU_SPEED)
CFLAGS      += -mcpu=$(MCU) -mthumb -D__IMXRT1062__
CFLAGS      += -ffunction-sections -fdata-sections
CFLAGS      += -Wno-error=narrowing
CFLAGS      += -mlittle-endian
CFLAGS      += -mfloat-abi=hard -mfpu=fpv5-d16
CFLAGS      += -DARDUINO=10813 -DTEENSYDUINO=153 -DARDUINO_TEENSY40
CFLAGS      += -DUSB_VID=null -DUSB_PID=null -DUSB_SERIAL -DLAYOUT_US_ENGLISH

LIBS         = -larm_cortexM7lfsp_math -lm -lstdc++
LD_FILE      = $(TEENSY_PATH)/cores/teensy4/imxrt1062.ld

INCLUDES     = -iquote. -iquotesrc/
INCLUDES    += -I$(BUILD_ROOT)/src/
INCLUDES    += -I$(BUILD_ROOT)/lib/CppPotpourri/src
INCLUDES    += -I$(BUILD_ROOT)/lib/ManuvrDrivers/src
INCLUDES    += -I$(BUILD_ROOT)/lib/ManuvrArduino/src
INCLUDES    += -I$(BUILD_ROOT)/lib/teensy4_i2c/src
INCLUDES    += -I$(TEENSY_PATH)/libraries
INCLUDES    += -I$(TEENSY_PATH)/cores/teensy4
INCLUDES    += -Ilib/Audio -Ilib/Audio/utility
INCLUDES    += -I$(TEENSY_PATH)/libraries/SD
INCLUDES    += -I$(TEENSY_PATH)/libraries/SPI
INCLUDES    += -I$(TEENSY_PATH)/libraries/SerialFlash
INCLUDES    += -I$(TEENSY_PATH)/libraries/EEPROM
INCLUDES    += -I$(TEENSY_PATH)/libraries/Time



###########################################################################
# Source file definitions...
###########################################################################
SOURCES_S    = lib/Audio/memcpy_audio.S
SOURCES_S   += /opt/Arduino/hardware/teensy/avr/cores/teensy4/memcpy-armv7m.S
SOURCES_S   += /opt/Arduino/hardware/teensy/avr/cores/teensy4/memset.S

SOURCES_C    = src/ICM_20948_C.c
SOURCES_C   += lib/Audio/data_waveforms.c
SOURCES_C   += lib/Audio/data_ulaw.c
SOURCES_C   += lib/Audio/data_windows.c
SOURCES_C   += lib/Audio/data_spdif.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/delay.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/debugprintf.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/eeprom.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/digital.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/analog.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/interrupt.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/bootdata.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/clockspeed.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/keylayouts.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/nonstd.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/pwm.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/rtc.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/startup.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/tempmon.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb_desc.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb_joystick.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb_keyboard.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb_midi.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb_mouse.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb_rawhid.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb_seremu.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb_serial.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb_serial2.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb_serial3.c
SOURCES_C   += $(TEENSY_PATH)/cores/teensy4/usb_touch.c
SOURCES_C   += lib/Audio/utility/sqrt_integer.c

SOURCES_CPP  = src/main.cpp
SOURCES_CPP += src/bitmaps.cpp
SOURCES_CPP += src/DRV425.cpp
SOURCES_CPP += src/ICM20948.cpp
SOURCES_CPP += src/Motherflux0rSupport.cpp
SOURCES_CPP += src/SensorGlue.cpp
SOURCES_CPP += src/uAppBoot.cpp
SOURCES_CPP += src/uAppComms.cpp
SOURCES_CPP += src/uAppConfigurator.cpp
SOURCES_CPP += src/uApp.cpp
SOURCES_CPP += src/uAppDataMgmt.cpp
SOURCES_CPP += src/uAppMeta.cpp
SOURCES_CPP += src/uAppRoot.cpp
SOURCES_CPP += src/uAppStandby.cpp
SOURCES_CPP += src/uAppSynthBox.cpp
SOURCES_CPP += src/uAppTouchTest.cpp
SOURCES_CPP += src/uAppTricorder.cpp
SOURCES_CPP += src/uAppLocation.cpp
SOURCES_CPP += src/Storage/CalConfRecord.cpp
SOURCES_CPP += src/Storage/ConfRecord.cpp
SOURCES_CPP += src/Storage/UsrConfRecord.cpp

SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/HardwareSerial2.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/AudioStream.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/HardwareSerial4.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/DMAChannel.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/HardwareSerial.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/HardwareSerial3.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/HardwareSerial1.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/EventResponder.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/HardwareSerial5.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/HardwareSerial6.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/HardwareSerial7.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/HardwareSerial8.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/IPAddress.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/IntervalTimer.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/Print.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/Stream.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/WMath.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/Tone.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/WString.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/main.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/new.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/serialEvent.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/serialEvent1.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/serialEvent2.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/serialEvent3.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/serialEvent4.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/serialEvent5.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/serialEvent6.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/serialEvent7.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/serialEvent8.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/serialEventUSB1.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/serialEventUSB2.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/usb_audio.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/usb_flightsim.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/usb_inst.cpp
SOURCES_CPP += $(TEENSY_PATH)/cores/teensy4/yield.cpp

SOURCES_CPP += lib/CppPotpourri/src/AbstractPlatform.cpp
SOURCES_CPP += lib/CppPotpourri/src/ParsingConsole.cpp
SOURCES_CPP += lib/CppPotpourri/src/GPSWrapper.cpp
SOURCES_CPP += lib/CppPotpourri/src/I2CDevice.cpp
SOURCES_CPP += lib/CppPotpourri/src/I2CBusOp.cpp
SOURCES_CPP += lib/CppPotpourri/src/BusQueue.cpp
SOURCES_CPP += lib/CppPotpourri/src/CppPotpourri.cpp
SOURCES_CPP += lib/CppPotpourri/src/I2CAdapter.cpp
SOURCES_CPP += lib/CppPotpourri/src/Quaternion.cpp
SOURCES_CPP += lib/CppPotpourri/src/SPIAdapter.cpp
SOURCES_CPP += lib/CppPotpourri/src/SPIBusOp.cpp
SOURCES_CPP += lib/CppPotpourri/src/SensorDatum.cpp
SOURCES_CPP += lib/CppPotpourri/src/SensorFilter.cpp
SOURCES_CPP += lib/CppPotpourri/src/SensorManager.cpp
SOURCES_CPP += lib/CppPotpourri/src/SensorWrapper.cpp
SOURCES_CPP += lib/CppPotpourri/src/StopWatch.cpp
SOURCES_CPP += lib/CppPotpourri/src/Storage.cpp
SOURCES_CPP += lib/CppPotpourri/src/StringBuilder.cpp
SOURCES_CPP += lib/CppPotpourri/src/TripleAxisCompass.cpp
SOURCES_CPP += lib/CppPotpourri/src/TripleAxisPipe.cpp
SOURCES_CPP += lib/CppPotpourri/src/UARTAdapter.cpp
SOURCES_CPP += lib/CppPotpourri/src/Battery.cpp
SOURCES_CPP += lib/CppPotpourri/src/WakeLock.cpp
SOURCES_CPP += lib/CppPotpourri/src/uuid.cpp
SOURCES_CPP += lib/CppPotpourri/src/Image/Image.cpp
SOURCES_CPP += lib/CppPotpourri/src/cbor-cpp/cbor.cpp

SOURCES_CPP += lib/ManuvrDrivers/src/AMG88xx/AMG88xx.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/BME280/BME280.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/BQ24155/BQ24155.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/LSM9DS1/LSM9DS1.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/LTC294x/LTC294x.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/MCP356x/MCP356x.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/MCP356x/MCP356x_Util.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/MCP356x/MCP356x_Regs.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/SSD13xx/SSD1331.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/SX1503/SX1503.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/SX8634/SX8634.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/TSL2561/TSL2561.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/VEML6075/VEML6075.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/VL53L0X/VL53L0X.cpp
SOURCES_CPP += lib/ManuvrDrivers/src/Composites/ManuvrPMU/ManuvrPMU-r2.cpp

SOURCES_CPP += lib/Audio/analyze_rms.cpp
SOURCES_CPP += lib/Audio/analyze_fft256.cpp
SOURCES_CPP += lib/Audio/Quantizer.cpp
SOURCES_CPP += lib/Audio/analyze_fft1024.cpp
SOURCES_CPP += lib/Audio/analyze_print.cpp
SOURCES_CPP += lib/Audio/analyze_notefreq.cpp
SOURCES_CPP += lib/Audio/Resampler.cpp
SOURCES_CPP += lib/Audio/analyze_peak.cpp
SOURCES_CPP += lib/Audio/analyze_tonedetect.cpp
SOURCES_CPP += lib/Audio/async_input_spdif3.cpp
# Requires Wire shim.
#SOURCES_CPP += lib/Audio/control_ak4558.cpp
#SOURCES_CPP += lib/Audio/control_cs42448.cpp
#SOURCES_CPP += lib/Audio/control_cs4272.cpp
#SOURCES_CPP += lib/Audio/control_sgtl5000.cpp
#SOURCES_CPP += lib/Audio/control_tlv320aic3206.cpp
#SOURCES_CPP += lib/Audio/control_wm8731.cpp
SOURCES_CPP += lib/Audio/effect_bitcrusher.cpp
SOURCES_CPP += lib/Audio/effect_chorus.cpp
SOURCES_CPP += lib/Audio/effect_combine.cpp
SOURCES_CPP += lib/Audio/effect_delay.cpp
SOURCES_CPP += lib/Audio/effect_delay_ext.cpp
SOURCES_CPP += lib/Audio/effect_envelope.cpp
SOURCES_CPP += lib/Audio/effect_fade.cpp
SOURCES_CPP += lib/Audio/effect_flange.cpp
SOURCES_CPP += lib/Audio/effect_freeverb.cpp
SOURCES_CPP += lib/Audio/effect_granular.cpp
SOURCES_CPP += lib/Audio/effect_midside.cpp
SOURCES_CPP += lib/Audio/effect_multiply.cpp
SOURCES_CPP += lib/Audio/effect_rectifier.cpp
SOURCES_CPP += lib/Audio/effect_reverb.cpp
SOURCES_CPP += lib/Audio/effect_waveshaper.cpp
SOURCES_CPP += lib/Audio/filter_biquad.cpp
SOURCES_CPP += lib/Audio/filter_fir.cpp
SOURCES_CPP += lib/Audio/filter_variable.cpp
SOURCES_CPP += lib/Audio/input_adc.cpp
SOURCES_CPP += lib/Audio/input_adcs.cpp
SOURCES_CPP += lib/Audio/input_i2s.cpp
SOURCES_CPP += lib/Audio/input_i2s2.cpp
SOURCES_CPP += lib/Audio/input_i2s_hex.cpp
SOURCES_CPP += lib/Audio/input_i2s_oct.cpp
SOURCES_CPP += lib/Audio/input_i2s_quad.cpp
SOURCES_CPP += lib/Audio/input_pdm.cpp
SOURCES_CPP += lib/Audio/input_spdif3.cpp
SOURCES_CPP += lib/Audio/input_tdm.cpp
SOURCES_CPP += lib/Audio/input_tdm2.cpp
SOURCES_CPP += lib/Audio/mixer.cpp
SOURCES_CPP += lib/Audio/output_adat.cpp
SOURCES_CPP += lib/Audio/output_dac.cpp
SOURCES_CPP += lib/Audio/output_dacs.cpp
SOURCES_CPP += lib/Audio/output_i2s.cpp
SOURCES_CPP += lib/Audio/output_i2s2.cpp
SOURCES_CPP += lib/Audio/output_i2s_hex.cpp
SOURCES_CPP += lib/Audio/output_i2s_oct.cpp
SOURCES_CPP += lib/Audio/output_i2s_quad.cpp
SOURCES_CPP += lib/Audio/output_mqs.cpp
SOURCES_CPP += lib/Audio/output_pt8211.cpp
SOURCES_CPP += lib/Audio/output_pt8211_2.cpp
SOURCES_CPP += lib/Audio/output_pwm.cpp
SOURCES_CPP += lib/Audio/output_spdif.cpp
SOURCES_CPP += lib/Audio/output_spdif2.cpp
SOURCES_CPP += lib/Audio/output_spdif3.cpp
SOURCES_CPP += lib/Audio/output_tdm.cpp
SOURCES_CPP += lib/Audio/output_tdm2.cpp
SOURCES_CPP += lib/Audio/play_memory.cpp
SOURCES_CPP += lib/Audio/play_queue.cpp
SOURCES_CPP += lib/Audio/play_sd_raw.cpp
SOURCES_CPP += lib/Audio/play_sd_wav.cpp
SOURCES_CPP += lib/Audio/play_serialflash_raw.cpp
SOURCES_CPP += lib/Audio/record_queue.cpp
SOURCES_CPP += lib/Audio/spi_interrupt.cpp
SOURCES_CPP += lib/Audio/synth_dc.cpp
SOURCES_CPP += lib/Audio/synth_karplusstrong.cpp
SOURCES_CPP += lib/Audio/synth_pinknoise.cpp
SOURCES_CPP += lib/Audio/synth_pwm.cpp
SOURCES_CPP += lib/Audio/synth_simple_drum.cpp
SOURCES_CPP += lib/Audio/synth_sine.cpp
SOURCES_CPP += lib/Audio/synth_tonesweep.cpp
SOURCES_CPP += lib/Audio/synth_waveform.cpp
SOURCES_CPP += lib/Audio/synth_wavetable.cpp
SOURCES_CPP += lib/Audio/synth_whitenoise.cpp
SOURCES_CPP += lib/Audio/utility/imxrt_hw.cpp
SOURCES_CPP += $(TEENSY_PATH)/libraries/SPI/SPI.cpp
SOURCES_CPP += $(TEENSY_PATH)/libraries/SerialFlash/SerialFlashChip.cpp
SOURCES_CPP += $(TEENSY_PATH)/libraries/SerialFlash/SerialFlashDirectory.cpp
SOURCES_CPP += $(TEENSY_PATH)/libraries/EEPROM/EEPROM.cpp
SOURCES_CPP += $(TEENSY_PATH)/libraries/Time/DateStrings.cpp
SOURCES_CPP += $(TEENSY_PATH)/libraries/Time/Time.cpp
#SOURCES_CPP += /home/ian/sketchbook/libraries/SD/src/File.cpp
#SOURCES_CPP += /home/ian/sketchbook/libraries/SD/src/SD.cpp
#SOURCES_CPP += /home/ian/sketchbook/libraries/SD/src/utility/Sd2Card.cpp
#SOURCES_CPP += /home/ian/sketchbook/libraries/SD/src/utility/SdFile.cpp
#SOURCES_CPP += /home/ian/sketchbook/libraries/SD/src/utility/SdVolume.cpp
SOURCES_CPP += lib/ManuvrArduino/src/Teensy4/ConcretePlatform.cpp
SOURCES_CPP += lib/ManuvrArduino/src/Teensy4/I2CAdapter.cpp
SOURCES_CPP += lib/ManuvrArduino/src/Teensy4/SPIAdapter.cpp
SOURCES_CPP += lib/teensy4_i2c/src/i2c_register_slave.cpp
SOURCES_CPP += lib/teensy4_i2c/src/i2c_driver_wire.cpp
SOURCES_CPP += lib/teensy4_i2c/src/imx_rt1060/imx_rt1060_i2c_driver.cpp
SOURCES_CPP += lib/teensy4_i2c/src/tests/ina260_reliability.cpp
SOURCES_CPP += lib/teensy4_i2c/src/tests/loopback_reliability.cpp
SOURCES_CPP += lib/teensy4_i2c/src/tests/raw_loopback.cpp
SOURCES_CPP += lib/teensy4_i2c/src/tests/raspberry_pi_reliability.cpp



###########################################################################
# exports, consolidation....
###########################################################################
OBJS  = $(SOURCES_C:.c=.o) $(SOURCES_CPP:.cpp=.o) $(SOURCES_S:.S=.o)
DEP_FILES = $(SOURCES_C:.c=.d) $(SOURCES_CPP:.cpp=.d) $(SOURCES_S:.S=.d)

# Merge our choices and export them to the downstream Makefiles...
CFLAGS += $(OPTIMIZATION) $(INCLUDES)

export CFLAGS
export CXXFLAGS  += $(CFLAGS)


###########################################################################
# Rules for building the firmware follow...
###########################################################################

.PHONY:  lib $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf

all:  lib $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf

%.o : %.cpp
	$(CXX) -std=$(CPP_STANDARD) $(OPTIMIZATION) $(CXXFLAGS) -c -o $@ $^

%.o : %.c
	$(CC) -std=$(C_STANDARD) $(OPTIMIZATION) $(CFLAGS) -c -o $@ $^

%.o : %.S
	$(CC) -std=$(C_STANDARD) $(OPTIMIZATION) -x assembler-with-cpp $(CFLAGS) -c -o $@ $^


lib: $(OBJS)
	mkdir -p $(OUTPUT_PATH)
	#$(MAKE) -C lib

$(OUTPUT_PATH)/$(FIRMWARE_NAME).elf:
	$(CC) $(OPTIMIZATION) -Wl,--gc-sections,--relax -T$(LD_FILE) -mcpu=$(MCU) -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -o $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OBJS) -L$(OUTPUT_PATH) $(LIBS)
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).eep
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf $(OUTPUT_PATH)/$(FIRMWARE_NAME).hex
	$(SZ) -A $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf

flash: $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf
	$(TEENSY_LOADER_PATH) -mmcu=TEENSY40 -s -w -v $(OUTPUT_PATH)/$(FIRMWARE_NAME).hex

clean:
	rm -f *.d *.o *.su *~ $(OBJS) $(DEP_FILES)

fullclean: clean
	rm -rf $(OUTPUT_PATH)
	#$(MAKE) clean -C lib
