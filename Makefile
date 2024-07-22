###########################################################################
# Makefile for Motherflux0r
# Author: J. Ian Lindsay
# Date:   2021.05.16
#
###########################################################################
export FIRMWARE_NAME      = Motherflux0r
export MCU                = cortex-m7
export CPU_SPEED          = 600000000
export OPTIMIZATION       = -O2 -g
export C_STANDARD         = gnu99
export CPP_STANDARD       = gnu++17

###########################################################################
# Environmental awareness...
###########################################################################
WHO_I_AM           = $(shell whoami)
ARDUINO_PATH       = /home/$(WHO_I_AM)/.arduino15
export TEENSY_PATH      = $(ARDUINO_PATH)/packages/teensy
export ARDUINO_LIBS     = $(TEENSY_PATH)/hardware/avr/1.59.0/libraries
export TEENSY_CORE_PATH = $(TEENSY_PATH)/hardware/avr/1.59.0/cores/teensy4

TOOLCHAIN          = $(TEENSY_PATH)/tools/teensy-compile/11.3.1/arm
TEENSY_LOADER_PATH = tools/teensy_loader_cli/teensy_loader_cli
FORMAT             = ihex

# This is where we will store compiled libs and the final output.
export BUILD_ROOT   = $(shell pwd)
export OUTPUT_PATH  = $(BUILD_ROOT)/build

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
CXXFLAGS    += -DCONFIG_C3P_CBOR
CXXFLAGS    += -DCONFIG_C3P_IMG_SUPPORT
CXXFLAGS    += -DCONFIG_C3P_M2M_SUPPORT
CXXFLAGS    += -DCONFIG_C3P_I2CADAPTER_ENABLE_CONSOLE
CXXFLAGS    += -DCONFIG_C3P_CONSOLE_REBOOT_TOOL
CXXFLAGS    += -DCONFIG_C3P_CONSOLE_PFINFO_TOOL
CXXFLAGS    += -DCONFIG_C3P_CONSOLE_GPIO_TOOL


CFLAGS       = -Wall -nostdlib
CFLAGS      += -DF_CPU=$(CPU_SPEED)
CFLAGS      += -mcpu=$(MCU) -mthumb -D__IMXRT1062__
CFLAGS      += -ffunction-sections -fdata-sections
CFLAGS      += -Wno-error=narrowing
CFLAGS      += -mlittle-endian
CFLAGS      += -mfloat-abi=hard -mfpu=fpv5-d16
CFLAGS      += -DARDUINO=10607 -DTEENSYDUINO=159 -DARDUINO_TEENSY40
CFLAGS      += -DUSB_VID=null -DUSB_PID=null -DUSB_SERIAL -DLAYOUT_US_ENGLISH

LIBS         = -lmotherflux0r -lm -larm_cortexM7lfsp_math -lstdc++
LD_FILE      = $(TEENSY_CORE_PATH)/imxrt1062.ld

INCLUDES     = -iquote. -iquotesrc/
INCLUDES    += -I$(ARDUINO_LIBS)
INCLUDES    += -I$(TEENSY_CORE_PATH)

INCLUDES    += -I$(BUILD_ROOT)/src/
INCLUDES    += -I$(BUILD_ROOT)/lib/CppPotpourri/src
INCLUDES    += -I$(BUILD_ROOT)/lib/ManuvrDrivers/src
INCLUDES    += -I$(BUILD_ROOT)/lib/ManuvrArduino/src
INCLUDES    += -I$(BUILD_ROOT)/lib/teensy4_i2c/src
INCLUDES    += -I$(ARDUINO_LIBS)/Audio
INCLUDES    += -I$(ARDUINO_LIBS)/Audio/utility
INCLUDES    += -I$(ARDUINO_LIBS)/SD/src
INCLUDES    += -I$(ARDUINO_LIBS)/SdFat/src
INCLUDES    += -I$(ARDUINO_LIBS)/SPI
INCLUDES    += -I$(ARDUINO_LIBS)/SerialFlash
INCLUDES    += -I$(ARDUINO_LIBS)/EEPROM
INCLUDES    += -I$(ARDUINO_LIBS)/Time


###########################################################################
# Source file definitions...
###########################################################################
SOURCES_C    = src/ICM_20948_C.c

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
SOURCES_CPP += src/uApp/uAppMagnetometer.cpp


###########################################################################
# exports, consolidation....
###########################################################################
OBJS  = $(SOURCES_C:.c=.o) $(SOURCES_CPP:.cpp=.o)
DEP_FILES = $(SOURCES_C:.c=.d) $(SOURCES_CPP:.cpp=.d)

# Merge our choices and export them to the downstream Makefiles...
CFLAGS += $(OPTIMIZATION) $(INCLUDES)

export CFLAGS
export CXXFLAGS  += $(CFLAGS)


###########################################################################
# Rules for building the firmware follow...
###########################################################################

.PHONY: lib $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf

all: lib $(OUTPUT_PATH)/$(FIRMWARE_NAME).elf

%.o : %.cpp
	$(CXX) -std=$(CPP_STANDARD) $(OPTIMIZATION) $(CXXFLAGS) -c -o $@ $^

%.o : %.c
	$(CC) -std=$(C_STANDARD) $(OPTIMIZATION) $(CFLAGS) -c -o $@ $^

%.o : %.S
	$(CC) -std=$(C_STANDARD) $(OPTIMIZATION) -x assembler-with-cpp $(CFLAGS) -c -o $@ $^


lib: $(OBJS)
	mkdir -p $(OUTPUT_PATH)
	$(MAKE) -C lib

$(OUTPUT_PATH)/$(FIRMWARE_NAME).elf: lib
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
	$(MAKE) clean -C lib
