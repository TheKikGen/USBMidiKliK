#
#             LUFA Library
#     Copyright (C) Dean Camera, 2014.
#
#  dean [at] fourwalledcubicle [dot] com
#           www.lufa-lib.org
#
# --------------------------------------
#         LUFA Project Makefile.
# --------------------------------------

# Run "make help" for target help.

# For Arduino Uno
MCU          = atmega16u2

# For Arduino Pro Micro
#MCU          = atmega32u4

ARCH         = AVR8
BOARD        = UNO
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = USBMidiKliK_dual
SRC          = 	$(TARGET).cpp Descriptors.c midiXparser.cpp
SRC          += $(LUFA_SRC_USB)
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Device/CDCClassDevice.c
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Host/CDCClassHost.c
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Device/MIDIClassDevice.c
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Host/MIDIClassHost.c
SRC          += $(LUFA_PATH)/Drivers/Peripheral/AVR8/Serial_AVR8.c


LUFA_PATH    = ../../LUFA

CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/ -fpermissive

# From gcc 4.7
CC_FLAGS     += -std=gnu++11

# Before gcc 4.7
#CC_FLAGS     += -std=c++0x

# For Arduino Pro Micro
#CC_FLAGS     += -DSINGLE_BOOT_MODE

LD_FLAGS     =

# ARDUINO libraries

# ARDUINO_PATH = /C/Arduino
# ARDUINO_CORE = $(ARDUINO_PATH)/hardware/arduino/avr
# ARDUINO_INC += "-I$(ARDUINO_CORE)/cores/arduino"
#ARDUINO_INC += "-I$(ARDUINO_CORE)/libraries/SoftwareSerial/src"
# ARDUINO_INC += "-I$(ARDUINO_CORE)/libraries/EEPROM/src"
# ARDUINO_INC += "-I$(ARDUINO_PATH)/packages/HoodLoader2/hardware/avr/2.0.5/variants/HoodLoader2"

# CC_FLAGS     += $(ARDUINO_INC)
# CC_FLAGS     += -llib/arduino/libraries/SoftwareSerial/SoftwareSerial.cpp.o
# CC_FLAGS     += -llib/arduino/core/core.a
# CC_FLAGS     += -llib/arduino/core/abi.cpp.o
# CC_FLAGS     += -llib/arduino/core/CDC.cpp.o
# CC_FLAGS     += -llib/arduino/core/HardwareSerial.cpp.o
# CC_FLAGS     += -llib/arduino/core/HardwareSerial0.cpp.o
# CC_FLAGS     += -llib/arduino/core/HardwareSerial1.cpp.o
# CC_FLAGS     += -llib/arduino/core/HardwareSerial2.cpp.o
# CC_FLAGS     += -llib/arduino/core/HardwareSerial3.cpp.o
# CC_FLAGS     += -llib/arduino/core/hooks.c.o
# CC_FLAGS     += -llib/arduino/core/IPAddress.cpp.o
# CC_FLAGS     += -llib/arduino/core/main.cpp.o
# CC_FLAGS     += -llib/arduino/core/new.cpp.o
# CC_FLAGS     += -llib/arduino/core/PluggableUSB.cpp.o
# CC_FLAGS     += -llib/arduino/core/Print.cpp.o
# CC_FLAGS     += -llib/arduino/core/Stream.cpp.o
# CC_FLAGS     += -llib/arduino/core/Tone.cpp.o
# CC_FLAGS     += -llib/arduino/core/USBCore.cpp.o
# CC_FLAGS     += -llib/arduino/core/WInterrupts.c.o
# CC_FLAGS     += -llib/arduino/core/wiring.c.o
# CC_FLAGS     += -llib/arduino/core/wiring_analog.c.o
# CC_FLAGS     += -llib/arduino/core/wiring_digital.c.o
# CC_FLAGS     += -llib/arduino/core/wiring_pulse.c.o
# CC_FLAGS     += -llib/arduino/core/wiring_pulse.S.o
# CC_FLAGS     += -llib/arduino/core/wiring_shift.c.o
# CC_FLAGS     += -llib/arduino/core/WMath.cpp.o
# CC_FLAGS     += -llib/arduino/core/WString.cpp.o


# ARDUINO DEVICE
# Specify the Vendor ID, Product ID and device name.
# This is used by Descriptors.c
# GENUINE ARDUINO UNO V3

ARDUINO_DEVICE_VENDORID							= 0x2341
#ARDUINO_DEVICE_PRODUCTID 						= 0x0043
ARDUINO_DEVICE_PRODUCTID 						= 0x0001

ARDUINO_DEVICE_MANUFACTURER_STRING 	= "Arduino (www.arduino.cc)"
ARDUINO_DEVICE_PRODUCT_STRING 			= "Arduino Uno dual midi"
ARDUINO_DEVICE_PRODUCT_SERIAL 			= "55732323430351718180"

# MIDI DEVICE
# Specify the Vender ID, Product ID and device name.
# This is used by Descriptors.c

MIDI_DEVICE_VENDORID								= 0x2912
MIDI_DEVICE_PRODUCTID 							= 0x1967
MIDI_DEVICE_MANUFACTURER_STRING 		= "The KikGen MIDI factory"
# The MIDI_DEVICE_PRODUCT_STRING size must not be changed as it is stored in PROGMEM
# and can be changed dynamically later...
MIDI_DEVICE_PRODUCT_STRING 					= "USB MidiKliK $(BUILD_STRING)                       "

CC_FLAGS     += -DARDUINO_DEVICE_VENDORID=$(ARDUINO_DEVICE_VENDORID)
CC_FLAGS     += -DARDUINO_DEVICE_PRODUCTID=$(ARDUINO_DEVICE_PRODUCTID)
CC_FLAGS     += -DARDUINO_DEVICE_MANUFACTURER_STRING=$(ARDUINO_DEVICE_MANUFACTURER_STRING)
CC_FLAGS     += -DARDUINO_DEVICE_PRODUCT_STRING=$(ARDUINO_DEVICE_PRODUCT_STRING)
CC_FLAGS     += -DARDUINO_DEVICE_PRODUCT_SERIAL=$(ARDUINO_DEVICE_PRODUCT_SERIAL)
CC_FLAGS     += -DMIDI_DEVICE_VENDORID=$(MIDI_DEVICE_VENDORID)
CC_FLAGS     += -DMIDI_DEVICE_PRODUCTID=$(MIDI_DEVICE_PRODUCTID)
CC_FLAGS     += -DMIDI_DEVICE_MANUFACTURER_STRING=$(MIDI_DEVICE_MANUFACTURER_STRING)
CC_FLAGS     += -DMIDI_DEVICE_PRODUCT_STRING=$(MIDI_DEVICE_PRODUCT_STRING)
CC_FLAGS     += $(BUILD_CFLAGS)

BUILD_NUMBER_FILE=build.txt

buildinc:
		# Create an auto-incrementing build number.
		@if ! test -f $(BUILD_NUMBER_FILE); then echo 0 > $(BUILD_NUMBER_FILE); fi
		@echo $$(($$(cat $(BUILD_NUMBER_FILE)) + 1)) > $(BUILD_NUMBER_FILE)
		make all BUILD_CFLAGS="-DBUILD_NUMBER=$(shell cat $(BUILD_NUMBER_FILE) ) -DBUILD_DATE=$(shell date +'%Y.%m.%d-%H:%M:%S')"\
		BUILD_NUMBER=$(shell cat $(BUILD_NUMBER_FILE)) BUILD_DATE=$(shell date +'%Y.%m.%d-%H:%M:%S') \
		BUILD_STRING=Build-$(shell cat $(BUILD_NUMBER_FILE))

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_hid.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk
