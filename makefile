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

MCU          = atmega16u2
ARCH         = AVR8
BOARD        = UNO
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
TARGET       = arduino_midi_dual
SRC          = 	$(TARGET).cpp Descriptors.c
SRC          += $(LUFA_SRC_USB)
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Device/CDCClassDevice.c
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Host/CDCClassHost.c
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Device/MIDIClassDevice.c
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Host/MIDIClassHost.c

#SRC          += $(ARDUINO_MIDILIB)/MIDI.cpp

LUFA_PATH    = ../../LUFA
#ARDUINO_MIDILIB = ./MIDI/src
CC_FLAGS     = -DUSE_LUFA_CONFIG_HEADER -IConfig/
LD_FLAGS     =

# ARDUINO DEVICE
# Specify the Vender ID, Product ID and device name.
# This is used by Descriptors.c
# GENUINE ARDUINO UNO V3

ARDUINO_DEVICE_VENDORID							= 0x2A03
ARDUINO_DEVICE_PRODUCTID 						= 0x0043
ARDUINO_DEVICE_MANUFACTURER_STRING 	= "Arduino Srl (www.arduino.org)"
ARDUINO_DEVICE_PRODUCT_STRING 			= "Arduino Uno"
ARDUINO_DEVICE_PRODUCT_SERIAL 			= "854393131303513111B1"

# MIDI DEVICE
# Specify the Vender ID, Product ID and device name.
# This is used by Descriptors.c

MIDI_DEVICE_VENDORID								= 0x2912
MIDI_DEVICE_PRODUCTID 							= 0x1968
MIDI_DEVICE_MANUFACTURER_STRING 		= "KikGen MIDI factory"
MIDI_DEVICE_PRODUCT_STRING 					= "KikGen USB-MIDI BETA1 $(BUILD_STRING)"

CC_FLAGS     += -DARDUINO_DEVICE_VENDORID=$(ARDUINO_DEVICE_VENDORID)
CC_FLAGS     += -DARDUINO_DEVICE_PRODUCTID=$(ARDUINO_DEVICE_PRODUCTID)
CC_FLAGS     += -DARDUINO_DEVICE_MANUFACTURER_STRING=$(ARDUINO_DEVICE_MANUFACTURER_STRING )
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
		make all BUILD_CFLAGS="-DBUILD_NUMBER=$(shell cat $(BUILD_NUMBER_FILE) )-DBUILD_DATE=$(shell date +'%Y.%m.%d-%H:%M:%S')"\
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
