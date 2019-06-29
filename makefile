#
#          USBMIDIKLIK (LUFA) Makefile
#     Copyright (C) TheKikGen Labs , 2018.
#
#

# Run "make help" for target help.


# COMMON DEFs ================================================================
SHELL=sh
# From gcc 4.7
CC_FLAGS     = -std=gnu++11
# Before gcc 4.7
#CC_FLAGS     += -std=c++0x
LD_FLAGS     =
SOURCE_FILE  = USBMidiKliK_dual
AVRDUDE_PATH = "/c/Program Files (x86)/Arduino/hardware/tools/avr/bin"

# ARDUINO MICRO / LEONARDO ==================================================
ifeq ($(TARGET_BOARD),micro)
TARGET_BOARD = micro
TARGET       = $(SOURCE_FILE)_$(TARGET_BOARD)
MCU          = atmega32u4
CC_FLAGS     += -DSINGLE_BOOT_MODE
ARCH         = AVR8
BOARD        = MICRO
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
FLASH_COMMAND_LINE = avrdude -v -patmega32u4 -cavr109 -PCOM$(COM) -b57600 -D -V -Uflash:w:./$(TARGET).hex.build:i

# Specify the Vendor ID, Product ID and device name.
# This is used by Descriptors.c
ARDUINO_DEVICE_VENDORID							= 0x2912
ARDUINO_DEVICE_PRODUCTID 						= 0x0001
ARDUINO_DEVICE_MANUFACTURER_STRING 	= "The KikGen Labs"
ARDUINO_DEVICE_PRODUCT_STRING 			= "Arduino Micro dual midi"
ARDUINO_DEVICE_PRODUCT_SERIAL 			= "55732323430351718180"

# UNO is the default ========================================================
else
TARGET_BOARD = uno
TARGET       = $(SOURCE_FILE)_$(TARGET_BOARD)
MCU          = atmega16u2
ARCH         = AVR8
BOARD        = UNO
F_CPU        = 16000000
F_USB        = $(F_CPU)
OPTIMIZATION = s
FLASH_COMMAND_LINE = avrdude -c usbasp -P usb -b 19200 -p m16u2  -U flash:w:./$(TARGET).hex.build:i

# Specify the Vendor ID, Product ID and device name.
# This is used by Descriptors.c
ARDUINO_DEVICE_VENDORID							= 0x2341
ARDUINO_DEVICE_PRODUCTID 						= 0x0001
ARDUINO_DEVICE_MANUFACTURER_STRING 	= "Arduino (www.arduino.cc)"
ARDUINO_DEVICE_PRODUCT_STRING 			= "Arduino Uno dual midi"
ARDUINO_DEVICE_PRODUCT_SERIAL 			= "55732323430351718180"
endif

# END OF BOARD SPECIFIC DEFs ================================================

# MIDI DEVICE
# Specify the Vender ID, Product ID and device name.
# This is used by Descriptors.c

MIDI_DEVICE_VENDORID								= 0x2912
MIDI_DEVICE_PRODUCTID 							= 0x1967
MIDI_DEVICE_MANUFACTURER_STRING 		= "The KikGen Labs"

# The MIDI_DEVICE_PRODUCT_STRING size must not be changed as it is used to set
# the max size in the USB header PROGMEM structure, and can be changed dynamically later...
MIDI_DEVICE_PRODUCT_STRING 					= "USB MidiKliK $(BUILD_STRING)                      "

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
BUILD_NUMBER=$(shell cat $(BUILD_NUMBER_FILE))
BUILD_DATE=$(shell date +'%Y.%m.%d-%H:%M:%S')
BUILD_STRING=Build-$(shell cat $(BUILD_NUMBER_FILE))
CC_FLAGS     += -DBUILD_NUMBER=$(shell cat $(BUILD_NUMBER_FILE) ) -DBUILD_DATE=$(shell date +'%Y.%m.%d-%H:%M:%S')


DEP_DIR      = dependencies
SRC          = 	$(SOURCE_FILE).cpp Descriptors.c $(DEP_DIR)/midiXparser/midiXparser.cpp

# LUFA DEFs ================================================================

SRC          += $(LUFA_SRC_USB)
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Device/CDCClassDevice.c
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Host/CDCClassHost.c
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Device/MIDIClassDevice.c
SRC          += $(LUFA_PATH)/Drivers/USB/Class/Host/MIDIClassHost.c
SRC          += $(LUFA_PATH)/Drivers/Peripheral/AVR8/Serial_AVR8.c

LUFA_PATH    = $(DEP_DIR)/lufa/LUFA
CC_FLAGS     += -DUSE_LUFA_CONFIG_HEADER -IConfig/ -fpermissive -Os



#=============================================================================

# Default target
uno:
		@make build_midiklik TARGET_BOARD=uno

micro:
		@make build_midiklik TARGET_BOARD=micro

uno_flash:
		@make uno
		@make flash_uno

micro_flash:
		@if make micro
		@make flash_micro COM=$(COM)

all_target:
		make purge
		make clean
		make uno
		make clean
		make micro
		make clean
		make purge

build_midiklik:
		@make buildinc
		@echo ======================================================================
		@echo ======== BUILD USBMIDIKLIK - TARGET=$(TARGET_BOARD) - BUILD NO=$(shell cat $(BUILD_NUMBER_FILE)) ========
		@echo ======================================================================
		@make all TARGET_BOARD=$(TARGET_BOARD)
		@make save_bin TARGET_BOARD=$(TARGET_BOARD)

save_bin:
		@if test -f $(TARGET).hex; then cp -f $(TARGET).hex $(TARGET).hex.build; fi

flash:
		echo $(FLASH_COMMAND_LINE)
		$(FLASH_COMMAND_LINE)

flash_uno:
		make flash TARGET_BOARD=uno

flash_micro:
		make flash COM=$(COM) TARGET_BOARD=micro

purge:
		@rm -rf *.elf *.bin *.eep *.hex *.lss *.map *.sym

buildinc:
		# Create an auto-incrementing build number.
		@if ! test -f $(BUILD_NUMBER_FILE); then echo 0 > $(BUILD_NUMBER_FILE); fi
		@echo $$(($$(cat $(BUILD_NUMBER_FILE)) + 1)) > $(BUILD_NUMBER_FILE)

#		make all BUILD_CFLAGS="-DBUILD_NUMBER=$(shell cat $(BUILD_NUMBER_FILE) ) -DBUILD_DATE=$(shell date +'%Y.%m.%d-%H:%M:%S')"\
	#	BUILD_NUMBER=$(shell cat $(BUILD_NUMBER_FILE)) BUILD_DATE=$(shell date +'%Y.%m.%d-%H:%M:%S') \
	#	BUILD_STRING=Build-$(shell cat $(BUILD_NUMBER_FILE))

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
