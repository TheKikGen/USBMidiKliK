#pragma once

// EEPROM parameters
// The signature is used to check if EEPROM is correctly initialized
// The parameters version is the version of the current structure.
// Changing the version will cause a new initialization in CheckEEPROM();.
// The following structure start at the first address of the EEPROM

#define EE_SIGNATURE "MDK"
#define EE_PRMVER 2
#define DEVICE_VENDORID_MIDI  MIDI_DEVICE_VENDORID
#define DEVICE_PRODUCTID_MIDI MIDI_DEVICE_PRODUCTID
#define MIDI_PRODUCT_STRING_SIZE 30

// Boot modes
enum nextBootMode {
    bootModeMidi   = 0,
    bootModeSerial = 1,
    bootModeConfigMenu = 2,
};

typedef struct {
        uint8_t  signature[3];
        uint8_t  prmVer;
        uint16_t buildNumber;
        uint8_t  nextBootMode;
        uint16_t vendorID;
        uint16_t productID;
        struct   {
                  USB_Descriptor_Header_t header;
                  uint16_t 	UnicodeString [MIDI_PRODUCT_STRING_SIZE+1];
                } midiProductStringDescriptor ;
        uint16_t midiChannelMap[16];
} EEPROM_Params_t;
