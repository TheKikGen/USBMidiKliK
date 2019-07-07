/***********************************************************************
 *  ARDUINO_MIDI_DUAL FIRMWARE -
 *  Franck Touanen - 2017.01.16
 *
 *  Based on :
 *     . The wonderful LUFA low-level midi lib and examples by Dean Camera
 *               (http://www.fourwalledcubicle.com/LUFA.php)
 *     . Inspired by HIDUINO from Dimitri Diakopoulos
 *               (http://www.dimitridiakopoulos.com)
 *     . Inspired also by dualMocoLUFA Project from morecat_lab
 *               (http://morecatlab.akiba.coocan.jp/)
  *
 *  Compiled against the last LUFA version.

        USB                        ATMEGA8U2                    ATMEGA 328P
   --------------      ------------------------------         ---------------
   IN Endpoint  o<-----o USBOUT | usbMidiKliK |  RX o<--------o (TX) MIDI IN
   OUT Endpoint o----->o USBIN  |  firmware   |  TX o-------->o (RX) MIDI OUT
 ***********************************************************************/

#ifndef _usbmidiklik_h
#define _usbmidiklik_h

	//#define BOARD BOARD_UNO

	#include <avr/io.h>
	#include <avr/wdt.h>
	#include <avr/boot.h>
	#include <avr/eeprom.h>
	#include <avr/power.h>
	#include <avr/interrupt.h>
	#include <util/atomic.h>
	#include <stdbool.h>

	#include "Descriptors.h"
	#include "LUFA/Common/Common.h"
	#include <LUFA/Drivers/Peripheral/Serial.h>
	#include <LUFA/Drivers/USB/USB.h>
	#include <LUFA/Platform/Platform.h>
	#include <LUFA/Drivers/Board/LEDs.h>
	#include <LUFA/Drivers/Board/Board.h>
	//#include <LUFA/Drivers/Misc/RingBuffer.h>
	#include <LUFA/Drivers/USB/Class/CDCClass.h>
  #include "LightweightRingBuff.h"
	#include <midiXparser.h>
	#include "EEPROM_Params.h"

	// Number of MIDI jacks (also number of cables by construction)
	#define SERIAL_INTERFACE_MAX 1

	// MIDI Routing
	#define FROM_SERIAL 0
	#define FROM_USB    1

	// Use this 32 bits structure to send and receive packet to/from USB
	// This is not the standard LUFA midi packet but we use this one
	// for compatibility reasons on other platforms
	union midiPacket_t {
	    uint32_t i;
	    uint8_t  packet[4];
	};

	#define USB_TO_USART_BUFF_SIZE 12
	#define USART_TO_USB_BUFF_SIZE 30

	#define SYSEX_INTERNAL_HEADER 0xF0,0x77,0x77,0x77
	#define SYSEX_INTERNAL_BUFF_SIZE MIDI_PRODUCT_STRING_SIZE + 2

	#define LEDMASK_USB_NOTREADY      LEDS_LED1
	#define LEDMASK_USB_ENUMERATING   LEDS_LED2
	#define LEDMASK_USB_READY         LEDS_LED2
	#define LEDMASK_USB_ERROR         LEDS_LED1
	#define TICK_COUNT 3000
	// Reset macros
	#define SoftReset_AVR() GO $0000;
	#define HardReset_AVR() wdt_enable(WDTO_30MS); while(1) {}

	/* Function Prototypes: */
	static void CheckEEPROM(void);
	static void SetupHardware(void);
	static void ConfigRootMenu();
	static uint8_t USBSerialScanHexChar(char *, uint8_t ,char,char);
	static void USBSerialPutChar(char );
	static void USBSerialPutStr(char *,bool);
	static void USBSerialPutStrN(char *, bool ,uint8_t );
	static char USBSerialGetChar();
	static void ShowCurrentSettings();

	void EVENT_USB_Device_Connect(void);
	void EVENT_USB_Device_Disconnect(void);
	void EVENT_USB_Device_ConfigurationChanged(void);
	void EVENT_USB_Device_ControlRequest(void);
	void EVENT_CDC_Device_ControLineStateChanged(const USB_ClassInfo_CDC_Device_t* );
	void EVENT_CDC_Device_LineEncodingChanged(const USB_ClassInfo_CDC_Device_t* );

	static void ProcessSerialUsbMode(void);
	static void ProcessMidiUsbMode(void);
	static void ProcessMidiToUsb(void);
	static void ProcessUsbToMidi(void);
	static void SerialTask(void);
	static void Serial_Insert(byte);
	static void RouteStdMidiMsg( uint8_t , midiXparser*);
	static void RouteSysExMidiMsg( uint8_t , midiXparser* );
	static void ParseSysExInternal(const midiPacket_t *);
	static void RoutePacketToTarget(uint8_t , const midiPacket_t *);
	static void MidiUSBWritePacket(const midiPacket_t *);
	static void SerialWritePacket(const midiPacket_t *);
	static void ProcessSysExInternal(void);
	static bool SetProductString(char *,uint8_t );
	static uint16_t GetInt16FromHex4Char(char *);
	static uint8_t GetInt8FromHexChar(char);
	static uint16_t GetInt16FromHex4Bin(char *);
	static void BootloaderMode();
	static void DefaultChannelMapping();

#endif
