/***********************************************************************
 *  arduino_midi firmware, 02.01.2015
 *  by Dimitri Diakopoulos (http://www.dimitridiakopoulos.com)
 *  Based on the LUFA low-level midi example by Dean Camera
 *  (http://www.fourwalledcubicle.com/LUFA.php)
 *  Compiled against LUFA-140928
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
	#include <LUFA/Drivers/Misc/RingBuffer.h>
	#include <LUFA/Drivers/USB/Class/CDCClass.h>

	#include "midiXparser.h"
	#include "EEPROM_Params.h"

	// Number of MIDI jacks (also number of cables by construction)
	#define SERIAL_INTERFACE_MAX 1

	// MIDI Routing
	#define FROM_SERIAL 0
	#define FROM_USB    1

	// Use this 32 bits structure to send and receive packet to/from USB
	// This is not the standard LUFA midi packet but we use this one
	// for compatibility resasons on other platforms
	union EVENT_t {
	    uint32_t i;
	    uint8_t  packet[4];
	};
	#define midiPacket_t union EVENT_t


	#define USB_TO_USART_BUFF_SIZE 32
	#define USART_TO_USB_BUFF_SIZE 16
	#define SYSEX_INTERNAL_BUFF_SIZE MIDI_PRODUCT_STRING_SIZE + 2

	#define LEDMASK_USB_NOTREADY      LEDS_LED1
	#define LEDMASK_USB_ENUMERATING   LEDS_LED2
	#define LEDMASK_USB_READY         LEDS_LED2
	#define LEDMASK_USB_ERROR         LEDS_LED1

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
	static void SendMidiSerialMsgToUsb( uint8_t , midiXparser*);
	static void PrepareSysExPacket( uint8_t , midiXparser* );
	static void ParseSysExInternal(const midiPacket_t ) ;
	static void RoutePacketToTarget(uint8_t , const midiPacket_t *);
	static void MidiUSBWritePacket(const midiPacket_t *);
	static void SerialWritePacket(const midiPacket_t *);
	static void ProcessSysExInternal(void);
	static bool SetProductString(char *,uint8_t );
	static uint16_t GetInt16FromHex4Char(char *);
	static uint8_t GetInt8FromHexChar(char);
	static uint16_t GetInt16FromHex4Bin(char *);
	static void RebootSerialMode();
	static void DefaultChannelMapping();

#endif
