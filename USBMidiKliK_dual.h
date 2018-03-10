/***********************************************************************
 *  arduino_midi firmware, 02.01.2015
 *  by Dimitri Diakopoulos (http://www.dimitridiakopoulos.com)
 *  Based on the LUFA low-level midi example by Dean Camera
 *  (http://www.fourwalledcubicle.com/LUFA.php)
 *  Compiled against LUFA-140928
 ***********************************************************************/

#ifndef _usbmidiklik_h
#define _usbmidiklik_h


	#define USB_TO_USART_BUFF_SIZE 32
	#define USART_TO_USB_BUFF_SIZE 16
	#define SYSEX_INTERNAL_BUFF_SIZE MIDI_PRODUCT_STRING_SIZE + 2


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
	#include "EEPROM_Params.h"


	#define LEDMASK_USB_NOTREADY      LEDS_LED1
	#define LEDMASK_USB_ENUMERATING   LEDS_LED2
	#define LEDMASK_USB_READY         LEDS_LED2
	#define LEDMASK_USB_ERROR         LEDS_LED1

	/* Function Prototypes: */
	void CheckEEPROM(void);
	void SetupHardware(void);
	void EVENT_USB_Device_Connect(void);
	void EVENT_USB_Device_Disconnect(void);
	void EVENT_USB_Device_ConfigurationChanged(void);
	void EVENT_USB_Device_ControlRequest(void);
	void EVENT_CDC_Device_ControLineStateChanged(const USB_ClassInfo_CDC_Device_t* );
	void EVENT_CDC_Device_LineEncodingChanged(const USB_ClassInfo_CDC_Device_t* );

	static void ProcessSerialUsbMode(void);
	static void ProcessMidiUsbMode(void);
	static bool ProcessMidiToUsb(void);
	static void MIDI_SendEventPacket(const MIDI_EventPacket_t *,uint8_t);
	static void ProcessUsbToMidi(void);
	static void ProcessSysExInternal(void);

#endif
