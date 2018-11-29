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

#include "USBMidiKliK_dual.h"

uint16_t tx_ticks = 0;
uint16_t rx_ticks = 0;
const uint16_t TICK_COUNT = 3000;

// To manage the dual boot
bool    MIDIBootMode  = false;
uint8_t softBootMode  = bootModeSerial;

// Midi serial parser
midiXparser midiSerial;

// Sysex used to set some parameters of the interface.
// Be aware that the 0x77 manufacturer id is reserved in the MIDI standard (but not used)
// The second byte is usually an id number or a func code + the midi channel (any here)
// The Third is the product id
static  uint8_t sysExInternalHeader[] = { 0xF0,0x77,0x77,0x77} ;
static  uint8_t sysExInternalBuffer[SYSEX_INTERNAL_BUFF_SIZE] ;

// Ring Buffers

static RingBuffer_t USBtoUSART_Buffer; 															// Circular buffer to hold host data
static uint8_t      USBtoUSART_Buffer_Data[USB_TO_USART_BUFF_SIZE]; // USB to USART_Buffer
static RingBuffer_t USARTtoUSB_Buffer;															// Circular buffer to hold data from the serial port
static uint8_t      USARTtoUSB_Buffer_Data[USART_TO_USB_BUFF_SIZE]; // USART to USB_Buffer
volatile uint8_t    rxByte;										   										// Used in ISR

EEPROM_Params_t  EEPROM_Params;

USB_Descriptor_String_t * ProductStringMIDI;
extern USB_ClassInfo_MIDI_Device_t Keyboard_MIDI_Interface;
extern USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface;
extern USB_Descriptor_Device_t DeviceDescriptorMIDI;

// Reset macros
#define SoftReset_AVR() GO $0000;
#define HardReset_AVR() wdt_enable(WDTO_30MS); while(1) {}

///////////////////////////////////////////////////////////////////////////////
// MAIN START HERE
///////////////////////////////////////////////////////////////////////////////
int  main(void)
{

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	CheckEEPROM();
	SetupHardware();

	if (MIDIBootMode)
		ProcessMidiUsbMode(); // Infinite loop;

	else if  ( softBootMode == bootModeConfigMenu )
		ConfigRootMenu(); // Infinite loop;

	ProcessSerialUsbMode(); // Infinite loop

}
///////////////////////////////////////////////////////////////////////////////
// CHECK EEPROM
//----------------------------------------------------------------------------
// Retrieve global parameters from EEPROM, or Initalize it
//////////////////////////////////////////////////////////////////////////////
static void CheckEEPROM() {

	// Read the EEPROM parameters structure
	eeprom_read_block((void*)&EEPROM_Params, (const void*)0, sizeof(EEPROM_Params));

	// If the signature is not found, of not the same version, or new build, then initialize
	if (
				memcmp( (const void*)&EEPROM_Params.signature,(const void*)EE_SIGNATURE,sizeof(EEPROM_Params.signature) ) ||
			  EEPROM_Params.prmVer != EE_PRMVER ||
				EEPROM_Params.buildNumber != BUILD_NUMBER
		 )
	{
		memset( (void*)&EEPROM_Params,0,sizeof(EEPROM_Params) );

		memcpy( (void*)&EEPROM_Params.signature,(const void*)EE_SIGNATURE,sizeof(EEPROM_Params.signature) );

		EEPROM_Params.prmVer = EE_PRMVER;

		EEPROM_Params.buildNumber = BUILD_NUMBER;
		EEPROM_Params.nextBootMode = bootModeConfigMenu;

		EEPROM_Params.vendorID  = DEVICE_VENDORID_MIDI;
		EEPROM_Params.productID = DEVICE_PRODUCTID_MIDI;

		uint8_t maxSize = MIN ( sizeof(EEPROM_Params.midiProductStringDescriptor.UnicodeString), sizeof(_utf8(MIDI_DEVICE_PRODUCT_STRING)) );
		EEPROM_Params.midiProductStringDescriptor.header.Size = sizeof(USB_Descriptor_Header_t)  + maxSize - 2;
		EEPROM_Params.midiProductStringDescriptor.header.Type = DTYPE_String;
		memcpy( (void*)&EEPROM_Params.midiProductStringDescriptor.UnicodeString,(const void*)_utf8(MIDI_DEVICE_PRODUCT_STRING),maxSize );

		// Channel Midi IN MAPPING
		DefaultChannelMapping();

		//Write the whole param struct
		eeprom_write_block((void*)&EEPROM_Params, (void*)0 , sizeof(EEPROM_Params));
	}
	DeviceDescriptorMIDI.VendorID = EEPROM_Params.vendorID;
	DeviceDescriptorMIDI.ProductID = EEPROM_Params.productID;
	ProductStringMIDI = (USB_Descriptor_String_t *) &EEPROM_Params.midiProductStringDescriptor.header;

}

///////////////////////////////////////////////////////////////////////////////
// SETUP HARWARE WITH DUAL BOOT SERIAL / MIDI
//----------------------------------------------------------------------------
// WHEN PB2 IS GROUNDED during reset, this allows to reflash the MIDI application
// firmware with the standard Arduino IDE without reflashing the usbserial bootloader.
//
// Original inspiration from dualMocoLUFA Project by morecat_lab
//     http://morecatlab.akiba.coocan.jp/
//
// Note : When PB3 is grounded, this allows the No midi serial RX parsing
///////////////////////////////////////////////////////////////////////////////
static void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	// Disable watchdog if enabled by bootloader/fuses
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	// Disable clock division
	clock_prescale_set(clock_div_1);

#endif

	// Pins assignments :
	// PB1 = LED
	// PB2 = MIDI / Arduino SERIAL when grounded

  DDRB  = 0x02;		// SET PB1 as OUTPUT and PB2/PB3 as INPUT
  PORTB = 0x0C;		// PULL-UP PB2/PB3

#ifndef SINGLE_BOOT_MODE
	#warning "DUAL BOOT MODE - JUMPER & SOFT"
	MIDIBootMode  		= ( (PINB & 0x04) == 0 ) ? false : true;

#else
	#warning "SINGLE BOOT MODE - SOFT DUAL - NO JUMPER"
	MIDIBootMode = true;
#endif


// Debug
//MIDIBootMode = true;
//EEPROM_Params.nextBootMode = bootModeConfigMenu;
// End Debug

	if (MIDIBootMode) {
		// A previous SYSEX asked for a one shot serial boot mode
		if  ( EEPROM_Params.nextBootMode != bootModeMidi ) {
				MIDIBootMode = false;
				softBootMode = EEPROM_Params.nextBootMode;
				EEPROM_Params.nextBootMode = bootModeMidi;
				eeprom_write_block((void*)&EEPROM_Params, (void*)0 , sizeof(EEPROM_Params));
		}
	}

  if (MIDIBootMode) {
		Serial_Init(31250, false);
		midiSerial.setMidiChannelFilter(midiXparser::allChannel);
		midiSerial.setMidiMsgFilter( midiXparser::allMidiMsg );
		midiSerial.setSysExFilter(true,0); // Sysex on the fly
	}
	else
		Serial_Init(9600, false);

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();

	// Start the flush timer so that overflows occur rapidly to
	// push received bytes to the USB interface
	TCCR0B = (1 << CS02);

	/* Pull target /RESET line high */
	AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
	AVR_RESET_LINE_DDR  |= AVR_RESET_LINE_MASK;
}

///////////////////////////////////////////////////////////////////////////////
// INFINITE LOOP - CONFIG ROOT MENU
//----------------------------------------------------------------------------
// Allow USBMidLKLIK configuration by using a menu as a user interface
// from the USB serial port.
///////////////////////////////////////////////////////////////////////////////
static void ConfigRootMenu()
{
	char choice=0;
	char buff [32];
	uint8_t i,j;
	char c;

	GlobalInterruptEnable();

	for ( ;; )
		{
			USBSerialPutStr(PSTR("\n\nUSBMIDIKliK MENU\n"),true);
			USBSerialPutStr(PSTR("(c)TheKikGen Labs\n\n"),true);
			USBSerialPutStr(PSTR("0.Show current settings\n"),true);
			USBSerialPutStr(PSTR("1.Reload settings\n"),true);
			USBSerialPutStr(PSTR("2.Product string\n"),true);
			USBSerialPutStr(PSTR("3.VID - PID\n"),true);
			USBSerialPutStr(PSTR("4.Channel mapping\n"),true);
			USBSerialPutStr(PSTR("5.Default channel mapping\n"),true);
			USBSerialPutStr(PSTR("a.Arduino mode\n"),true);
			USBSerialPutStr(PSTR("s.Save & quit\n"),true);
			USBSerialPutStr(PSTR("x.Abort\n"),true);
			USBSerialPutStr(PSTR("=>"),true);

			// Wait for a character on USB CDC
			choice = USBSerialGetChar();
			USBSerialPutChar(choice);

			switch (choice)
			{
  			case '0':
					ShowCurrentSettings();
					break;

				case '1':
					// Read the EEPROM parameters structure
					eeprom_read_block((void*)&EEPROM_Params, (const void*)0, sizeof(EEPROM_Params));
					USBSerialPutStr(PSTR("\nSettings reloaded from EEPROM.\n"),true);
					break;

				case '2':
					USBSerialPutStr(PSTR("\nEnter product string - ENTER to terminate :\n"),true);
					i = 0;
					while ( i < MIDI_PRODUCT_STRING_SIZE && (c = USBSerialGetChar() ) !=13 ) {
						if ( c >= 32 && c < 127 ) {
							USBSerialPutChar(c);
							buff[i++]	= c;
						}
					}
					buff[i] = 0;
					SetProductString((char *)buff,i);
					break;

				case '3':
					USBSerialPutStr(PSTR("\nEnter VID - PID, in hex (0-9,a-f) :\n"),true);
					USBSerialScanHexChar( (char *) buff, 4, 0, 0);
					EEPROM_Params.vendorID  = GetInt16FromHex4Bin((char*)buff);
					USBSerialPutStr(PSTR("-"),true);
					USBSerialScanHexChar( (char * ) buff, 4, 0, 0);
					EEPROM_Params.productID = GetInt16FromHex4Bin((char*)buff);
					break;

				case '4':
					USBSerialPutStr(PSTR("\nMIDI IN (hex 0-f) : "),true);
					USBSerialScanHexChar( (char *) buff, 1, 0, 0);
					c = buff[0];
					EEPROM_Params.midiChannelMap[c] = 0L;
					USBSerialPutStr(PSTR(" ->OUT (n hex 0-f) : "),true);
					i = USBSerialScanHexChar( (char *) buff, 16, 13,',');
					if ( i == 0) {
							USBSerialPutStr(PSTR("Muted"),true);
					} else {
						for ( j = 0 ; j < i ; j++ ) {
							EEPROM_Params.midiChannelMap[c] += ( 1 << buff[j] );
							CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
							USB_USBTask();
						}
					}
					break;

				case '5':
						DefaultChannelMapping();
						USBSerialPutStr(PSTR("\nDefault mapping restored.\n"),true);
						break;

				case 's':
						ShowCurrentSettings();
						USBSerialPutStr(PSTR("\nWrite and exit (y/n) ?"),true);
						choice = USBSerialGetChar();USBSerialPutChar(choice);
						if ( choice == 'y') {
							eeprom_write_block((void*)&EEPROM_Params, (void*)0 , sizeof(EEPROM_Params));
							HardReset_AVR();
						}

						break;

				// Jump to Arduino Mode
				case 'a':
					USBSerialPutStr(PSTR("\nArduino mode. Bye.\n\n"),true);
					RebootSerialMode();
					break;

				case 'q':
					HardReset_AVR();
					break;
			}

		 CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		 USB_USBTask();
	 }
}


///////////////////////////////////////////////////////////////////////////////
// "Scanf like" for hexadecimal inputs
///////////////////////////////////////////////////////////////////////////////

static uint8_t USBSerialScanHexChar(char *buff, uint8_t len,char exitchar,char sepa) {

	uint8_t i = 0, c = 0;

	while ( i < len ) {
		c = USBSerialGetChar();
		if ( (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'  ) ) {
			USBSerialPutChar(c);
			if (sepa) USBSerialPutChar(sepa);
			buff[i++]	= GetInt8FromHexChar(c);
		} else if (c == exitchar && exitchar !=0 ) break;
	}
	return i;
}

///////////////////////////////////////////////////////////////////////////////
// USB serial putchar
///////////////////////////////////////////////////////////////////////////////
static void USBSerialPutChar(char c) {
		Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);
		while (!Endpoint_IsINReady()) {
			CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
			USB_USBTask();
		};
		CDC_Device_SendByte(&VirtualSerial_CDC_Interface,c );
}

///////////////////////////////////////////////////////////////////////////////
// USB serial putstr from PROGMEM when pgm is true
///////////////////////////////////////////////////////////////////////////////
static void USBSerialPutStr(char *s, bool pgm) {
		char c;
		while ( ( c = ( pgm ? pgm_read_byte(s) : *s ) ) != 0x00) {
			USBSerialPutChar(c);
      s++;
			CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
			USB_USBTask();
    }
}

///////////////////////////////////////////////////////////////////////////////
// USB serial putstrN from PROGMEM when pgm is true
// + Filter non printable chars. Used for UTF-8.
///////////////////////////////////////////////////////////////////////////////
static void USBSerialPutStrN(char *s, bool pgm,uint8_t n) {
		char c;
		while ( n ) {
		  c = pgm ? pgm_read_byte(s) : *s ;
			if (c >=32 && c<=127 ) 	USBSerialPutChar(c);
      s++; n--;
			CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
			USB_USBTask();
    }
}

///////////////////////////////////////////////////////////////////////////////
// USB serial getchar
///////////////////////////////////////////////////////////////////////////////
static char USBSerialGetChar() {
	int16_t ReceivedByte;
	// Wait for a character on USB CDC
	while ( (ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface) ) < 0  ) {
			CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
			USB_USBTask();
	};
	return (char) ReceivedByte;
}

///////////////////////////////////////////////////////////////////////////////
// Show current EEPROM settings
///////////////////////////////////////////////////////////////////////////////
static void ShowCurrentSettings() {
	char buff [8];
	uint8_t i,j;

	USBSerialPutStr(PSTR("\n\nMagic - V - Build :"),true);
	USBSerialPutStrN( (char *)&EEPROM_Params.signature , false,sizeof(EEPROM_Params.signature));
	USBSerialPutStr(PSTR("-"),true);
	USBSerialPutStr( itoa(EEPROM_Params.prmVer,buff,10), false);
	USBSerialPutStr(PSTR("-"),true);
	USBSerialPutStr( itoa(EEPROM_Params.buildNumber,buff,10) , false);
	USBSerialPutStr(PSTR("\nBootMode          :"),true);
	USBSerialPutStr( itoa(EEPROM_Params.nextBootMode,buff,10) , false);

	USBSerialPutStr(PSTR("\nVID - PID - STR   :"),true);
	USBSerialPutStr( itoa(EEPROM_Params.vendorID,buff,16), false);
	//sprintf(buff,"%X",EEPROM_Params.vendorID);USBSerialPutStr( buff , false);
	USBSerialPutStr(PSTR(" - "),true);
	USBSerialPutStr( itoa(EEPROM_Params.productID,buff,16), false);
	//sprintf(buff,"%X",EEPROM_Params.productID);USBSerialPutStr( buff , false);
	USBSerialPutStr(PSTR(" - "),true);
	USBSerialPutStrN((char*)&EEPROM_Params.midiProductStringDescriptor.UnicodeString, false,sizeof(EEPROM_Params.midiProductStringDescriptor.UnicodeString));

	USBSerialPutStr(PSTR("\nChannel map       :\n"),true);
	for ( i=0; i<= 15; i++) {
			USBSerialPutStr( itoa(i+1,buff,10), false);
			USBSerialPutStr(PSTR(" <-> "),true);
			if ( EEPROM_Params.midiChannelMap[i] == 0 ) USBSerialPutStr(PSTR("Muted") , true);
			else {
				for ( j=0 ; j<=15 ; j++) {
					 if ( EEPROM_Params.midiChannelMap[i] & ( 1 << j )  ) {
							USBSerialPutStr( itoa(j+1,buff,10), false);
							USBSerialPutStr(PSTR(","),true);
					 }
				 }
			}
			USBSerialPutStr(PSTR("\n"),true);
	}

}

///////////////////////////////////////////////////////////////////////////////
// USB Configuration and  events
///////////////////////////////////////////////////////////////////////////////

// Event handler for the library USB Connection event
void EVENT_USB_Device_Connect(void) {
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

// Event handler for the library USB Disconnection event
void EVENT_USB_Device_Disconnect(void) {
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

// Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
// of the USB device after enumeration - the device endpoints are configured and the MIDI management task started.
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	if (MIDIBootMode) {
		// Setup MIDI Data Endpoints
		ConfigSuccess &= MIDI_Device_ConfigureEndpoints(&Keyboard_MIDI_Interface);
	}
	else {
    ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
	}

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

// Event handler for the library USB Control Request event.
void EVENT_USB_Device_ControlRequest(void)
{
	if (MIDIBootMode)
	    MIDI_Device_ProcessControlRequest(&Keyboard_MIDI_Interface);
	else
	   CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

// Event handler for the CDC Class driver Host-to-Device Line Encoding Changed event.
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	bool CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);

	if (CurrentDTRState)
	  AVR_RESET_LINE_PORT &= ~AVR_RESET_LINE_MASK;
	else
	  AVR_RESET_LINE_PORT |= AVR_RESET_LINE_MASK;
}

// Event handler for the CDC Class driver Line Encoding Changed event.
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
		case CDC_PARITY_Odd:
			ConfigMask = ((1 << UPM11) | (1 << UPM10));
			break;
		case CDC_PARITY_Even:
			ConfigMask = (1 << UPM11);
			break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
		ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
		case 6:
			ConfigMask |= (1 << UCSZ10);
			break;
		case 7:
			ConfigMask |= (1 << UCSZ11);
			break;
		case 8:
			ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
			break;
	}

	// Must turn off USART before reconfiguring it, otherwise incorrect operation may occur
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	// Special case 57600 baud for compatibility with the ATmega328 bootloader.
	UBRR1  = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600)
			 ? SERIAL_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS)
			 : SERIAL_2X_UBBRVAL(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);

	UCSR1C = ConfigMask;
	UCSR1A = (CDCInterfaceInfo->State.LineEncoding.BaudRateBPS == 57600) ? 0 : (1 << U2X1);
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));
}

///////////////////////////////////////////////////////////////////////////////
// INFINITE LOOP FOR USB SERIAL PROCESSING
// ----------------------------------------------------------------------------
// Serial Worker Functions
// Used when Arduino is in USB Serial mode.
///////////////////////////////////////////////////////////////////////////////
static void ProcessSerialUsbMode(void) {

		RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
		RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));

		GlobalInterruptEnable();

		for ( ;; )
		{
		 // Only try to read bytes from the CDC interface if the transmit buffer is not full

		 if (!(RingBuffer_IsFull(&USBtoUSART_Buffer))) {

			 int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
			 // Store received byte into the USART transmit buffer
			 if (!(ReceivedByte < 0))
				 RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
		 }

		 uint16_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
		 if (BufferCount) {
			 Endpoint_SelectEndpoint(VirtualSerial_CDC_Interface.Config.DataINEndpoint.Address);

			 // Check if a packet is already enqueued to the host - if so, we shouldn't try to send more data
			// until it completes as there is a chance nothing is listening and a lengthy timeout could occur
			 if (Endpoint_IsINReady()) {
				 // Never send more than one bank size less one byte to the host at a time, so that we don't block
			// while a Zero Length Packet (ZLP) to terminate the transfer is sent if the host isn't listening
				 uint8_t BytesToSend = MIN(BufferCount, (CDC_TXRX_EPSIZE - 1));

				 // Read bytes from the USART receive buffer into the USB IN endpoint
				 while (BytesToSend--) {
					 // Try to send the next byte of data to the host, abort if there is an error without dequeuing
					 if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
											 RingBuffer_Peek(&USARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError) break;
					 // Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred
					 RingBuffer_Remove(&USARTtoUSB_Buffer);
				 }
			 }
		 }

		 // Load the next byte from the USART transmit buffer into the USART if transmit buffer space is available
		 if (Serial_IsSendReady() && !(RingBuffer_IsEmpty(&USBtoUSART_Buffer))  )
			 Serial_SendByte( RingBuffer_Remove(&USBtoUSART_Buffer) );

		 CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		 USB_USBTask();
	 }
}

///////////////////////////////////////////////////////////////////////////////
// INFINITE LOOP FOR USB MIDI AND SERIAL MIDI PROCESSING
// ----------------------------------------------------------------------------
// MIDI Worker Functions
// Used when Arduino is in USB MIDI mode (default mode).
///////////////////////////////////////////////////////////////////////////////
static void ProcessMidiUsbMode(void) {

	RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
	RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));

  // Enable USART receive complete interrupt, transmitter, receiver
	UCSR1B = 0;
	UCSR1B = ((1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1));

	PORTB  = 0x0E;	       /* PORTB1 = HIGH (LED ON) */

	GlobalInterruptEnable();

	for ( ;; ) {

		if (tx_ticks > 0) tx_ticks--;
		else if (tx_ticks == 0) LEDs_TurnOffLEDs(LEDS_LED1);

		if (rx_ticks > 0) rx_ticks--;
		else if (rx_ticks == 0) LEDs_TurnOffLEDs(LEDS_LED2);

		ProcessMidiToUsb();
		ProcessUsbToMidi();

		MIDI_Device_USBTask(&Keyboard_MIDI_Interface);
		USB_USBTask();
	}
}

///////////////////////////////////////////////////////////////////////////////
// PROCESS SERIAL MIDI TO USB
// ----------------------------------------------------------------------------
// Check whether we've received any MIDI data from the USART, and if it's
// complete send it over USB.
///////////////////////////////////////////////////////////////////////////////
static void ProcessMidiToUsb()
{

  if (USB_DeviceState != DEVICE_STATE_Configured) return ;

	// Get a byte from the ring buffer
  if ( RingBuffer_IsEmpty(&USARTtoUSB_Buffer) ) return ;

	uint8_t receivedByte = RingBuffer_Remove(&USARTtoUSB_Buffer) ;

	if ( midiSerial.parse( receivedByte)) {
						sendMidiSerialMsgToUsb( &midiSerial );
	}
	else
	// Check if a SYSEX msg is currently sent or terminated
	// as we proceed on the fly.
	if ( midiSerial.isByteCaptured() &&
				( midiSerial.isSysExMode() ||
					midiSerial.getByte() == midiXparser::eoxStatus ||
					midiSerial.isSysExError()  ) )
	{
					// Process for eventual SYSEX unbuffered on the fly
					scanMidiSerialSysExToUsb(&midiSerial) ;
	}

}

///////////////////////////////////////////////////////////////////////////////
// PROCESS USB MIDI TO USART.
// ----------------------------------------------------------------------------
// USB MIDI will do all the parsing stuff for us.
// We just need to get the length of the MIDI message embedded in the packet
// ex : Note-on message on virtual cable 1 (CN=0x1; CIN=0x9) 	19 9n kk vv => 9n kk vv
// ex : tagged Note-on message on virtual cable 1 :  	19 9n kk vv => FD 19 9n kk vv
///////////////////////////////////////////////////////////////////////////////
static void ProcessUsbToMidi(void)
{
	MIDI_EventPacket_t MIDIEvent;

	// Load the next byte from the USB/USART transmit buffer into the USART
	// if transmit buffer space is available

	if ( !(RingBuffer_IsEmpty(&USBtoUSART_Buffer)) && Serial_IsSendReady() ) {
		Serial_SendByte( RingBuffer_Remove(&USBtoUSART_Buffer) );
		LEDs_TurnOnLEDs(LEDS_LED1);
		tx_ticks = TICK_COUNT;
	}

	// Do not read the USB Midi command if not enough room in the USART buffer
	// 3 bytes for an USB Midi event
	if ( RingBuffer_GetFreeCount(&USBtoUSART_Buffer) < 4 ) return;

	/* Check if a MIDI command has been received */
	if (MIDI_Device_ReceiveEventPacket(&Keyboard_MIDI_Interface, &MIDIEvent)) {
				routePacketToTarget( FROM_USB,&MIDIEvent);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Send a serial midi msg to the USB midi
///////////////////////////////////////////////////////////////////////////////
static void sendMidiSerialMsgToUsb( midiXparser* serialMidiParser ) {

		MIDI_EventPacket_t MIDIEvent;

		uint8_t msgLen = serialMidiParser->getMidiMsgLen();
		uint8_t msgType = serialMidiParser->getMidiMsgType();

		memset(&MIDIEvent,0,4 );
		memcpy(&MIDIEvent.Data1,&(serialMidiParser->getMidiMsg()[0]),msgLen);

		// Real time single byte message CIN F->
		if ( msgType == midiXparser::realTimeMsgType ) MIDIEvent.Event = 0xF;
		else

		// Channel voice message CIN A-E
		if ( msgType == midiXparser::channelVoiceMsgType )
				MIDIEvent.Event  = ( (serialMidiParser->getMidiMsg()[0]) >> 4);

		else

		// System common message CIN 2-3
		if ( msgType == midiXparser::systemCommonMsgType ) {

				// 5 -  single-byte system common message (Tune request is the only case)
				if ( msgLen == 1 ) MIDIEvent.Event = 5;

				// 2/3 - two/three bytes system common message
				else MIDIEvent.Event = msgLen;
		}

		else return; // We should never be here !

		//MIDI_SendEventPacket(&MIDIEvent);
		routePacketToTarget( FROM_SERIAL,&MIDIEvent);
}
//////////////////////////////////////////////////////////////////////////////
// Scan and parse serial sysex flows
// ----------------------------------------------------------------------------
// We use the midiXparser 'on the fly' mode, allowing to tag bytes as "captured"
// when they belong to a midi SYSEX message, without storing them in a buffer.
// SYSEX Error (not correctly terminated by 0xF7 for example) are cleaned up,
// to restore a correct parsing state.
///////////////////////////////////////////////////////////////////////////////
static void scanMidiSerialSysExToUsb( midiXparser* serialMidiParser ) {

  static  MIDI_EventPacket_t MIDIEvent = {
		.Event = 0, .Data1 = 0 , .Data2 = 0, .Data3 = 0
	};
  static uint8_t 	packetLen = 0 ;

  byte readByte = serialMidiParser->getByte();

  // Normal End of SysEx or : End of SysEx with error.
  // Force clean end of SYSEX as the midi usb driver
  // will not understand if we send the packet as is
  if ( readByte == midiXparser::eoxStatus || serialMidiParser->isSysExError() ) {
      // Force the eox byte in case we have a SYSEX error.
			// In case of eror, the readbyte is already parsed by SerialMidiParser here,
			// so we don't loose it.

      packetLen++;
			((uint8_t*)&MIDIEvent)[packetLen] = midiXparser::eoxStatus;

      // CIN = 5/6/7  sysex ends with one/two/three bytes,
      MIDIEvent.Event =  (packetLen + 4) ;
			//MIDI_SendEventPacket(&MIDIEvent);
			routePacketToTarget( FROM_SERIAL,&MIDIEvent);
			packetLen = 0;
			memset(&MIDIEvent,0,4);
  }

  // Stop if not in sysexmode anymore here !
  // The SYSEX error could be caused by another SOX, or Midi status...,
  if ( ! serialMidiParser->isSysExMode() ) return;

  // Fill USB sysex packet
  packetLen++;
  ((uint8_t*)&MIDIEvent)[packetLen] = readByte ;

  // Packet complete ?
  if (packetLen == 3 ) {
      MIDIEvent.Event  =  4 ; // Sysex start or continue
			//MIDI_SendEventPacket(&MIDIEvent);
			routePacketToTarget( FROM_SERIAL,&MIDIEvent);
			packetLen = 0;
    	memset(&MIDIEvent,0,4);
  }
}


///////////////////////////////////////////////////////////////////////////////
// MIDI PACKET ROUTER
//-----------------------------------------------------------------------------
// Route a packet from one MIDI IN / USB OUT to n MIDI OUT/USB IN
///////////////////////////////////////////////////////////////////////////////

static void routePacketToTarget(uint8_t source,const MIDI_EventPacket_t *MIDIEvent) {

	uint8_t cin   = MIDIEvent->Event & 0x0F ;

	// Other channels messages
	if (source == FROM_SERIAL) {
		MIDI_SendEventPacket(MIDIEvent);
	} else

	if (source == FROM_USB) {
		sendMidiUsbPacketToSerial(MIDIEvent);
	}

	// Internal SYSEX
	if (cin >= 4 && cin <= 7 ) {
		 ParseSysExInternal(MIDIEvent);
	}

}
///////////////////////////////////////////////////////////////////////////////
// PARSE INTERNAL SYSEX, either for serial or USB
///////////////////////////////////////////////////////////////////////////////
static void ParseSysExInternal(const MIDI_EventPacket_t *MIDIEvent) {

		static unsigned sysExInternalMsgIdx = 0;
		static bool 	sysExInternalHeaderFound = false;

		uint8_t cin   = MIDIEvent->Event & 0x0F ;

		// Only SYSEX and concerned packet
		if (cin < 4 || cin > 7) return;
		if (cin == 4 && MIDIEvent->Data1 != 0xF0 && sysExInternalMsgIdx<3 ) return;
		if (cin > 4  && sysExInternalMsgIdx<3) return;

		uint8_t pklen = ( cin == 4 ? 3 : cin - 4) ;
		uint8_t EvData[3];
		uint8_t ev = 0;

		EvData[0]	= MIDIEvent->Data1;
		EvData[1]	= MIDIEvent->Data2;
		EvData[2]	= MIDIEvent->Data3;

		for ( uint8_t i = 0 ; i< pklen ; i++ ) {
			if (sysExInternalHeaderFound) {
				// Start storing the message in the msg buffer
				// If Message too big. don't store...
				if ( sysExInternalBuffer[0] <  sizeof(sysExInternalBuffer)-1  ) {
						if (EvData[ev] != 0xF7) {
							sysExInternalBuffer[0]++;
							sysExInternalBuffer[sysExInternalBuffer[0]]  = EvData[ev];
						}
						ev++;
				}
			}	else

			if ( sysExInternalHeader[sysExInternalMsgIdx] == EvData[ev] ) {
				sysExInternalMsgIdx++;
				ev++;
				if ( sysExInternalMsgIdx >= sizeof(sysExInternalHeader) ) {
					sysExInternalHeaderFound = true;
					sysExInternalBuffer[0] = 0; // Len of the sysex buffer
				}
			}

			else {
				// No match
				sysExInternalMsgIdx = 0;
				sysExInternalHeaderFound = false;
				return;
			}
		}

		// End of SYSEX for a valid message ? => Process
		if (cin != 4  && sysExInternalHeaderFound ) {
			sysExInternalMsgIdx = 0;
			sysExInternalHeaderFound = false;
			ProcessSysExInternal();
		}
}


///////////////////////////////////////////////////////////////////////////////
// Send a MIDI Event packet to the USB
///////////////////////////////////////////////////////////////////////////////
static void MIDI_SendEventPacket(const MIDI_EventPacket_t *MIDIEvent)
{
 	MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, MIDIEvent);
	MIDI_Device_Flush(&Keyboard_MIDI_Interface);

	LEDs_TurnOnLEDs(LEDS_LED2);
	rx_ticks = TICK_COUNT;
}

///////////////////////////////////////////////////////////////////////////////
// Send a USB midi packet to ad-hoc serial MIDI (via Ring buffer)
///////////////////////////////////////////////////////////////////////////////
static void sendMidiUsbPacketToSerial(const MIDI_EventPacket_t *MIDIEvent) {

	uint8_t CIN = MIDIEvent->Event & 0x0F;
	switch (CIN) {
					// 1 byte
					case 0x05: case 0x0F:
						RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent->Data1 );
						break;

					// 2 bytes
					case 0x02: case 0x06: case 0x0C: case 0x0D:
						RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent->Data1 );
						RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent->Data2 );
						break;

					// 3 bytes
					case 0x03: case 0x07: case 0x04: case 0x08:
					case 0x09: case 0x0A: case 0x0B: case 0x0E:
						RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent->Data1 );
						RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent->Data2 );
						RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent->Data3 );
						break;
	}
}

///////////////////////////////////////////////////////////////////////////////
// Process internal USBMidiKlik SYSEX
// ----------------------------------------------------------------------------
// MidiKlik SYSEX are of the following form :
//
// F0            SOX Start Of Sysex
// 77 77 77      USBMIDIKliK header
// <xx>          USBMIDIKliK sysex command
// <dddddd...dd> data
// F7            EOX End of SYSEX
//
// SOX, Header and EOX are not stored in sysExInternalBuffer.
//
// Cmd Description                           Data
//
// 0xA Hard reset interface
// 0xB Set a new product string		< product String 30 x7 bits char max >
// 0xC Set VendorID & ProductID   <n1n2n3n4=VendorId > <n1n2n3n4=ProductId>
// 0xD Midi IN channel mapping    <channel 1 to 16 > <Target channel 1,...16>
// ----------------------------------------------------------------------------
// sysExInternalBuffer[0] length of the message (func code + data)
// sysExInternalBuffer[1] function code
// sysExInternalBuffer[2] data without EOX
///////////////////////////////////////////////////////////////////////////////
static void ProcessSysExInternal() {

	uint8_t msgLen = sysExInternalBuffer[0];
	uint8_t cmdId  = sysExInternalBuffer[1];


	if ( msgLen == 0 ) return;

	switch (cmdId) {


		// TEMP REBOOT IN CONFIG MODE
		// F0 77 77 77 08 F7
		case 0x08:

			// Set serial boot mode & Write the whole param struct
			EEPROM_Params.nextBootMode = bootModeConfigMenu;
			eeprom_write_block((void*)&EEPROM_Params, (void*)0 , sizeof(EEPROM_Params));
			HardReset_AVR();
			break;

		// TEMP REBOOT IN ARDUINO MODE
		// F0 77 77 77 09 F7
		case 0x09:
			RebootSerialMode();
			break;

		// RESET USB MIDI INTERFACE
		// F0 77 77 77 0A F7
		case 0x0A:
			HardReset_AVR();
			break;

		// CHANGE MIDI PRODUCT STRING --------------------------------------------
		// F0 77 77 77 0B <character array> F7
		case 0x0B:
			// Copy the receive message to the Product String Descriptor
			// For MIDI protocol compatibility, and avoid a sysex encoding,
			// We only support non accentuated ASCII characters, below 128.

			if ( (msgLen-1) > MIDI_PRODUCT_STRING_SIZE  )	{
					// Error : Product name too long
					return;
			}

			// Write the product string in EEPROM struct
			// If ok, write the whole param struct to EEPROM
			if ( SetProductString((char*)&sysExInternalBuffer[2],msgLen-1) )
				eeprom_write_block((void*)&EEPROM_Params, (void*)0 , sizeof(EEPROM_Params));

 			break;

		// VENDOR ID & PRODUCT ID ------------------------------------------------
		// F0 77 77 77 0C <n1 n2 n3 n4 = Vendor Id nibbles> <n1 n2 n3 n4 = Product Id nibbles> F7
		case 0x0C:
				// As MIDI data are 7 bits bytes, we must use a special encoding, to encode 8 bits values,
				// as light as possible. As we have here only 2 x 16 bits values to handle,
				// the encoding will consists in sending each nibble (4 bits) serialized in bytes.
				// For example sending VendorID and ProductID 0X8F12 0X9067 will be encoded as :
				//   0x08 0XF 0x1 0x2  0X9 0X0 0X6 0X7,  so the complete SYSEX message will be :
				// F0 77 77 77 0C 08 0F 01 02 09 00 06 07 F7

				if ( msgLen != 9 ) return;

				EEPROM_Params.vendorID = GetInt16FromHex4Bin((char *) &sysExInternalBuffer[3]);
				EEPROM_Params.productID= GetInt16FromHex4Bin((char *) &sysExInternalBuffer[6]);
				eeprom_write_block((void*)&EEPROM_Params, (void*)0 , sizeof(EEPROM_Params));

				break;


		// MIDI CHANNEL MAPPING. Map a midi IN channel to n midi OUT channels -----
		// F0 77 77 77 0D <MIDI IN 1-16> <MIDI OUT 1, 2, 3...16 max> F7

		case 0x0D:

		// Targets midi OUT channels are passed as bytes from 1 to 16
		// (0x01 to 0x10), right after the MIDI IN byte.
		// You can pass a variable number of channels, but 16 as a maximum.
		// Command :
		// Passing 0x00 will "mute" the channel
		// Passing 0x7F will reset the mapping to default
		// Default is 1=>1, 2=2,....16=>16

		// For example, to map the midi channel 2 to channel 2,5 and 16,
		// map the channel 3 to channel 4, and mute the channel 16,
		// send the SYSEX MESSAGES :
		// F0 77 77 77 0D 02 02 05 10 F7
		// F0 77 77 77 0D 03 04 F7
		// F0 77 77 77 0D 10 00 F7

			if ( msgLen <3 || msgLen > 18 ) return;

			break;
	}

};

///////////////////////////////////////////////////////////////////////////////
// Write the Product String to the EEPROM struct
///////////////////////////////////////////////////////////////////////////////
static bool SetProductString(char *s,uint8_t len) {

		memset( (void*)&EEPROM_Params.midiProductStringDescriptor.UnicodeString,0,
					 sizeof(EEPROM_Params.midiProductStringDescriptor.UnicodeString) );

		uint8_t j=0;
		for ( uint8_t i=0 ; i < len ; i++ ) {
			 if ( s[i] < 128 ) {
					EEPROM_Params.midiProductStringDescriptor.UnicodeString[j] = s[i];
					j++;
				} else {
						// Error : UTF8 not supported
							return false;
				}
		}

		// Fix the header size
		EEPROM_Params.midiProductStringDescriptor.header.Size = sizeof(USB_Descriptor_Header_t) + j*2;
		return true;

}

///////////////////////////////////////////////////////////////////////////////
// Get an Int16 From a 4 Char hex array.  letters are minus.
// Warning : No control of the size or hex validity!!
///////////////////////////////////////////////////////////////////////////////
static uint16_t GetInt16FromHex4Char(char * buff) {
	char val[4];

	val[0] = GetInt8FromHexChar(buff[0]);
	val[1] = GetInt8FromHexChar(buff[1]);
	val[2] = GetInt8FromHexChar(buff[2]);
	val[3] = GetInt8FromHexChar(buff[3]);

	return GetInt16FromHex4Bin(val);
}

///////////////////////////////////////////////////////////////////////////////
// Get an Int8 From a hex char.  letters are minus.
// Warning : No control of the size or hex validity!!
///////////////////////////////////////////////////////////////////////////////
static uint8_t GetInt8FromHexChar(char c) {
	return c <= '9' ? c - '0' : c - 'a' + 0xa;
}

///////////////////////////////////////////////////////////////////////////////
// Get an Int16 From a 4 hex digit binary array.
// Warning : No control of the size or hex validity!!
///////////////////////////////////////////////////////////////////////////////
static uint16_t GetInt16FromHex4Bin(char * buff) {
	return (buff[0] << 12) + (buff[1] << 8) + (buff[2] << 4) + buff[3] ;
}

///////////////////////////////////////////////////////////////////////////////
// Reboot in Arduino serial mode
///////////////////////////////////////////////////////////////////////////////
static void RebootSerialMode() {

	// Set serial boot mode & Write the whole param struct
	EEPROM_Params.nextBootMode = bootModeSerial;
	eeprom_write_block((void*)&EEPROM_Params, (void*)0 , sizeof(EEPROM_Params));
	HardReset_AVR();
}

///////////////////////////////////////////////////////////////////////////////
// Default midi channel mapping
///////////////////////////////////////////////////////////////////////////////
static void DefaultChannelMapping() {
	EEPROM_Params.midiChannelMap[0]  =	0B0000000000000001;
	EEPROM_Params.midiChannelMap[1]  =	0B0000000000000010;
	EEPROM_Params.midiChannelMap[2]  =	0B0000000000000100;
	EEPROM_Params.midiChannelMap[3]  =	0B0000000000001000;
	EEPROM_Params.midiChannelMap[4]  =	0B0000000000010000;
	EEPROM_Params.midiChannelMap[5]  =	0B0000000000100000;
	EEPROM_Params.midiChannelMap[6]  =	0B0000000001000000;
	EEPROM_Params.midiChannelMap[7]  =	0B0000000010000000;
	EEPROM_Params.midiChannelMap[8]  =	0B0000000100000000;
	EEPROM_Params.midiChannelMap[9]  =	0B0000001000000000;
	EEPROM_Params.midiChannelMap[10] =	0B0000010000000000;
	EEPROM_Params.midiChannelMap[11] =	0B0000100000000000;
	EEPROM_Params.midiChannelMap[12] =	0B0001000000000000;
	EEPROM_Params.midiChannelMap[13] =	0B0010000000000000;
	EEPROM_Params.midiChannelMap[14] =	0B0100000000000000;
	EEPROM_Params.midiChannelMap[15] =	0B1000000000000000;
}

///////////////////////////////////////////////////////////////////////////////
// ISR to manage the reception of data from the midi/serial port, placing
// received bytes into a circular buffer for later transmission to the host.
///////////////////////////////////////////////////////////////////////////////
// Parse via Arduino/Serial
ISR(USART1_RX_vect, ISR_BLOCK)
{
	rxByte = UDR1;
	RingBuffer_Insert(&USARTtoUSB_Buffer, rxByte);
}
