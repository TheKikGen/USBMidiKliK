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
bool MIDIBootMode  = false;

// Midi serial parser
midiXparser midiSerial;

// Used to tag MIDI events when sending to serial if
// required. So it is possible to get CN
uint8_t UsbMIDITagPacket = 0x00 ;

// Sysex used to set some parameters of the interface.
// Be aware that the 0x77 manufacturer id is reserved in the MIDI standard (but not used)
// The second byte is usually an id number or a func code + the midi channel (any here)
// The Third is the product id
static  uint8_t sysExInternalHeader[] = { 0x77,0x77,0x77} ;
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

	CheckEEPROM();
	SetupHardware();

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);

	if (MIDIBootMode) ProcessMidiUsbMode(); // Inifinite loop

	else ProcessSerialUsbMode();

}
///////////////////////////////////////////////////////////////////////////////
// CHECK EEPROM
//----------------------------------------------------------------------------
// Retrieve global parameters from EEPROM, or Initalize it
//////////////////////////////////////////////////////////////////////////////
void CheckEEPROM() {

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

		EEPROM_Params.vendorID  = DEVICE_VENDORID_MIDI;
		EEPROM_Params.productID = DEVICE_PRODUCTID_MIDI;

		uint8_t maxSize = MIN ( sizeof(EEPROM_Params.midiProductStringDescriptor.UnicodeString), sizeof(_utf8(MIDI_DEVICE_PRODUCT_STRING)) );
		EEPROM_Params.midiProductStringDescriptor.header.Size = sizeof(USB_Descriptor_Header_t)  + maxSize - 2;
		EEPROM_Params.midiProductStringDescriptor.header.Type = DTYPE_String;
		memcpy( (void*)&EEPROM_Params.midiProductStringDescriptor.UnicodeString,(const void*)_utf8(MIDI_DEVICE_PRODUCT_STRING),maxSize );

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
void SetupHardware(void)
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
	#warning "DUAL BOOT MODE"

	MIDIBootMode  		= ( (PINB & 0x04) == 0 ) ? false : true;

#else
	#warning "SINGLE BOOT MODE"

	MIDIBootMode = true;
#endif

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

		for (;;)
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

	for (;;) {

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

// ----------------------------------------------------------------------------
// MIDI PARSER
// Check whether we've received any MIDI data from the USART, and if it's
// complete send it over USB. return true if a packet was sent
// ----------------------------------------------------------------------------
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
// Send a serial midi msg to the USB midi
///////////////////////////////////////////////////////////////////////////////
void sendMidiSerialMsgToUsb( midiXparser* serialMidiParser ) {

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

		MIDI_SendEventPacket(&MIDIEvent);

}
//////////////////////////////////////////////////////////////////////////////
// Scan and parse sysex flows
// ----------------------------------------------------------------------------
// We use the midiXparser 'on the fly' mode, allowing to tag bytes as "captured"
// when they belong to a midi SYSEX message, without storing them in a buffer.
// SYSEX Error (not correctly terminated by 0xF7 for example) are cleaned up,
// to restore a correct parsing state.
///////////////////////////////////////////////////////////////////////////////

void scanMidiSerialSysExToUsb( midiXparser* serialMidiParser ) {

  static  MIDI_EventPacket_t MIDIEvent = {
		.Event = 0, .Data1 = 0 , .Data2 = 0, .Data3 = 0
	};
  static uint8_t 	packetLen = 0 ;
	static unsigned sysExInternalMsgIdx = 0;
	static bool 		sysExInternalHeaderFound = false;

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
			MIDI_SendEventPacket(&MIDIEvent);
			packetLen = 0;
			memset(&MIDIEvent,0,4);

			// Internal SYSEX to manage ?
      if ( sysExInternalHeaderFound  && sysExInternalBuffer[0] > 0   ) {
        // Process internal Sysex
        ProcessSysExInternal();
      }
      sysExInternalHeaderFound = false;
      sysExInternalMsgIdx = 0;
      sysExInternalBuffer[0] = 0;
  }

  // Stop if not in sysexmode anymore here !
  // The SYSEX error could be caused by another SOX, or Midi status...,
  if ( ! serialMidiParser->isSysExMode() ) return;

  // Fill USB sysex packet
  packetLen++;
  ((uint8_t*)&MIDIEvent)[packetLen] = readByte ;

	// Is it an internal SYSEX message for the MIDI interface   ?

  if ( readByte == sysExInternalHeader[sysExInternalMsgIdx] || sysExInternalHeaderFound ) {
      if (sysExInternalHeaderFound) {
        // Start storing the message in the msg buffer
        // If Message too big. don't store...
        if ( sysExInternalBuffer[0] <  sizeof(sysExInternalBuffer)-1  ) {
            sysExInternalBuffer[0]++;
            sysExInternalBuffer[sysExInternalBuffer[0]]  = readByte;
        }

      }
      else {
        sysExInternalMsgIdx++;
        if (sysExInternalMsgIdx >= sizeof(sysExInternalHeader) ) {
            sysExInternalHeaderFound = true;
            sysExInternalBuffer[0] = 0;
        }
        else sysExInternalHeaderFound = false;
      }
  } else sysExInternalMsgIdx =0;

  // Packet complete ?
  if (packetLen == 3 ) {
      MIDIEvent.Event  =  4 ; // Sysex start or continue
			MIDI_SendEventPacket(&MIDIEvent);
			packetLen = 0;
    	memset(&MIDIEvent,0,4);
  }
}

// ----------------------------------------------------------------------------
// Send a MIDI Event packet to the USB
// ----------------------------------------------------------------------------

static void MIDI_SendEventPacket(const MIDI_EventPacket_t *MIDIEvent)
{
 	MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, MIDIEvent);
	MIDI_Device_Flush(&Keyboard_MIDI_Interface);

	LEDs_TurnOnLEDs(LEDS_LED2);
	rx_ticks = TICK_COUNT;
}

// ----------------------------------------------------------------------------
// Read a MIDI USB event and send it to the USART.
// ----------------------------------------------------------------------------
// USB MIDI will do all the parsing stuff for us.
// We just need to get the length of the MIDI message embedded in the packet
// ex : Note-on message on virtual cable 1 (CN=0x1; CIN=0x9) 	19 9n kk vv => 9n kk vv
// ex : tagged Note-on message on virtual cable 1 :  	19 9n kk vv => FD 19 9n kk vv
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
			uint8_t CIN = MIDIEvent.Event & 0x0F;
			switch (CIN) {
							// 1 byte
							case 0x05: case 0x0F:
								RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent.Data1 );
								break;

							// 2 bytes
							case 0x02: case 0x06: case 0x0C: case 0x0D:
								RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent.Data1 );
								RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent.Data2 );
								break;

							// 3 bytes
							case 0x03: case 0x07: case 0x04: case 0x08:
							case 0x09: case 0x0A: case 0x0B: case 0x0E:
								RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent.Data1 );
								RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent.Data2 );
								RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent.Data3 );
								break;
			}
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
// ----------------------------------------------------------------------------
// sysExInternalBuffer[0] length of the message (func code + data)
// sysExInternalBuffer[1] function code
// sysExInternalBuffer[2] data without EOX
///////////////////////////////////////////////////////////////////////////////
static void ProcessSysExInternal() {

	uint8_t msgLen = sysExInternalBuffer[0];
	uint8_t cmdId  = sysExInternalBuffer[1];
	uint8_t i,j;

	switch (cmdId) {

		// RESET USB MIDI INTERFACE
		// F0 77 77 77 0A F7
		case 0x0A:
			HardReset_AVR();
			break;

		// CHANGE MIDI PRODUCT STRING
		// F0 77 77 77 0B <character array> F7
		case 0x0B:
			// Copy the receive message to the Product String Descriptor
			// For MIDI protocol compatibility, and avoid a sysex encoding,
			// We only support non accentuated ASCII characters, below 128.

			if ( (msgLen-1) > MIDI_PRODUCT_STRING_SIZE  )	{
					// Error : Product name too long
					return;
			}

			memset( (void*)&EEPROM_Params.midiProductStringDescriptor.UnicodeString,0,
							 sizeof(EEPROM_Params.midiProductStringDescriptor.UnicodeString) );

			j=0;
			for ( i=2 ; i<= msgLen ; i++ ) {
					if ( sysExInternalBuffer[i] < 128 ) {
						EEPROM_Params.midiProductStringDescriptor.UnicodeString[j] = sysExInternalBuffer[i];
						j++;
					} else {
						// Error : UTF8 not supported
						return;
					}
			}

			// Fix the header size
			EEPROM_Params.midiProductStringDescriptor.header.Size = sizeof(USB_Descriptor_Header_t) + j*2;

			// Write the whole param struct
			eeprom_write_block((void*)&EEPROM_Params, (void*)0 , sizeof(EEPROM_Params));

 			break;

		// VENDOR ID & PRODUCT ID
		// F0 77 77 77 0C <n1 n2 n3 n4 = Vendor Id nibbles> <n1 n2 n3 n4 = Product Id nibbles> F7
		case 0x0C:
				// As MIDI data are 7 bits bytes, we must use a special encoding, to encode 8 bits values,
				// as light as possible. As we have here only 2 x 16 bits values to handle,
				// the encoding will consists in sending each nibble (4 bits) serialized in bytes.
				// For example sending VendorID and ProductID 0X8F12 0X9067 will be encoded as :
				//   0x08 0XF 0x1 0x2  0X9 0X0 0X6 0X7,  so the complete SYSEX message will be :
				// F0 77 77 77 0C 08 0F 01 02 09 00 06 07 F7

				if ( msgLen != 9 ) return;


				EEPROM_Params.vendorID = (sysExInternalBuffer[2] << 12) + (sysExInternalBuffer[3] << 8) +
				                               (sysExInternalBuffer[4] << 4) + sysExInternalBuffer[5] ;

				EEPROM_Params.productID= (sysExInternalBuffer[6] << 12) + (sysExInternalBuffer[7] << 8) +
				                               (sysExInternalBuffer[8] << 4) + sysExInternalBuffer[9] ;

 			  // Write the whole param struct
				eeprom_write_block((void*)&EEPROM_Params, (void*)0 , sizeof(EEPROM_Params));

				break;
	}

};



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
