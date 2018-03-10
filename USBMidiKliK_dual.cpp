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

bool MIDIBootMode  = false;
bool MIDISerialRxParse = true ; 	  // Use 1 byte packet to USB without parsing if false
uint8_t UsbMIDITagPacket = 0x00 ;  	// Used to tag MIDI events when sending to serial if
																		// required. So it is possible to get CN

// Use to set some parameters of the interface.
// Be aware that the 0x77 manufacturer id is reserved in the MIDI standard (but not used)
// The second byte is usually an id number or a func code + the midi channel (any here)
// The Third the product id
static  uint8_t sysExInternalHeader[] = { 0x77,0x77,0x77} ;
static  uint8_t sysExInternalBuffer[SYSEX_INTERNAL_BUFF_SIZE] ;

// Ring Buffers

static RingBuffer_t USBtoUSART_Buffer;           										// Circular buffer to hold host data
static uint8_t      USBtoUSART_Buffer_Data[USB_TO_USART_BUFF_SIZE]; // USB to USART_Buffer
static RingBuffer_t USARTtoUSB_Buffer;           										// Circular buffer to hold data from the serial port
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

///////////////////////////////////////////////////////////////////////////////
}
///////////////////////////////////////////////////////////////////////////////
// CHECK EEPROM
//
// Retrieve global parameters from EEPROM, or Initalize
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
//
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
	// PB3 = No RX serial parsing when grounded

  DDRB  = 0x02;		// SET PB1 as OUTPUT and PB2/PB3 as INPUT
  PORTB = 0x0C;		// PULL-UP PB2/PB3

	MIDIBootMode  		= ( (PINB & 0x04) == 0 ) ? false : true;
	MIDISerialRxParse = ( (PINB & 0x08) == 0 ) ? false : true;

  if (MIDIBootMode)
		Serial_Init(31250, false);
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
// Serial Worker Functions
// Used when Arduino is in USB Serial mode.
///////////////////////////////////////////////////////////////////////////////

// ----------------------------------------------------------------------------
// INFINITE LOOP FOR USB SERIAL PROCESSING
// ----------------------------------------------------------------------------
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
// MIDI Worker Functions
// Used when Arduino is in USB MIDI mode (default mode).
///////////////////////////////////////////////////////////////////////////////

// ----------------------------------------------------------------------------
// INFINITE LOOP FOR USB MIDI AND SERIAL MIDI PROCESSING
// ----------------------------------------------------------------------------
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
// MIDI USB  - Table 4-1: Code IndexNumber Classifications of MIDI10.PDF
// ----------------------------------------------------------------------------
/*
 The first byte in each 32-bit USB-MIDI Event Packet is a Packet Header
 contains a Cable Number (4 bits) followed by a Code Index Number (4 bits).
 The remaining threebytes contain the actual MIDI event.
 Most typical parsed MIDI events are two or threebytes in length.

 The Cable Number (C N) is a value ranging from 0x0 to 0xF indicating the number
 assignment of the Embedded MIDI Jack associated with the endpoint that is
 transferring the data.
 The Code Index Number (CIN) indicates the classification of the bytes in
 the MIDI_x fields. The following table summarizes these classifications.

Note-on message on virtual cable 1 (CN=0x1; CIN=0x9) 		9n kk vv 19 9n kk vv
Control change message on cable 10 (CN=0xA; CIN=0xB) 		Bn pp vv AB Bn pp vv
Real-time message F8 on cable 3 (CN=0x3; CIN=0xF) 			F8 xx xx 3F F8 xx xx

(Table 4-1: Code IndexNumber Classifications of MIDI10.PDF)                */
static const int BytesIn_USB_MIDI_Command[] =
{
	 0,	/* 0 - function codes (reserved) */
	 0,	/* 1 - cable events (reserved) */
	 2,	/* 2 - two-byte system common message */
	 3,	/* 3 - three-byte system common message */
	 3,	/* 4 - sysex starts or continues */
	 1,	/* 5 - sysex ends with one byte, or single-byte system common message */
	 2,	/* 6 - sysex ends with two bytes */
	 3,	/* 7 - sysex ends with three bytes */
	 3,	/* 8 - note-off */
	 3,	/* 9 - note-on */
	 3,	/* A - poly-keypress */
	 3,	/* B - control change */
	 2,	/* C - program change */
	 2,	/* D - channel pressure */
	 3,	/* E - pitch bend change */
	 1,	/* F - single byte */
 };

// ----------------------------------------------------------------------------
// MIDI PARSER
// Check whether we've received any MIDI data from the USART, and if it's
// complete send it over USB. return true if a packet was sent
// ----------------------------------------------------------------------------
static bool ProcessMidiToUsb()
{
  static  MIDI_EventPacket_t MIDIEvent;
  MIDI_EventPacket_t         xMIDIEvent;  // Event to process specific cases like RT

  static  bool sysExMode               = false;	 // True if SYSEX active
	static  bool sysExInternal 				   = false;  // True if internal SYSEX

	static  uint8_t sysExInternalMsgIdx = 0 ;			// internal Sysex msg index
  static  uint8_t dataBufferIndex   	= 0;			// Index on Data1 to Data3
  static  uint8_t nextMidiMsgLength 	= 0;			// Len of the midi message to be processed
  static  uint8_t runningStatus 			= 0;			// Used for running status


  if (USB_DeviceState != DEVICE_STATE_Configured) return false;

	// Get a byte from the ring buffer
  if ( RingBuffer_IsEmpty(&USARTtoUSB_Buffer) ) return false;

	uint8_t receivedByte = RingBuffer_Remove(&USARTtoUSB_Buffer) ;

// Does the parsing is required ?
// As explained in section 4, page 17, note 2, of MIDI10 (Universal Serial
// Bus Device Class Definition - Release 1.0 Nov1, 1999), it is possible to
// send packets without parsing anything. This is perfectly MIDI compliant :
// "CIN=0xF, Single Byte: in some special cases, an application may prefer not
// to use parsed MIDI events. Using CIN=0xF, a MIDI data stream may be
// transferred by placing each individual byte in one 32 Bit USB-MIDI Event
// Packet. This way, any MIDI data may be transferred without being parsed."
//
// HOWEVER  : it will not work with most of USB MIDI drivers and midi apps.
// So, better to use that for debuging purpose.

	if (  ! MIDISerialRxParse ) {
			 MIDIEvent.Event    = 0xF; /* F - single byte (after the event) / Cable number 0*/
			 MIDIEvent.Data1    = receivedByte;
			 // Send immediatly
			 MIDI_SendEventPacket(&MIDIEvent,1);
			 return true;
	}

////////////////////// SYSTEM COMMON MESSAGES ///////////////////////////////

////////////////////////////////////////////////
// REAL TIME MESSAGES
// 11111000= F8= 248 Timing clock  none  none
// 11111001= F9= 249 Undefined (Reserved)  --- ---
// 11111010= FA= 250 Start none  none
// 11111011= FB= 251 Continue  none  none
// 11111100= FC= 252 Stop  none  none
// 11111101= FD= 253 Undefined (Reserved)  --- ---
// 11111110= FE= 254 Active Sensing  none  none
// 11111111= FF= 255 System Reset  none  none

	if ( receivedByte >= 0xF8 ) {
			 // Do not use current event struct , making real time transparent, notably
			 // when in SYSEX mode
       xMIDIEvent.Event    = 0xF; /* F - single byte (after the event) / Cable number 0*/
       xMIDIEvent.Data1    = receivedByte;
			 // Send immediatly
       MIDI_SendEventPacket(&xMIDIEvent,1);
       return true;
  }

	////////////////////////////////////////////////
	// SYSTEM EXCLUSIVE
	////////////////////////////////////////////////
  if (sysExMode ) {
      if ( receivedByte >= 0x80 )  {
        // End of SysEx -- send the last bytes
        // SYSEX can be terminated by an F7 (EOX) or
        // any other status byte except real time
        // Force EOX in any cases to be clean

        // Do we have an sysex internal message to parse ?
        if ( sysExInternal ) {

            // Save the len at the first pos
            sysExInternalBuffer[0] = sysExInternalMsgIdx - sizeof(sysExInternalHeader) + 1;

            // Process internal Sysex
            ProcessSysExInternal();

            sysExMode = sysExInternal = false;
            dataBufferIndex = 0;
            if ( receivedByte == 0xF7 ) return false;
        }
        else
        {
            * ( &MIDIEvent.Data1 + dataBufferIndex++ ) = 0XF7 ;
            MIDIEvent.Event     = 0x4 + dataBufferIndex;
            MIDI_SendEventPacket(&MIDIEvent,dataBufferIndex);
            sysExMode = false;
            dataBufferIndex = 0;
            if ( receivedByte == 0xF7 ) return true;
        }

      }
      else {
        // SYSEX DATA

        // Is it an internal message for the MIDI interface   ?
        // We just receive an F0, check if our header is right after
        if ( !sysExInternal ) {
          if ( MIDIEvent.Data1 == 0xF0 && dataBufferIndex == 1 &&
               receivedByte == sysExInternalHeader[0] ) {
                 sysExInternalMsgIdx = 0;
                 sysExInternal = true;
                 return false;
          }
        }
        else
        {
          sysExInternalMsgIdx++;
          // header complete ?
          if ( sysExInternalMsgIdx >= sizeof(sysExInternalHeader) )  {
            // Start storing the message in the msg buffer
            // If Message too big. Stop exclusive
            if (sysExInternalMsgIdx - sizeof(sysExInternalHeader)  >= (sizeof(sysExInternalBuffer)-1 ) ) {
                sysExMode = sysExInternal = false ;
                dataBufferIndex = 0;
            } else sysExInternalBuffer[sysExInternalMsgIdx - sizeof(sysExInternalHeader) + 1 ]  = receivedByte;
            return false;
          }
          else {
            // Still receiving the header or error
            if ( receivedByte != sysExInternalHeader[sysExInternalMsgIdx] ) {
              sysExMode = sysExInternal = false ;
              dataBufferIndex = 0;
            }
            return false;
          }

        }

        // Sysex data as usual
        * ( &MIDIEvent.Data1 + dataBufferIndex++ ) =  receivedByte ;
        // Packet full ?
        if (dataBufferIndex == 3)  {
          MIDIEvent.Event     = 0x4;    /* 4 - sysex starts or continues */
          MIDI_SendEventPacket(&MIDIEvent,dataBufferIndex);
          dataBufferIndex = 0;
          return true;
        }
        return false;

      }
  }
  // Ignore EOX when not in SYSEX mode
  else if (receivedByte == 0xF7 ) return false;

	////////////////////////////////////////////////
	// SYSTEM COMMON messages
	////////////////////////////////////////////////

  // Clear RUNNING Status
  if (receivedByte >= 0xF0 ) runningStatus  = 0;

  // SYSEX START
  if (receivedByte == 0xF0 ) {
      sysExMode           = true;   // Start SYSEX packets
      MIDIEvent.Data1     = receivedByte;
      dataBufferIndex     = 1;
      return false;       // Stay at the sysex level parsing above
  } else

  // RESERVED SYSTEM COMMON
	if (receivedByte == 0xF4 || receivedByte == 0xF5 ) {
		return false;
	} else

	// Midi Quarter Frame MTC
  if (receivedByte == 0xF1 ) {
			MIDIEvent.Event    	= 0x2; // 2 - two-byte system common message
      nextMidiMsgLength 	= 2; 		// This is an exception. We must fix the length...
      dataBufferIndex 		= 0;
	} else

	// Song position
  if ( receivedByte == 0xF2){
      MIDIEvent.Event     = 0x3; // 3 - three-byte system common message */
      nextMidiMsgLength 	= 3; 	 // This is an exception. We must fix the length...
      dataBufferIndex 		= 0;
  } else

	// Song select
  if (receivedByte == 0xF3  ) {
      MIDIEvent.Event     = 0x2; // 2 - two-byte system common message
      nextMidiMsgLength 	= 2;	 // This is an exception. We must fix the length...
      dataBufferIndex 		= 0;
 } else

	// Tune Request
	if ( receivedByte == 0xF6 ) {
       MIDIEvent.Event    = 0xF; /* F - single byte (after the event) / Cable number 0*/
			 nextMidiMsgLength 	= 1;
			 dataBufferIndex 		= 0;
  } else

////////////////////////////////////////////////
// CHANNEL MESSAGES
////////////////////////////////////////////////

  if (receivedByte >= 0x80)  {
    // A new status byte from serial midi */
    // Fill the USB packet header. Keep the CIN only.
		// Get the data length for this status byte from the table
    MIDIEvent.Event     = (receivedByte >> 4);
    nextMidiMsgLength = BytesIn_USB_MIDI_Command[MIDIEvent.Event];
    dataBufferIndex = 0;
    runningStatus = receivedByte; // Running status
  } else

////////////////////////////////////////////////
// MIDI DATA FROM 00 TO 0X7F
////////////////////////////////////////////////
	if ( dataBufferIndex == 0  )
  {
    // If the previous packet was sent, and we get no status byte,
		// we can apply Running status.
    // 0x90 0x3C 0x7F 0x3C 0x00     <- Note-On Note-Off
    if ( runningStatus >0 ) {
      // As we never delete the MIDI packet, we still have the previous command
      // and status bytes. So we simply update the data index.
      // Move to data 2
      dataBufferIndex ++;
			runningStatus = 0;
    } else return false; // Ignore data according MIDI specs
  }

////////////////////////////////////////////////
// Fill Packet data
////////////////////////////////////////////////

   * ( &MIDIEvent.Data1 + dataBufferIndex++) = receivedByte ;

	// Send complete packet to USB
	if (dataBufferIndex == nextMidiMsgLength) {
    MIDI_SendEventPacket(&MIDIEvent,dataBufferIndex);
    dataBufferIndex = 0;
    return true;
  }

  return false;
}

// ----------------------------------------------------------------------------
// Send a MIDI Event packet to the USB
// ----------------------------------------------------------------------------

static void MIDI_SendEventPacket(const MIDI_EventPacket_t *MIDIEvent,uint8_t dataSize)
{

	// Zero padding
  if (dataSize < 3 ) {
         memset(  (void*) (&MIDIEvent->Data1 + dataSize),0, 3 - dataSize);
  }
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
	// 4 bytes for an USB Midi event
	if ( RingBuffer_GetFreeCount(&USBtoUSART_Buffer) < 4 ) return;

	/* Check if a MIDI command has been received */
	if (MIDI_Device_ReceiveEventPacket(&Keyboard_MIDI_Interface, &MIDIEvent)) {
			uint8_t CIN = MIDIEvent.Event & 0x0F;
			if (CIN >= 2 ) {
					uint8_t BytesIn = BytesIn_USB_MIDI_Command[CIN];
					RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent.Data1 );
					if ( BytesIn >=2 ) RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent.Data2 );
					if ( BytesIn ==3 ) RingBuffer_Insert(&USBtoUSART_Buffer, MIDIEvent.Data3 );
			}
	}
}


///////////////////////////////////////////////////////////////////////////////
// Process internal USBMidiKlik SYSEX
///////////////////////////////////////////////////////////////////////////////
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
