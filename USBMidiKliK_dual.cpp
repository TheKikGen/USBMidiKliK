/***********************************************************************
 *  ARDUINO_MIDI_DUAL FIRMWARE -
 *  Franck Touanen - 2017.01.16
 *
 *  Based on :
 *     . The wonderful LUFA low-level midi lib and examples by Dean Camera
 *               (http://www.fourwalledcubicle.com/LUFA.php)
 *     . Inspired by HIDUINO by Dimitri Diakopoulos
 *               (http://www.dimitridiakopoulos.com)
 *     . Inspired also by dualMocoLUFA Project by morecat_la
 *               (http://morecatlab.akiba.coocan.jp/)
  *
 *  Compiled against the last LUFA version / MIDI library

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
bool MIDIHighSpeed = false;	// 0: normal speed(31250bps), 1: high speed (1250000bps)
uint8_t UsbMIDITagPacket = 0x00 ;  	// Used to tag MIDI events when sending to serial if
																		// required. So it is possible to get CN

// Ring Buffers

static RingBuffer_t USBtoUSART_Buffer;           // Circular buffer to hold host data
static uint8_t      USBtoUSART_Buffer_Data[128]; // USB to USART_Buffer
static RingBuffer_t USARTtoUSB_Buffer;           // Circular buffer to hold data from the serial port
static uint8_t      USARTtoUSB_Buffer_Data[128]; // USART to USB_Buffer

extern USB_ClassInfo_MIDI_Device_t Keyboard_MIDI_Interface;
extern USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface;

///////////////////////////////////////////////////////////////////////////////
// MAIN START HERE
///////////////////////////////////////////////////////////////////////////////

int  main(void)
{

	SetupHardware();
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);

	if (MIDIBootMode) ProcessMidiUsbMode(); // Inifinite loop

	else ProcessSerialUsbMode();

///////////////////////////////////////////////////////////////////////////////
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
// Note : When PB3 is grounded, this allows the HighSpeed mode for MIDI
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
	// PB2 = (MIDI/ Arduino SERIAL)
	// PB3 = (NORMAL/HIGH) SPEED

  DDRB  = 0x02;		// SET PB1 as OUTPUT and PB2/PB3 as INPUT
  PORTB = 0x0C;		// PULL-UP PB2/PB3

	MIDIBootMode  = ( (PINB & 0x04) == 0 ) ? false : true;
	MIDIHighSpeed = ( (PINB & 0x08) == 0 ) ? true  : false ;

  if (MIDIBootMode) {
		if ( MIDIHighSpeed ) Serial_Init(1250000, false);
		else 		Serial_Init(31250, false);
	}
	else Serial_Init(9600, false);

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

	UCSR1B |= (1 << RXCIE1 ); // Enable the USART Receive Complete interrupt ( USART_RXC )
	GlobalInterruptEnable();

	for (;;) {

		if (tx_ticks > 0) tx_ticks--;
		else if (tx_ticks == 0) LEDs_TurnOffLEDs(LEDS_LED2);

		if (rx_ticks > 0) rx_ticks--;
		else if (rx_ticks == 0) LEDs_TurnOffLEDs(LEDS_LED1);

		ProcessUsbToMidi();
		// NB : ProcessMidiToUsb is done in the ISR directly because of
		// real time aspects

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
static bool ProcessMidiToUsb(uint8_t receivedByte)
{
	static  MIDI_EventPacket_t MIDIEvent;
	static  bool sysExMode = false;					// True if SYSEX active
	static 	bool MTCFrame=false;						// True if waiting the second MTC byte
																					// MTC is a REALTIME msg
	static  uint8_t dataBufferIndex = 0;		// Index on Data1 to Data3
  static  uint8_t nextMidiMsgLength=0;		// Len of the current midi message processed
  static  uint8_t lastVoiceStatus=0;			// Used for running status

	if (USB_DeviceState != DEVICE_STATE_Configured) return false;

	////////////////////// SYSTEM COMMON MESSAGES ///////////////////////////////

	////////////////////////////////////////////////
	// USB MIDI packet tagging activation
	// NOT STANDARD !!!  SPECIFIC TO USBMIDIKLIK
	////////////////////////////////////////////////
	// The message status is defined in usbMidiKlik.H
	// Use only an undefined one and this if bloc must
	// stay here, before the reserved/undefines if bloc.
	// This should be totally transparent to any device.
	// Later : better to implement that with SYSEX...
	if ( receivedByte == USBMIDI_PKTAG ) {
			//  MIDI Message is 2 bytes, structured as follow :
			//  [USBMIDI_PKTAG] 00X00  :  Inactivate tagging
			//  [USBMIDI_PKTAG] 01X01  :  Activate tagging
			//  [USBMIDI_PKTAG] 01X02  :  Get tagging current status
			//
			// Packet Tagging looks like :
			// [USBMIDI_PKTAG][CN/CIN][Data1][Data2][Data3]

			// A tagged packet includes the CN/CIN. This can be usefull
			// for debugging or routing reasons.

			// Fill the USB packet header.
			MIDIEvent.Event     = 0x02; /* 2 - two-byte system common message */
			nextMidiMsgLength 	 = 2;
			dataBufferIndex 		 = 0;
			lastVoiceStatus 		 = 0;
	}

	else

	////////////////////////////////////////////////
  // RESERVED
  // 11110100= F4= 244 Undefined (Reserved)  --- ---
  // 11110101= F5= 245 Undefined (Reserved)  --- ---
  // 11111001= F9= 249 Undefined (Reserved)  --- ---
  // 11111101= FD= 253 Undefined (Reserved)  --- ---

  if (receivedByte == 0xF4 || receivedByte == 0xF5 ||
 		 receivedByte == 0xF9 || receivedByte == 0xFD)
 		 return false;
  else

	////////////////////////////////////////////////
  // REAL TIME MESSAGES
	// 11111000= F8= 248 Timing clock  none  none
  // 11111010= FA= 250 Start none  none
  // 11111011= FB= 251 Continue  none  none
  // 11111100= FC= 252 Stop  none  none
  // 11111110= FE= 254 Active Sensing  none  none
  // 11111111= FF= 255 System Reset  none  none

	if ( receivedByte >= 0xF8 ) {
       MIDIEvent.Event    = 0xF; /* F - single byte (after the event) / Cable number 0*/
       MIDIEvent.Data1    = receivedByte;
			 // Send immediatly
       MIDI_SendEventPacket(&MIDIEvent,1);
       return true;
  } else

	// 11110001= F1= 241 MIDI Time Code Qtr. Frame -see spec-  -see spec-
	// Midi Quarter Frame MTC
  if (receivedByte == 0xF1 ) {
			lastVoiceStatus = 0; // Cancel any running status
			MIDIEvent.Event    = 0x2; // 2 - two-byte system common message
			MIDIEvent.Data1    = receivedByte;
			MTCFrame = true;
			return false;
	} else if (MTCFrame) {
			MIDIEvent.Data2    = receivedByte;
			// Send immediatly
			MIDI_SendEventPacket(&MIDIEvent,2);
			MTCFrame = false;
			return true;
	} else

	////////////////////////////////////////////////
	// SYSTEM EXCLUSIVE
	// 11110000= F0= 240  System Exclusive  **  **

  if (receivedByte == 0xF0 ) {
      sysExMode = true;  // Start SYSEX
      dataBufferIndex = 0;
      lastVoiceStatus = 0;
  } else

  // END OF SYSEX
  // 11110111= F7= 247 End of SysEx (EOX)  none  none
  if (receivedByte == 0xF7 ) {
			lastVoiceStatus = 0;
      sysExMode = false; // SYSEX END
  } else

  ////////////////////////////////////////////////
  // OTHERS COMMON MESSAGES
  // 11110010= F2= 242 Song Position Pointer LSB MSB
  // 11110011= F3= 243 Song Select (Song #)  (0-127) none
	// 11110110= F6= 246 Tune request  none  none
  if ( receivedByte == 0xF6 ) {
       MIDIEvent.Event    = 0xF; /* F - single byte (after the event) / Cable number 0*/
       MIDIEvent.Data1    = receivedByte;
			 // Send immediatly
       MIDI_SendEventPacket(&MIDIEvent,1);
       return true;
  } else

  // Song position
  if ( receivedByte == 0xF2){
      // Fill the USB packet header. Keep the CIN only.
      MIDIEvent.Event     = 0x3; // 3 - three-byte system common message */
      // This is an exception. We must fix the length...
      nextMidiMsgLength = 3;
      dataBufferIndex = 0;
      lastVoiceStatus = 0;

  } else

  // Song select
  if (receivedByte == 0xF3  ) {
      // Fill the USB packet header. Keep the CIN only.
      MIDIEvent.Event     = 0x2; // 2 - two-byte system common message
      // This is an exception. We must fix the length...
      nextMidiMsgLength = 2;
      dataBufferIndex = 0;
      lastVoiceStatus = 0;
 } else



  ////////////////////// CHANNEL VOICES MESSAGES //////////////////////
 if (receivedByte >= 0x80)  {

	  // Still some data to send here ?
    // Not normal. Probably a message dropped...
    // if (dataBufferIndex > 0 ) {
    // -[PACKET DROPPED]"
    //}
		// =>Incomplete messages due to latency are purely ignored

    // A new status byte from serial midi */
    // Fill the USB packet header. Keep the CIN only.
    MIDIEvent.Event     = (receivedByte >> 4);
    // Get the data length for this status byte from the table
    nextMidiMsgLength = BytesIn_USB_MIDI_Command[MIDIEvent.Event];
    // Reset the data index . That was the first byte.
    dataBufferIndex = 0;
    lastVoiceStatus = receivedByte;
  }

  else if (dataBufferIndex == 0 && !sysExMode)
  {
    // Running status
    // 0x90 0x3C 0x7F 0x3C 0x00     <- Note-On Note-Off
    if ( lastVoiceStatus >0 ) {
      // As we never delete the MIDI packet, we still have the previous command
      // and status bytes. So we simply update the data and index.
      // Move to data 2
      dataBufferIndex ++;
    } else return false;
  }

	 //////////////// Fill Packet data ///////////////////////

   * ( &MIDIEvent.Data1 + dataBufferIndex) = receivedByte ;
   dataBufferIndex++;

  /////////////// Process SYSEX ////////////////////////////
  if (sysExMode) {
    // Packet full ?
    if (dataBufferIndex == 3)  {
      MIDIEvent.Event     = 0x4;
      MIDI_SendEventPacket(&MIDIEvent,dataBufferIndex);
      dataBufferIndex = 0;
      return true;
    }
  }
  else if (receivedByte == 0xF7)
  {
    // End of SysEx -- send the last bytes
    MIDIEvent.Event     = 0x4 + dataBufferIndex;
    MIDI_SendEventPacket(&MIDIEvent,dataBufferIndex);
    dataBufferIndex = 0;
    lastVoiceStatus = 0;
    return true;
  }
	/////////////// Send complete packet to USB //////////////
	else if (dataBufferIndex == nextMidiMsgLength)
  {
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

	// Intercept the tagging packet command
	//  MIDI Message is 2 bytes, structured as follow :
	//  [USBMIDI_PKTAG] 00X00  :  Inactivate tagging
	//  [USBMIDI_PKTAG] 01X01  :  Activate tagging
	//  [USBMIDI_PKTAG] 01X02  :  Get tagging current status
	//
	// Packet Tagging looks like :
	// [USBMIDI_PKTAG][CN/CIN][Data1][Data2][Data3]

	// Long packets include the CN/CIN. This can be usefull
	// for debugging or routing reasons.
	// NB : This command is unidirectionnal, from serial only.

	if ( MIDIEvent->Data1 == USBMIDI_PKTAG ) {
			// Current status
			if ( MIDIEvent->Data2 == 0X02 ) {
				if ( Serial_IsSendReady() ) {
					Serial_SendByte(USBMIDI_PKTAG); // A first time as Tag
					Serial_SendByte(USBMIDI_PKTAG); // A second time as message type
					Serial_SendByte( UsbMIDITagPacket );
				}
			} else {
			// Set
					UsbMIDITagPacket = MIDIEvent->Data2 ;
			}
			// This is not obviously for USB !
			return;
	}
	// Zero padding
  if (dataSize < 3 ) {
         memset(  (void*) (&MIDIEvent->Data1 + dataSize),0, 3 - dataSize);
  }
 	MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, MIDIEvent);
	MIDI_Device_Flush(&Keyboard_MIDI_Interface);

	LEDs_TurnOnLEDs(LEDS_LED1);
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

	while (MIDI_Device_ReceiveEventPacket(&Keyboard_MIDI_Interface, &MIDIEvent))
	{
			// Passthrough to Arduino
			uint8_t CIN = MIDIEvent.Event & 0x0F;
			if (CIN >= 2 ) {
					uint8_t BytesIn = BytesIn_USB_MIDI_Command[CIN];
					if ( Serial_IsSendReady() ) {

						// if Tagging mode active, TAG ONLY TO SERIAL
						if (UsbMIDITagPacket) {
							Serial_SendByte(USBMIDI_PKTAG);
							Serial_SendByte(MIDIEvent.Event);
						}
						Serial_SendData	(	(void *)&MIDIEvent.Data1, BytesIn);

						LEDs_TurnOnLEDs(LEDS_LED2);
						tx_ticks = TICK_COUNT;
					}
			}
	}
}

///////////////////////////////////////////////////////////////////////////////
// ISR to manage the reception of data from the midi/serial port, placing
// received bytes into a circular buffer for later transmission to the host.
///////////////////////////////////////////////////////////////////////////////
// Parse via Arduino/Serial
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	if (MIDIBootMode )
			ProcessMidiToUsb(ReceivedByte);
	else RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}
