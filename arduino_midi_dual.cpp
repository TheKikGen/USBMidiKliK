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
 *	   . Francois Best's Arduino MIDI Library, for sure the best one !
 *						(https://github.com/FortySevenEffects/arduino_midi_library)
 *
 *  Compiled against the last LUFA version / MIDI library
 ***********************************************************************/

#include "arduino_midi_dual.h"

uint16_t tx_ticks = 0;
uint16_t rx_ticks = 0;
const uint16_t TICK_COUNT = 5000;

bool MIDIBootMode  = false;
bool MIDIHighSpeed = false;	// 0: normal speed(31250bps), 1: high speed (1250000bps)

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

	if (MIDIBootMode) processMIDI(); // Inifinite loop

	else processUSBtoSerial();

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
		ConfigSuccess &= Endpoint_ConfigureEndpoint(MIDI_STREAM_IN_EPADDR, EP_TYPE_BULK, MIDI_STREAM_EPSIZE, 1);
		ConfigSuccess &= Endpoint_ConfigureEndpoint(MIDI_STREAM_OUT_EPADDR, EP_TYPE_BULK, MIDI_STREAM_EPSIZE, 1);
		//ConfigSuccess &= MIDI_Device_ConfigureEndpoints(&Keyboard_MIDI_Interface);
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
///////////////////////////////////////////////////////////////////////////////

static void processUSBtoSerial(void) {

	RingBuffer_InitBuffer(&USBtoUSART_Buffer, USBtoUSART_Buffer_Data, sizeof(USBtoUSART_Buffer_Data));
	RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));
	GlobalInterruptEnable();

           ///////////////////////  INFINITE LOOP /////////////////////////
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
///////////////////////////////////////////////////////////////////////////////

static void processMIDI(void) {

	RingBuffer_InitBuffer(&USARTtoUSB_Buffer, USARTtoUSB_Buffer_Data, sizeof(USARTtoUSB_Buffer_Data));
  UCSR1B |= (1 << RXCIE1 ); // Enable the USART Receive Complete interrupt ( USART_RXC )
	sei () ; // Enable the Global Interrupt Enable flag so that interrupts can be processe
	//GlobalInterruptEnable();

  ///////////////////////  INFINITE LOOP /////////////////////////
	for (;;) {

		if (tx_ticks > 0) tx_ticks--;
		else if (tx_ticks == 0) LEDs_TurnOffLEDs(LEDS_LED2);

		if (rx_ticks > 0) rx_ticks--;
		else if (rx_ticks == 0) LEDs_TurnOffLEDs(LEDS_LED1);

		// Device must be connected and configured for the task to run
		if (USB_DeviceState == DEVICE_STATE_Configured) {
		 	if (!(RingBuffer_IsEmpty(&USARTtoUSB_Buffer) ) )
		 								midiParse(RingBuffer_Remove(&USARTtoUSB_Buffer));

			processMIDItoUSB();
			processUSBtoMIDI();

		}

    MIDI_Device_USBTask(&Keyboard_MIDI_Interface);
		USB_USBTask();
	}
}

///////////////////////////////////////////////////////////////////////////////
// ARDUINO SERIAL  TO HOST  MIDI IN
///////////////////////////////////////////////////////////////////////////////
static void processMIDItoUSB(void) {

	// Select the MIDI IN stream
	Endpoint_SelectEndpoint(MIDI_STREAM_IN_EPADDR);

	if ( Endpoint_IsINReady() ) 	{
		if (mPendingMessageValid == true) {
			mPendingMessageValid = false;

			// Write the MIDI event packet to the endpoint
			Endpoint_Write_Stream_LE(&mCompleteMessage, sizeof(mCompleteMessage), NULL);

			// Clear out complete message
			memset(&mCompleteMessage, 0, sizeof(mCompleteMessage));

			// Send the data in the endpoint to the host
			Endpoint_ClearIN();

			LEDs_TurnOnLEDs(LEDS_LED2);
			tx_ticks = TICK_COUNT;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// HOST MIDI OUT TO ARDUINO SERIAL
///////////////////////////////////////////////////////////////////////////////

static void processUSBtoMIDI(void)
{

	Endpoint_SelectEndpoint(MIDI_STREAM_OUT_EPADDR);

	/* Check if a MIDI command has been received */
	if (Endpoint_IsOUTReceived())
	{
		MIDI_EventPacket_t MIDIEvent;
		/* Read the MIDI event packet from the endpoint */
		Endpoint_Read_Stream_LE(&MIDIEvent, sizeof(MIDIEvent), NULL);

		// Passthrough to Arduino
		Serial_SendByte(MIDIEvent.Data1);
		Serial_SendByte(MIDIEvent.Data2);
		Serial_SendByte(MIDIEvent.Data3);

		LEDs_TurnOnLEDs(LEDS_LED1);
		rx_ticks = TICK_COUNT;

		/* If the endpoint is now empty, clear the bank */
		if (!(Endpoint_BytesInEndpoint()))
		{
			/* Clear the endpoint ready for new packet */
			Endpoint_ClearOUT();
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
	RingBuffer_Insert(&USARTtoUSB_Buffer, UDR1);
}

///////////////////////////////////////////////////////////////////////////////
// MIDI PARSER
///////////////////////////////////////////////////////////////////////////////

void midiParse(uint8_t extracted )
{
	// Borrowed + Modified from Francois Best's Arduino MIDI Library
	// https://github.com/FortySevenEffects/arduino_midi_library
  if (mPendingMessageIndex == 0)
  {
      // Start a new pending message
      mPendingMessage[0] = extracted;

      // Check for running status first
      if (isChannelMessage(getTypeFromStatusByte(mRunningStatus_RX)))
      {
          // Only these types allow Running Status

          // If the status byte is not received, prepend it to the pending message
          if (extracted < 0x80)
          {
              mPendingMessage[0]   = mRunningStatus_RX;
              mPendingMessage[1]   = extracted;
              mPendingMessageIndex = 1;
          }
          // Else we received another status byte, so the running status does not apply here.
          // It will be updated upon completion of this message.
      }

      switch (getTypeFromStatusByte(mPendingMessage[0]))
      {
          // 1 byte messages
          case Start:
          case Continue:
          case Stop:
          case Clock:
          case ActiveSensing:
          case SystemReset:
          case TuneRequest:
              // Handle the message type directly here.
          	mCompleteMessage.Event 	 = MIDI_EVENT(0, getTypeFromStatusByte(mPendingMessage[0]));
              mCompleteMessage.Data1   = mPendingMessage[0];
              mCompleteMessage.Data2   = 0;
              mCompleteMessage.Data3   = 0;
              mPendingMessageValid  	 = true;

              // We still need to reset these
              mPendingMessageIndex = 0;
              mPendingMessageExpectedLength = 0;

              return;
              break;

          // 2 bytes messages
          case ProgramChange:
          case AfterTouchChannel:
          case TimeCodeQuarterFrame:
          case SongSelect:
              mPendingMessageExpectedLength = 2;
              break;

          // 3 bytes messages
          case NoteOn:
          case NoteOff:
          case ControlChange:
          case PitchBend:
          case AfterTouchPoly:
          case SongPosition:
              mPendingMessageExpectedLength = 3;
              break;

          case SystemExclusive:
              break;

          case InvalidType:
          default:
              // Something bad happened
              break;
      }

      if (mPendingMessageIndex >= (mPendingMessageExpectedLength - 1))
      {
          // Reception complete
          mCompleteMessage.Event = MIDI_EVENT(0, getTypeFromStatusByte(mPendingMessage[0]));
          mCompleteMessage.Data1 = mPendingMessage[0]; // status = channel + type
					mCompleteMessage.Data2 = mPendingMessage[1];

          // Save Data3 only if applicable
          if (mPendingMessageExpectedLength == 3)
              mCompleteMessage.Data3 = mPendingMessage[2];
          else
              mCompleteMessage.Data3 = 0;

          mPendingMessageIndex = 0;
          mPendingMessageExpectedLength = 0;
          mPendingMessageValid = true;
          return;
      }
      else
      {
          // Waiting for more data
          mPendingMessageIndex++;
      }
  }
  else
  {
      // First, test if this is a status byte
      if (extracted >= 0x80)
      {
          // Reception of status bytes in the middle of an uncompleted message
          // are allowed only for interleaved Real Time message or EOX
          switch (extracted)
          {
              case Clock:
              case Start:
              case Continue:
              case Stop:
              case ActiveSensing:
              case SystemReset:

                  // Here we will have to extract the one-byte message,
                  // pass it to the structure for being read outside
                  // the MIDI class, and recompose the message it was
                  // interleaved into. Oh, and without killing the running status..
                  // This is done by leaving the pending message as is,
                  // it will be completed on next calls.
         		 	mCompleteMessage.Event = MIDI_EVENT(0, getTypeFromStatusByte(extracted));
          		mCompleteMessage.Data1 = extracted;
                  mCompleteMessage.Data2 = 0;
                  mCompleteMessage.Data3 = 0;
                 	mPendingMessageValid   = true;
                  return;
                  break;
              default:
                  break;
          }
      }

      // Add extracted data byte to pending message
      mPendingMessage[mPendingMessageIndex] = extracted;

      // Now we are going to check if we have reached the end of the message
      if (mPendingMessageIndex >= (mPendingMessageExpectedLength - 1))
      {

      		mCompleteMessage.Event = MIDI_EVENT(0, getTypeFromStatusByte(mPendingMessage[0]));
          mCompleteMessage.Data1 = mPendingMessage[0];
          mCompleteMessage.Data2 = mPendingMessage[1];

          // Save Data3 only if applicable
          if (mPendingMessageExpectedLength == 3)
              mCompleteMessage.Data3 = mPendingMessage[2];
          else
              mCompleteMessage.Data3 = 0;

          // Reset local variables
          mPendingMessageIndex = 0;
          mPendingMessageExpectedLength = 0;
          mPendingMessageValid = true;

          // Activate running status (if enabled for the received type)
          switch (getTypeFromStatusByte(mPendingMessage[0]))
          {
              case NoteOff:
              case NoteOn:
              case AfterTouchPoly:
              case ControlChange:
              case ProgramChange:
              case AfterTouchChannel:
              case PitchBend:
                  // Running status enabled: store it from received message
                  mRunningStatus_RX = mPendingMessage[0];
                  break;

              default:
                  // No running status
                  mRunningStatus_RX = InvalidType;
                  break;
          }
          return;
      }
      else
      {
          // Not complete? Then update the index of the pending message.
          mPendingMessageIndex++;
      }
  }
}

///////////////////////////////////////////////////////////////////////////////
// MIDI Utility Functions
///////////////////////////////////////////////////////////////////////////////

uint8_t getStatus(MidiMessageType inType, uint8_t inChannel)
{
    return (   inType |  ( (inChannel - 1) & 0x0f) );
}

uint8_t getTypeFromStatusByte(uint8_t inStatus)
{
    if ((inStatus  < 0x80) ||
        (inStatus == 0xf4) ||
        (inStatus == 0xf5) ||
        (inStatus == 0xf9) ||
        (inStatus == 0xfD))
    {
        // Data bytes and undefined.
        return InvalidType;
    }

    if (inStatus < 0xf0)
    {
        // Channel message, remove channel nibble.
        return (inStatus & 0xf0);
    }

    return inStatus;
}

uint8_t getChannelFromStatusByte(uint8_t inStatus)
{
	return (inStatus & 0x0f) + 1;
}

bool isChannelMessage(uint8_t inType)
{
    return (inType == NoteOff           ||
            inType == NoteOn            ||
            inType == ControlChange     ||
            inType == AfterTouchPoly    ||
            inType == AfterTouchChannel ||
            inType == PitchBend         ||
            inType == ProgramChange);
}
