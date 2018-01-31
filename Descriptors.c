/*
             LUFA Library
     Copyright (C) Dean Camera, 2014.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  USB Device Descriptors, for library use when in USB device mode. Descriptors are special
 *  computer-readable structures which the host requests upon device enumeration, to determine
 *  the device's capabilities and functions.
 */

#include "Descriptors.h"
// Only one MIDI device below
//#include "Descriptor_MIDI_KloK.c"
#include "Descriptor_MIDI.c"

#include "Descriptor_CDC_ArduinoUno.c"

// LUFA MIDI Class driver interface configuration and state information.
// NB : C style initializers here.  Forbidden in C++
// Structs are declared as extern in the main cpp module.

USB_ClassInfo_MIDI_Device_t Keyboard_MIDI_Interface =
		{
			.Config =
				{
					.StreamingInterfaceNumber = INTERFACE_ID_AudioStream,
					.DataINEndpoint           =
						{
							.Address          = MIDI_STREAM_IN_EPADDR,
							.Size             = MIDI_STREAM_EPSIZE,
							.Banks            = 1,
						},
					.DataOUTEndpoint           =
						{
							.Address          = MIDI_STREAM_OUT_EPADDR,
							.Size             = MIDI_STREAM_EPSIZE,
							.Banks            = 1,
						},
				},
		};

// LUFA CDC Class driver interface configuration and state information. This structure is
// passed to all CDC Class driver functions, so that multiple instances of the same class
// within a device can be differentiated from one another.
 // NB : C style initializers here.  Forbidden in C++
 // Structs are declared as extern in the main cpp module.

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
		{
			.Config =
				{
					.ControlInterfaceNumber         = INTERFACE_ID_CDC_CCI,
					.DataINEndpoint                 =
						{
							.Address                = CDC_TX_EPADDR,
							.Size                   = CDC_TXRX_EPSIZE,
							.Banks                  = 1,
						},
					.DataOUTEndpoint                =
						{
							.Address                = CDC_RX_EPADDR,
							.Size                   = CDC_TXRX_EPSIZE,
							.Banks                  = 1,
						},
					.NotificationEndpoint           =
						{
							.Address                = CDC_NOTIFICATION_EPADDR,
							.Size                   = CDC_NOTIFICATION_EPSIZE,
							.Banks                  = 1,
						},
				},
		};


/** Language descriptor structure. This descriptor, located in FLASH memory, is returned when the host requests
 *  the string descriptor with index 0 (the first index). It is actually an array of 16-bit integers, which indicate
 *  via the language ID table available at USB.org what languages the device supports for its string descriptors.
 */
const USB_Descriptor_String_t PROGMEM LanguageString = USB_STRING_DESCRIPTOR_ARRAY(LANGUAGE_ID_ENG);

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 */
extern bool MIDIBootMode;

uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint16_t wIndex,
                                    const void** const DescriptorAddress)
{
	const uint8_t  DescriptorType   = (wValue >> 8);
	const uint8_t  DescriptorNumber = (wValue & 0xFF);

	const void* Address = NULL;
	uint16_t    Size    = NO_DESCRIPTOR;

	switch (DescriptorType)
	{
		case DTYPE_Device:
			if (MIDIBootMode) 			Address = (void*)&DeviceDescriptorMIDI;
			else 			Address = (void*)&DeviceDescriptorCDC;

			Size    = sizeof(USB_Descriptor_Device_t);
			break;

		case DTYPE_Configuration:
			if (MIDIBootMode) {
				Address = (void*)&ConfigurationDescriptorMIDI;
				Size    = sizeof(USB_Descriptor_ConfigurationMIDI_t);
			}
			else {
				Address = (void*)&ConfigurationDescriptorCDC;
				Size    = sizeof(USB_Descriptor_ConfigurationCDC_t);
			}
			break;

		case DTYPE_String:
			switch (DescriptorNumber)
			{
				case STRING_ID_Language:
					Address = (void*)&LanguageString;
					Size    = pgm_read_byte(&LanguageString.Header.Size);
					break;

				case STRING_ID_Manufacturer:
					if (MIDIBootMode) {
						Address = (void*)&ManufacturerStringMIDI;
						Size    = pgm_read_byte(&ManufacturerStringMIDI.Header.Size);
					}
					else {
						Address = (void*)&ManufacturerStringCDC;
						Size    = pgm_read_byte(&ManufacturerStringCDC.Header.Size);
					}
					break;

				case STRING_ID_Product:
					if (MIDIBootMode) {
						Address = (void*)&ProductStringMIDI;
						Size    = pgm_read_byte(&ProductStringMIDI.Header.Size);
					} else {
						Address = (void*)&ProductStringCDC;
						Size    = pgm_read_byte(&ProductStringCDC.Header.Size);
					}
					break;

				case STRING_ID_Serial:
						if (!MIDIBootMode) {
							Address = (void*)&ProductSerialCDC;
							Size    = pgm_read_byte(&ProductSerialCDC.Header.Size);
						}
						break;
			}
			break;
	}

	*DescriptorAddress = Address;
	return Size;
}
