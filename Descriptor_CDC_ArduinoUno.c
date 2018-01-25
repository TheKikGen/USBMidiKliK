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

/** Device descriptor structure. This descriptor, located in FLASH memory, describes the overall
 *  device characteristics, including the supported USB version, control endpoint size and the
 *  number of device configurations. The descriptor is read out by the USB host when the enumeration
 *  process begins.
 */

// CDC DSECRIPTOR FOR A GENUINE ARDUINO UNO V3
/* idVendor                 : 0x2A03
   idProduct                : 0x0043
   bcdDevice                : 0x0001
*/
const USB_Descriptor_Device_t PROGMEM DeviceDescriptorCDC =
{
	.Header                 = {.Size = sizeof(USB_Descriptor_Device_t), .Type = DTYPE_Device},

	.USBSpecification       = VERSION_BCD(1,1,0),
	.Class                  = CDC_CSCP_CDCClass,
	.SubClass               = CDC_CSCP_NoSpecificSubclass,  // 0x00
	.Protocol               = CDC_CSCP_NoSpecificProtocol,  // 0x00

	.Endpoint0Size          = FIXED_CONTROL_ENDPOINT_SIZE,

	.VendorID               = 0x2A03,
	.ProductID              = 0x0043,
	.ReleaseNumber          = VERSION_BCD(0,0,1),

	.ManufacturerStrIndex   = STRING_ID_Manufacturer,
	.ProductStrIndex        = STRING_ID_Product,
	.SerialNumStrIndex      = STRING_ID_Serial,

	.NumberOfConfigurations = FIXED_NUM_CONFIGURATIONS
};

/** Configuration descriptor structure. This descriptor, located in FLASH memory, describes the usage
 *  of the device in one of its supported configurations, including information about any device interfaces
 *  and endpoints. The descriptor is read out by the USB host during the enumeration process when selecting
 *  a configuration so that the host may correctly communicate with the USB device.

 ---------------- Configuration Descriptor -----------------
bLength                  : 0x09 (9 bytes)
bDescriptorType          : 0x02 (Configuration Descriptor)
wTotalLength             : 0x003E (62 bytes)
bNumInterfaces           : 0x02
bConfigurationValue      : 0x01
iConfiguration           : 0x00 (No String Descriptor)
bmAttributes             : 0xC0
D7: Reserved, set 1     : 0x01
D6: Self Powered        : 0x01 (yes)
D5: Remote Wakeup       : 0x00 (no)
D4..0: Reserved, set 0  : 0x00
MaxPower                 : 0x32 (100 mA)
 */

const USB_Descriptor_ConfigurationCDC_t PROGMEM ConfigurationDescriptorCDC =
{
	.Config =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Configuration_Header_t), .Type = DTYPE_Configuration},

			.TotalConfigurationSize = sizeof(USB_Descriptor_ConfigurationCDC_t),
			.TotalInterfaces        = 2,

			.ConfigurationNumber    = 1,
			.ConfigurationStrIndex  = NO_DESCRIPTOR,

			.ConfigAttributes       = (USB_CONFIG_ATTR_RESERVED | USB_CONFIG_ATTR_SELFPOWERED),

			.MaxPowerConsumption    = USB_CONFIG_POWER_MA(100)
		},
/*
---------------- Interface Descriptor -----------------
bLength                  : 0x09 (9 bytes)
bDescriptorType          : 0x04 (Interface Descriptor)
bInterfaceNumber         : 0x00
bAlternateSetting        : 0x00
bNumEndpoints            : 0x01 (1 Endpoint)
bInterfaceClass          : 0x02 (Communications and CDC Control)
bInterfaceSubClass       : 0x02
bInterfaceProtocol       : 0x01
iInterface               : 0x00 (No String Descriptor)
*/
	.CDC_CCI_Interface =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},

			.InterfaceNumber        = INTERFACE_ID_CDC_CCI,
			.AlternateSetting       = 0,

			.TotalEndpoints         = 1,

			.Class                  = CDC_CSCP_CDCClass,   // 0x02
			.SubClass               = CDC_CSCP_ACMSubclass,   // 0x02
			.Protocol               = CDC_CSCP_ATCommandProtocol, // 0x01

			.InterfaceStrIndex      = NO_DESCRIPTOR
		},
/*
-------------- CDC Interface Descriptor ---------------
bFunctionLength          : 0x05 (5 bytes)
bDescriptorType          : 0x24 (Interface)
bDescriptorSubType       : 0x00 (Header Functional Descriptor)
bcdCDC                   : 0x1001 (CDC Version 10.01)

		-------------- CDC Interface Descriptor ---------------
bFunctionLength          : 0x04 (4 bytes)
bDescriptorType          : 0x24 (Interface)
bDescriptorSubType       : 0x02 (Abstract Control Management Functional Descriptor)
bmCapabilities           : 0x06

		-------------- CDC Interface Descriptor ---------------
bFunctionLength          : 0x05 (5 bytes)
bDescriptorType          : 0x24 (Interface)
bDescriptorSubType       : 0x06 (Union Functional Descriptor)
bControlInterface        : 0x00
bSubordinateInterface[0] : 0x01
*/
	.CDC_Functional_Header =
			{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalHeader_t), .Type = DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_Header,
			.CDCSpecification       = 0x1001,
			},

	.CDC_Functional_ACM =
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalACM_t), .Type = DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_ACM,
			.Capabilities           = 0x06
		},

	.CDC_Functional_Union =
		{
			.Header                 = {.Size = sizeof(USB_CDC_Descriptor_FunctionalUnion_t), .Type = DTYPE_CSInterface},
			.Subtype                = CDC_DSUBTYPE_CSInterface_Union,
			.MasterInterfaceNumber  = INTERFACE_ID_CDC_CCI,
			.SlaveInterfaceNumber   = INTERFACE_ID_CDC_DCI,
		},


/*
----------------- Endpoint Descriptor -----------------
bLength                  : 0x07 (7 bytes)
bDescriptorType          : 0x05 (Endpoint Descriptor)
bEndpointAddress         : 0x82 (Direction=IN EndpointID=2)
bmAttributes             : 0x03 (TransferType=Interrupt)
wMaxPacketSize           : 0x0008 (8 bytes)
bInterval                : 0xFF (255 ms)
*/
	.CDC_NotificationEndpoint =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
			.EndpointAddress        = 0x82,
			.Attributes             = 0x03,
			.EndpointSize           = 0x08,
			.PollingIntervalMS      = 0xFF
		},
/*
		---------------- Interface Descriptor -----------------
	bLength                  : 0x09 (9 bytes)
	bDescriptorType          : 0x04 (Interface Descriptor)
	bInterfaceNumber         : 0x01
	bAlternateSetting        : 0x00
	bNumEndpoints            : 0x02 (2 Endpoints)
	bInterfaceClass          : 0x0A (CDC-Data)
	bInterfaceSubClass       : 0x00
	bInterfaceProtocol       : 0x00
	iInterface               : 0x00 (No String Descriptor)
	*/

	.CDC_DCI_Interface =
		{
			.Header                 = {.Size = sizeof(USB_Descriptor_Interface_t), .Type = DTYPE_Interface},
			.InterfaceNumber        = 0x01,
			.AlternateSetting       = 0,
			.TotalEndpoints         = 2,
			.Class                  = CDC_CSCP_CDCDataClass,
			.SubClass               = CDC_CSCP_NoDataSubclass,
			.Protocol               = CDC_CSCP_NoDataProtocol,
			.InterfaceStrIndex      = NO_DESCRIPTOR
		},

/*		----------------- Endpoint Descriptor -----------------
bLength                  : 0x07 (7 bytes)
bDescriptorType          : 0x05 (Endpoint Descriptor)
bEndpointAddress         : 0x04 (Direction=OUT EndpointID=4)
bmAttributes             : 0x02 (TransferType=Bulk)
wMaxPacketSize           : 0x0040 (64 bytes)
bInterval                : 0x01 (ignored)
*/
		.CDC_DataOutEndpoint =
			{
				.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
				.EndpointAddress        = 0x04,
				.Attributes             = 0x02,
				.EndpointSize           = CDC_TXRX_EPSIZE,
				.PollingIntervalMS      = 0x01
			},
/*		----------------- Endpoint Descriptor -----------------
	bLength                  : 0x07 (7 bytes)
	bDescriptorType          : 0x05 (Endpoint Descriptor)
	bEndpointAddress         : 0x83 (Direction=IN EndpointID=3)
	bmAttributes             : 0x02 (TransferType=Bulk)
	wMaxPacketSize           : 0x0040 (64 bytes)
	bInterval                : 0x01 (ignored)
*/
		.CDC_DataInEndpoint =
			{
				.Header                 = {.Size = sizeof(USB_Descriptor_Endpoint_t), .Type = DTYPE_Endpoint},
				.EndpointAddress        = 0x83,
				.Attributes             = 0x02,
				.EndpointSize           = CDC_TXRX_EPSIZE,
				.PollingIntervalMS      = 0x01
			}
};

/** Manufacturer descriptor string. This is a Unicode string containing the manufacturer's details in human readable
 *  form, and is read out upon request by the host when the appropriate string ID is requested, listed in the Device
 *  Descriptor.

 ------ String Descriptor 0 ------
bLength                  : 0x04 (4 bytes)
bDescriptorType          : 0x03 (String Descriptor)
Language ID[0]           : 0x0409 (English - United States)
 ------ String Descriptor 1 ------
bLength                  : 0x32 (50 bytes)
bDescriptorType          : 0x03 (String Descriptor)
Language 0x0409          : "Arduino Srl            ø"  *!*ERROR  contains 1 NULL character
 ------ String Descriptor 2 ------
bLength                  : 0x18 (24 bytes)
bDescriptorType          : 0x03 (String Descriptor)
Language 0x0409          : "Arduino Uno"
 ----- String Descriptor 0xDC -----
bLength                  : 0x2A (42 bytes)
bDescriptorType          : 0x03 (String Descriptor)
Language 0x0409          : "854393131303513111B1"

*/
const USB_Descriptor_String_t PROGMEM ManufacturerStringCDC = USB_STRING_DESCRIPTOR(L"Arduino Srl (www.arduino.org)");
const USB_Descriptor_String_t PROGMEM ProductStringCDC = USB_STRING_DESCRIPTOR(L"Arduino Uno");
const USB_Descriptor_String_t PROGMEM ProductSerialCDC = USB_STRING_DESCRIPTOR(L"854393131303513111B1");
