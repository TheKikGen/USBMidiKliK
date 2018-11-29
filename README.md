# USBMidiKliK
A robust USB MIDI Arduino firmware, with a dual bootloader, based on the last version of the LUFA library.

As other project, like HIDUINO, or MOCOLUFA (thanks to them for inspiration), USBMIDIKLIK allows your Arduino board to become a very reliable MIDI IN/OUT USB interface.  Despite the very good work done on these projects, i was facing some issues...
An heavy MIDI traffic was blocking the serial, and some MIDI messages were purely ignored by the parser, like the song pointer position for example... more, these projects rely on a quite old version of the LUFA library.

USBMidiKliK uses interrupts and ring buffers to ensure that (fast) USB to (slow) midi transfers are reliable, plus a "more transparent as possible" midi parser. MIDI product device name is integrated in the makefile, and can also be modified by sysex...so easy to change.

This firmware is uploaded in the ATMEGA8U2 chip managing the USB, and changes the default USB serial descriptors to the MIDI ones.
For more convenience when updates are needed, a "dual mode" is embedded, allowing to switch back to the USB serial : when the PB2/MOSI pin of the ATMEGA8U2 is connected to ground, the Arduino is a classical one again, and you can change and upload a new firmware in the ATMEGA328P (UNO) with the standard Arduino IDE.
NEW !! It is now possible to change the boot mode with a sysex command without installing a jumper on PB2/GND. See below.

To use the Uno as a USB to MIDI converter, you must upload the "USB converter" ino sketch in the ATMEGA328P with the Arduino IDE.
Then, MIDI mode , Serial directions are the following :

                                                                         ATMEGA 328P
           USB                         ATMEGA8U2                        UART NOT ACTIVE
       -------------        ------------------------------          -------------------------
       IN Endpoint  o<-----o| USBOUT  (usbMidiKliK )  RX |o<-+----o|  TX 1 (pinMode INPUT)  |    +--<  MIDI IN JACK
       OUT Endpoint o----->o| USBIN   ( firmware   )  TX |o--|-+->o|  RX 0 (pinMode INPUT)  |  +-|-->  MIDI OUT JACK
                                                             | |   | (USB Converter sketch) |  | |
                                                             | |_______________________________| |
                                                             |___________________________________|

If you need USB to talk with external MIDI IN/OUT (with DIN jacks), the RX/TX on the ATMEGA328P must be inactivated by setting them to pinMode INPUT as the ATMEGA8U2 TX/RX are hardwired to these RX and TX pins on the Uno board.  When PIN0 (RX on the Arduino board socket) and PIN1 (TX on the Arduino board) are configured as INPUT, external devices can talk directly with the ATMEGA8U2 managing the USB, making the Arduino UART transparent.

If your project is a pure USB MIDI controller, simply setup serial to 31250 bauds in your sketch, to receive and send from/to MIDI application on the host side.  In that configuration, you can still have an external MIDI-OUT jack connected to TX, but UART RX will be dedicated to USB.  You can eventually use the SoftwareSerial library to get a MIDI IN on another pin, if your MIDI traffic is light.

                                                                       ATMEGA 328P
           USB                           ATMEGA8U2                     UART ACTIVE
       -------------         ------------------------------          --------------
       IN Endpoint  o<-----o | USBOUT  (usbMidiKliK )  RX | o<-----o |     (TX)   | ----->  MIDI OUT   
       OUT Endpoint o----->o | USBIN   ( firmware   )  TX | o----->o |     (RX)   | X NOT POSSIBLE <-----  MIDI IN


TTL/Serial MIDI IN and MIDI OUT conversion schematics can be found easily on the web.

## Changing the device ProductStringName with a USB MIDIKLIK internal SYSEX

The last version (V1.1) allows to change the USB device ProductStringName via a SYSEX. The new name is saved in the ATMEGA8U EEPROM, so it persists even after powering off the Arduino.   The message structure is the following :

       F0 <USB MidiKlik sysex header = 0x77 0x77 0x77> <sysex function id = 0x0b> <USB Midi Product name > F7

Only Serial is parsed (but USB will be in a next version), so you must send the SYSEX from an Arduino sketch.  
If you prefer to use a tool like MIDI-OX and you have MIDI IN/OUT jacks:
- connect the Arduino board with the USBMidiKliK firmware to USB (default MIDI mode)
- connect the MIDIOUT JACK to the MIDI IN JACK
- Open MIDI-OX and connect the USBMidiKliK in the MIDI output device dialog box
- Open the SysEx windows in the "View" menu
- Enter the SYSEX msg in the command window and click on "Send Sysex" in the "CommandWindow" menu.  
- Unplug/plug the USB cable or send a Midi USB HardReset sysex (see below)
- Quit and reopen MIDI-OX and you should see a new name in the MIDI devices dialog box.

The following SYSEX sent from an Arduino sketch will change the name of the MIDI interface to "USB MidiKliK" :

       F0 77 77 77 0B 55 53 42 20 4D 69 64 69 4B 6C 69 4B F7

The product name is limited to 30 characters max, non accentuated (ascii code between 0 and 0x7F).

Example code you can use in your Arduino sketch :

    // NB : Setting Product string will reboot the ATMEGA8U and the Arduino

      // Send SYSEX Message # 0B
      Serial.write( 0xF0 );
      Serial.write( 0x77 );
      Serial.write( 0x77 );
      Serial.write( 0x77 );
      Serial.write( 0x0B );

      // Important : do not use accentuated characters to avoid 8 bits values
      char * ProductString = "USB MIDI Demo";

      for (uint8_t i=0; *(ProductString + i ) !=0 ; i++ ) {
         Serial.write(* ( ProductString + i ) );
      }

      Serial.write( 0xF7 );     

## Changing the USB VendorID and ProductID with a USB MIDIKLIK internal SYSEX

In the same way, you can also change the USB Vendor and Product Ids with a SYSEX. They are also saved in the ATMEGA8U EEPROM after an update and persist after power off. The sysex message structure is the following :

    F0 77 77 77 <func id = 0x0C> <n1n2n3n4 = Vendor Id nibbles> <n1n2n3n4 = Product Id nibbles> F7

As MIDI data are 7 bits bytes, we must use a special encoding, to handle VendorId and ProductID beeing 16 bits values.  To stay light, and because the message is very short, 2 x 16 bits values, the encoding will consists in sending each nibble (4 bits) serialized in a bytes train. For example sending VendorID and ProductID 0X8F12 0X9067 will be encoded as :

      0x08 0XF 0x1 0x2  0X9 0X0 0X6 0X7

so the complete SYSEX message will be :

      F0 77 77 77 0C 08 0F 01 02 09 00 06 07 F7

Example of an Arduino code you can use in a sketch :

          void setVendorProductIds(uint16_t vendorID,uint16_t productID) {

                Serial.begin(31250);

                // Send SYSEX Message # 0C
                Serial.write( 0xF0 );
                Serial.write( 0x77 );
                Serial.write( 0x77 );
                Serial.write( 0x77 );
                Serial.write( 0x0C );

                Serial.write( (vendorID & 0xF000)  >> 12 );
                Serial.write( (vendorID & 0x0F00)  >> 8  );
                Serial.write( (vendorID & 0x00F0)  >> 4  );
                Serial.write(  vendorID & 0x000F );

                Serial.write( (productID & 0xF000)  >> 12 );
                Serial.write( (productID & 0x0F00)  >> 8  );
                Serial.write( (productID & 0x00F0)  >> 4  );
                Serial.write(  productID & 0x000F );

                Serial.write( 0xF7 );        
                resetMidiUSB();
          }

## USB Midi temporary boot in serial mode (0x09)

If you need to update the firmware without positionnig a jumper, you can you cand send this sysex that will reboot the device in serial mode, until the next boot.

       F0 77 77 77 <sysex function id = 0x09> F7

## USB Midi hard reset with a USB MIDIKLIK internal SYSEX (0x0A)

To avoid unplugging the USB cable, you cand send this sysex that will do an harware reset programatically.  The full Arduino board will be resetted (ATMEGA8U2 USBMidiKliK firmware + ATMEGA328P and sketch firmware).
The sysex message structure is the following :

       F0 77 77 77 <sysex function id = 0x0A> F7

For example to set ProductString, VendorID, Product ID and restart the USB Midi interface with new data, send the following sysex :

      F0 77 77 77 0B 55 53 42 20 4D 69 64 69 4B 6C 69 4B F7
      F0 77 77 77 0C 08 0F 01 02 09 00 06 07 F7
      F0 77 77 77 0A F7
