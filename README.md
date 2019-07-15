# USBMidiKliK
A robust USB MIDI Arduino firmware, with a dual bootloader, based on the last version of the LUFA library.

As other project, like HIDUINO, or MOCOLUFA (thanks to them for inspiration), USBMIDIKLIK allows your Arduino board to become a very reliable MIDI IN/OUT USB interface.  Despite the very good work done on these projects, i was facing some issues...
An heavy MIDI traffic (sysex...) was blocking the serial, and some MIDI messages were purely ignored by the parser, like the song pointer position for example. More, these projects rely on a quite old version of the LUFA library.

USBMidiKliK uses interrupts and ring buffers to ensure that (fast) USB to (slow) midi transfers are reliable, plus a "more transparent as possible" midi parser. MIDI product device name is integrated in the makefile, and can also be modified by sysex, or a configuration menu...so easy to change.

This firmware is uploaded in the ATMEGA8U2 chip managing the USB, and changes the default USB serial descriptors to the MIDI ones.
For more convenience when updates are needed, a "dual mode" is embedded, allowing to switch back to the USB serial : when the PB2/MOSI pin of the ATMEGA8U2 is connected to ground, the Arduino is a classical one again, and you can change and upload a new firmware in the ATMEGA328P (UNO) with the standard Arduino IDE.

<img border="0" src="https://image.tubefr.com/upload/4/28/428246e5f47d82268003693e084d05e2.jpg"  />

Obviously, it is also possible to upload this firmware to other members of the Arduino family, like the Arduino Micro for example.

With a Uno, in a "MIDI USB converter" mode, data flow are the following :

				--------------------------- ARDUINO UNO -------------------
                                                                           ATMEGA 328P
           USB                          ATMEGA8U2                        UART NOT ACTIVE
       -------------         ------------------------------          ------------------------
       IN Endpoint  o<-----o | USBOUT  (usbMidiKliK )  RX | o<-----o |  (TX) pinMode(INPUT) | o<-----  MIDI IN
       OUT Endpoint o----->o | USBIN   ( firmware   )  TX | o----->o |  (RX) pinMode(INPUT) | o----->  MIDI OUT   

If you need USB to talk with external MIDI IN/OUT (with DIN jacks), the RX/TX on the ATMEGA328P must no be crossed, as the ATMEGA8U2 TX/RX are hardwired to these RX and TX pins on the Uno board.  When PIN0 (RX on the Arduino board socket) and PIN1 (TX on the Arduino board) are configured as INPUT, external devices can talk directly with the ATMEGA8U2 managing the USB, making the Arduino UART transparent.

If your project is a pure USB MIDI controller, simply setup serial to 31250 bauds in your sketch, to receive and send from/to MIDI application on the host side.  In that configuration, you can still have an external MIDI-OUT jack connected to TX, but UART RX will be dedicated to USB.  You can eventually use the SoftwareSerial library to get a MIDI IN on another pin, if your MIDI traffic is light.

                                                                       ATMEGA 328P
           USB                           ATMEGA8U2                     UART ACTIVE
       -------------         ------------------------------          --------------
       IN Endpoint  o<-----o | USBOUT  (usbMidiKliK )  RX | o<-----o |     (TX)   | o----->  MIDI OUT   
       OUT Endpoint o----->o | USBIN   ( firmware   )  TX | o----->o |     (RX)   | X NOT POSSIBLE <-----  MIDI IN


TTL/Serial MIDI IN and MIDI OUT conversion schematics can be found easily on the web.

# System Exclusive messages

Sysex have the following format :

    F0 <USB MidiKlik sysex header = 0x77 0x77 0x77> <sysex function id > <data > F7

## Serial Configuration menu Bootmode (function 0x08)

This sysex enables the configuration menu accessible from the USB serial. Immediatly after sending this sequence, the interface reboots in CDC serial COM mode, allowing you to open a terminal to configure easily USBMIDIKLIK.

     F0 77 77 77 08 F7

The following menu should appear after pressing ENTER :

     USBMIDIKliK                                                                     
     (c)TheKikGen Labs                                                               

     0.Show current settings                                                         
     1.Reload settings                                                               
     2.Product string                                                                
     3.VID - PID                                                                     
     4.Channel mapping                                                               
     5.Default channel mapping                                                       
     a.Arduino mode                                                                  
     s.Save & quit                                                                   
     x.Abort                                                                         
     =>s  

## Temporary boot in Arduino serial mode (0x09)

If you need to update the Arduino firmware without positionnig a jumper, you can you cand send this sysex that will reboot the device in serial mode, until the next boot.

     F0 77 77 77 09 F7

## Hardware reset (0x0A)

To avoid unplugging the USB cable, you cand send this sysex that will do an harware reset programatically.  The full Arduino board will be resetted (ATMEGA8U2 USBMidiKliK firmware + ATMEGA328P and sketch firmware).
The sysex message structure is the following :

       F0 77 77 77 0A F7

For example to set ProductString, VendorID, Product ID and restart the USB Midi interface with new data, send the following sysex :

      F0 77 77 77 0B 55 53 42 20 4D 69 64 69 4B 6C 69 4B F7
      F0 77 77 77 0C 08 0F 01 02 09 00 06 07 F7
      F0 77 77 77 0A F7

## Changing the device ProductStringName (0X0B)

It is possible to change the USB device ProductStringName via a SYSEX or from the configuration menu. The new name is saved in the ATMEGA8U EEPROM, so it persists even after powering off the Arduino.   The message structure is the following :

       F0 <USB MidiKlik sysex header = 0x77 0x77 0x77> <sysex function id = 0x0b> <USB Midi Product name > F7

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

## Changing the USB VendorID and ProductID (0x0C)

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
## Define a new midi channel mapping (0xD)

It is possible to remap a midi channel to one or many other channels, when you need to change, for example, static midi channels in an equipment.  The sysex message structure is the following :

     F0 77 77 77 <sysex function id = 0x0D> <Midi IN channel> <n bytes Midi OUT targets> F7

or

    F0 77 77 77 <sysex function id = 0x0D> <Midi IN channel> <MUTE = 0x00 | DEFAULT = 0x7F> F7

Midi IN and Targets midi OUT channels are passed as bytes from 1 to 16 (0x01 to 0x10).
You can pass a variable number of MIDI OUT channels, but 16 as a maximum.

Passing 0x00 immediatly after the MIDI IN byte will "mute" the channel.
Passing 0x7F will reset all the mapping to default.  Default is 1=>1, 2=2,....16=>16

For example, to map the midi channel 2 to channel 2,5 and 16,
map the channel 3 to channel 4, and mute the channel 16, send the SYSEX MESSAGES :

		F0 77 77 77 0D 02 02 05 10 F7
		F0 77 77 77 0D 03 04 F7
		F0 77 77 77 0D 10 00 F7
