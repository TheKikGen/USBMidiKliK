# USBMidiKliK
A robust USB MIDI Arduino firmware, with a dual bootloader, based on the last version of the LUFA library.

As other project, like HIDUINO, or MOCOLUFA (thanks to them for inspiration), USBMIDIKLIK allows your Arduino board to become a very reliable MIDI IN/OUT USB interface.  Despite the very good work done on these projects, i was facing some issues...
An heavy MIDI traffic was blocking the serial, and some MIDI messages were purely ignored by the parser, like the song pointer position for example... more, these projects rely on a quite old version of the LUFA library.

USBMidiKliK uses interrupts and ring buffers to ensure that (fast) USB to (slow) midi transfers are reliable, plus a "more transparent as possible" midi parser. MIDI product device name is integrated in the makefile, and can also be modified by sysex...so easy to change.

This firmware is uploaded in the ATMEGA8U2 chip managing the USB, and changes the default USB serial descriptors to the MIDI ones. 
For more convenience when updates are needed, a "dual mode" is embedded, allowing to switch back to the USB serial : when the PB2/MOSI pin of the ATMEGA8U2 is connected to ground, the Arduino is a classical one again, and you can change and upload a new firmware in the ATMEGA328P (UNO) with the standard Arduino IDE.

In "USB converter" MIDI mode, Serial directions are the following :

                                                                            ATMEGA 328P 
           USB                          ATMEGA8U2                        UART NOT ACTIVE
       -------------         ------------------------------          ------------------------
       IN Endpoint  o<-----o | USBOUT  (usbMidiKliK )  RX | o<-----o |  (TX) pinMode(INPUT) | o<-----  MIDI IN
       OUT Endpoint o----->o | USBIN   ( firmware   )  TX | o----->o |  (RX) pinMode(INPUT) | o----->  MIDI OUT   

If you need USB to talk with external MIDI IN/OUT (with DIN jacks), the RX/TX on the ATMEGA328P must no be crossed, as the ATMEGA8U2 TX/RX are hardwired to these RX and TX pins on the Uno board.  When PIN0 (RX on the Arduino board socket) and PIN1 (TX on the Arduino board) are configured as INPUT, external devices can talk directly with the ATMEGA8U2 managing the USB, making the Arduino UART transparent.

If your project is a pure USB MIDI controller, simply setup serial to 31250 bauds in your sketch, to receive and send from/to MIDI application on the host side.  In that configuration, you can still have an external MIDI-OUT jack connected to TX

                                                                        ATMEGA 328P 
           USB                           ATMEGA8U2                    UART NOT ACTIVE
       -------------         ------------------------------          --------------
       IN Endpoint  o<-----o | USBOUT  (usbMidiKliK )  RX | o<-----o |     (TX)   | o----->  MIDI OUT   
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
- Enter the SYSEX msg in the command window and click on "Send Sysex" in the "CommandWindow" menu.  The Uno Board will reboot if the command was correctly received.
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
