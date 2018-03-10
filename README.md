# USBMidiKliK
A robust USB MIDI Arduino firmware, with a dual bootloader, based on the last version of the LUFA library.

As other project, like HIDUINO, or MOCOLUFA (thanks to them for inspiration), USBMIDIKLIK allows your Arduino board to become a very reliable MIDI IN/OUT USB interface.  

The USBMIDIKLIK firmware is uploaded in the ATMEGA8U2 chip managing the USB, and changes the default USB serial descriptors to the MIDI ones. 

For more convenience when updates are needed, a "dual mode" is embedded, allowing to switch back to the USB serial : when the PB2/MOSI pin of the ATMEGA8U2 is connected to ground, the Arduino is a classical one again, and you can change and upload a new firmware in the ATMEGA328P (UNO) with the standard IDE.

In MIDI mode, Serial directions are the following :

       USB                             ATMEGA8U2                           ATMEGA 328P
       -------------         ------------------------------          ------------------------
       IN Endpoint  o<-----o | USBOUT  (usbMidiKliK )  RX | o<-----o |  (TX) pinMode(INPUT) | o<-----  MIDI IN
       OUT Endpoint o----->o | USBIN   ( firmware   )  TX | o----->o |  (RX) pinMode(INPUT) | o----->  MIDI OUT   

If you need to use external MIDI IN/OUT (with DIN jacks), the RX/TX on the ATMEGA328P must no be crossed, as the ATMEGA8U2 is directly connected to the RX and TX pins.  When PIN0 (RX on the Arduino board) and PIN1 (TX on the Arduino board) are configured as INPUT, we can talk directly with the ATMEGA8U2 managing the USB, making the Arduino transparent.

TTL/Serial MIDI IN and MIDI OUT conversion schematics can be found easily on the web.
  
## Changing the device ProductStringName with a USB MIDIKLIK internal SYSEX

The last version (V1.1) allows to change the USB device ProductStringName via a SYSEX. The new name is saved in the ATMEGEA8U EEPROM, so it persists even after powering off the Arduino.   The message structure is the following :

       F0 <USB MidiKlik sysex header = 0x77 0x77 0x77> <sysex function id = 0x 0b> <USB Midi Product name > F7

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



