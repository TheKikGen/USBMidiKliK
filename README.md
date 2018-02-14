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

The RX/TX on the ATMEGA328P must no be crossed, as the ATMEGA8U2 is directly connected to the RX and TX pins.  When PIN0 (RX on the Arduino board) and PIN1 (TX on the Arduino board) are configured as INPUT, we can talk directly with the ATMEGA8U2 managing the USB, making the Arduino transparent.

TTL/Serial MIDI IN and MIDI OUT conversion schematics can be found easily on the web.

  
