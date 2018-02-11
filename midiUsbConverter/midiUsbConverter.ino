/**********************************************************************************
  
  KIKGEN MIDI USB 
  A MIDI USB RELIABLE INTERFACE

  USB-MIDI simple converter Sketch for Arduino


       USB                           ATMEGA8U2                    ATMEGA 328P
  --------------         ------------------------------         ---------------
  IN Endpoint  o<--------o USBOUT | usbMidiKliK |  RX o<--------o (TX) MIDI IN
  OUT Endpoint o-------->o USBIN  |  firmware   |  TX o-------->o (RX) MIDI OUT   
  
  The RX/TX on the Arduino must no be crossed, as the ATMEGA8U2 is directly connected
  to the RX and TX pins.  When PIN0 (RX) and PIN1 (TX) are configured as INPUT, we
  can talk directly with the ATMEGA8U2 managing the USB, making the Arduino transparent.
  
************************************************************************************/

void setup() {

  // Make Arduino transparent for serial communications from and to USB
  pinMode(0,INPUT); // Arduino RX - ATMEGA8U2 TX
  pinMode(1,INPUT); // Arduino TX - ATMEGA8U2 RX


}

void loop() {
  
}

