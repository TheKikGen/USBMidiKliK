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

void resetMidiUSB() {

       Serial.begin(31250);

      // Send SYSEX Message # 0A
      Serial.write( 0xF0 );
      Serial.write( 0x77 );
      Serial.write( 0x77 );
      Serial.write( 0x77 );
      Serial.write( 0x0A );
      Serial.write( 0xF7 );      

}

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

void SetProductString() {

      Serial.begin(31250);
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
      resetMidiUSB();       
}


void setup() {

  // Make Arduino transparent for serial communications from and to USB
  pinMode(0,INPUT); // Arduino RX - ATMEGA8U2 TX
  pinMode(1,INPUT); // Arduino TX - ATMEGA8U2 RX

}

void loop() {
    
}

