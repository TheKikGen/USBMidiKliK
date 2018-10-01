goto uno
:micro
echo Reset the Arduino micro now...
pause
avrdude -v -patmega32u4 -cavr109 -PCOM5 -b57600 -D -V -Uflash:w:./USBMidiKliK_dual.hex:i
goto end

:uno
 cd "c:\Program Files (x86)\Arduino/hardware\tools\avr\bin"
 avrdude.exe  -c usbasp -P usb -b 19200 -p m16u2  -U flash:w:USBMidiKliK_dual.hex:i 
 -U eeprom:w:USBMidiKliK_dual.eep
rem  avrdude -p at90usb82  -F -P usb -c usbasp -U flash:w:USBMidiKliK_dual.hex -U lfuse:w:0xFF:m -U hfuse:w:0xD9:m -U efuse:w:0xF4:m -U lock:w:0x0F:m 
rem  avrdude.exe  -c usbasp -P usb -b 19200 -p m16u2 -U eeprom:r:USBMidiKliK_dual_ee.bin:r

:end



pause
