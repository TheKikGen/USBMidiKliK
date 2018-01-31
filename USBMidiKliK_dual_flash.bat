rem cd "c:\Program Files (x86)\Arduino/hardware\tools\avr\bin"
avrdude.exe  -c usbasp -P usb -b 19200 -p m16u2  -U flash:w:USBMidiKliK_dual.hex:i
rem avrdude -p at90usb82  -F -P usb -c usbasp -U flash:w:USBMidiKliK_dual.hex -U lfuse:w:0xFF:m -U hfuse:w:0xD9:m -U efuse:w:0xF4:m -U lock:w:0x0F:m 

pause
