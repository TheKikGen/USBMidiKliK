cd "c:\Program Files (x86)\Arduino/hardware\tools\avr\bin"
c:avrdude.exe -C c:avrdude.conf -c usbasp -P /dev/ttyACM0 -b 19200 -p m16u2 -vvv -U flash:w:USBMidiKliK_dual.hex:i

pause
