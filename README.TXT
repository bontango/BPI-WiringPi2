WiringPI2

Fork of BPI-SINOVOIP/BPI-WiringPi2:master
with adapations for Banana PI M2 Zero board

Bug fixes:
'gpio readall' is showing the correct board type and correct numbers for BCM pins.

remaining bugs:
- setting pull-up on ports is not working
- wiringPiISR not implemented

installation
- git clone https://github.com/bontango/BPI-WiringPi2.git
- cd BPI-WiringPi2
- ./build
