WiringPI2

Fork of BPI-SINOVOIP/BPI-WiringPi2:master
with adapations for Banana PI M2 Zero board
NOTE: ONLY TESTED WITH M2-ZERO, may NOT WORK with other boards!

Bug fixes:
- 'gpio readall' is showing the correct board type and correct numbers for BCM pins.
- setting pull-up/pull-down on ports is working ( gpio & library call)

Add on:
- new command 'gpio readallbpi' gives BCM port numbers internally used by M2-ZERO (for information only)

remaining bugs:
- wiringPiISR not implemented

INSTALLATION
-create board.sh file
sudo su
mkdir -p /var/lib/bananapi
echo "BOARD=bpi-m2z" > /var/lib/bananapi/board.sh
echo "BOARD_AUTO=bpi-m2z" >> /var/lib/bananapi/board.sh
echo "BOARD_OLD=bpi-m64" >> /var/lib/bananapi/board.sh

- clone git repository
git clone https://github.com/bontango/BPI-WiringPi2.git

- build & install
cd BPI-WiringPi2
./build
