WiringPI2

Fork of BPI-SINOVOIP/BPI-WiringPi2:master
with adapations for Banana PI M2 Zero board

Bug fixes:
'gpio readall' is showing the correct board type and correct numbers for BCM pins.

remaining bugs:
- setting pull-up on ports is not working
- wiringPiISR not implemented

installation
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
