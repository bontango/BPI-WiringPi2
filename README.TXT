WiringPI2

Fork of BPI-SINOVOIP/BPI-WiringPi2:master
with adapations for Banana PI M2 Zero board
NOTE: ONLY TESTED WITH M2-ZERO, may NOT WORK with other boards!

Bug fixes:
- 'gpio readall' is showing the correct board type and correct numbers for BCM pins.
- setting pull-up/pull-down on ports is working ( gpio & library call)
- wiringPiISR (partly) implemented, see limitations below

Add on:
- new command 'gpio readallbpi' gives BPI port numbers internally used by M2-ZERO (for information only)

Limitations:
The 'wiringPiISR' library call ( and 'gpio wfi ...') DO NOT WORK with the wPi ports: 5,10,11,12,13,14,26,27
These are the BPI port 64 and above, which you can identify by issuing 'gpio readallbpi'

remaining bugs:
There are for sure still bugs, I did not test all wiringPI features!
Please let me know in case you find another bug ...

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
