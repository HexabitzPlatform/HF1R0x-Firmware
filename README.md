# HF1R0x-Firmware
Hexabitz Raspberry Pi 3 Interface Module Firmware

# Information Regarding Pi Hat
Must Read: https://www.raspberrypi.org/forums/viewtopic.php?t=108134
Schematics: https://github.com/HexabitzPlatform/HF1R0x-Hardware/blob/master/HF1R00/Documentation/HF1R00.pdf
Pi Hat: https://github.com/raspberrypi/hats
ID EEPROM Datasheet: https://www.onsemi.com/pub/Collateral/CAT24C32-D.PDF

# How to Compile using autotools
autoreconf --install
mkdir build
cd build
../configure
make
make install

# How to setup CMake for deb package creation
https://blog.usejournal.com/creating-debian-packages-cmake-e519a0186e87
