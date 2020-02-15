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

# How to compile your source file (e.g. demo.cpp) and link with shared library
g++ demo.cpp -lhexabitz -I/usr/include/hexabitz

# How to setup for the first time
sudo apt-get update
udo apt-get upgrade
sudo apt-get install build-essential cmake
sudo apt-get install cpack
cd ~
git clone https://github.com/HexabitzPlatform/HF1R0x-Firmware.git
cd HF1R0x-Firmware
git checkout master
rm -rf build
mkdir build
cd build
cmake ..


# How to compile the repository
make
cpack

# How to run the executable
./hexabitz-demo

# Update Firmware using CLI
/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI -c port=/dev/ttyUSB0 br=57600

/usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI -c port=/dev/ttyUSB0 br=57600 -w H08R6.hex 0x08000000



