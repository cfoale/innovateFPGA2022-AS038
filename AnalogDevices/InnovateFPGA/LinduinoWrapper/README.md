# LinduinoWrapper

4/14/2022 Changes by Colin Michael Foale for InnovateFPGA
file LinduinoWrapper/Arduino/SpiWrapper.cpp was changed:
Method bool SpiWrapper::transferWords() 
It was modified to reverse endian assumption of the Linux LDE SPI device driver.


Wrapper between Linduino and Embedded Linux

## Overview ##

This code provides support for using the existing Linduino drivers on Embedded Linux systems. Several FPGA SoC platforms - such as the Terasic DE10-Nano - have the LTC 14-pin connector which allows connecting a large variety of LTC Demo Circuit boards. With the help of this wrapper, the original Linduino drivers can be included in Linux projects, ideally with no modifications, or with a minimum effort of integration and adaptation. In the case of OS images with no particular driver support at kernel level, using just the required drivers at userspace is one of the few alternatives.

The example for the DC934A board makes use only of unmodified driver files (LTC2422.cpp, LTC2422.h, LTC2607.cpp, and LTC2607.h). 

Following a similar model, the 130+ Linduino drivers can be ported from Arduino/Linduino to Linux projects, one use case scenario being the InnovateFPGA Design Contest.

A total of three repositories need to be cloned at the same folder level:

git clone https://github.com/mu1492/LinduinoWrapper.git

git clone https://github.com/altera-opensource/intel-socfpga-hwlib.git

git clone https://github.com/analogdevicesinc/Linduino.git


## How to build the example code ##
1. Change directory to the DC934A example.

cd LinduinoWrapper/Examples/DC934A

2. Build the executable

make

