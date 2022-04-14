These files were used on the Linux LXDE to represent a digital twin of our drone data.  
The executable /AnalogDevices/InnovateFPGA/LinduinoWrapper/Examples/adc_dac uses the DE10-Nano SPI interface to access the DC2025A DAC.

Changes by Colin Michael Foale for InnovateFPGA
file LinduinoWrapper/Arduino/SpiWrapper.cpp were made:
Method bool SpiWrapper::transferWords() 
It was modified to reverse endian assumption of the Linux LDE SPI device driver.
