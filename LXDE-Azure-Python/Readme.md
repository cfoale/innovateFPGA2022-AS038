The Linux LXDE root folder contains azure.sh to activate Azure IoT monitoring of DE10-Nano upset detection and forwarding to the Azure cloud.
For local display of FPGA DSP result detection, ./hps_adc_dsp is executed with appropriate command line options.  For immediate detection and display of
an upset on LXDE, ./plot_dsp is executed, which calls ./hps_adc_dsp and waits for an upset to be detected.  The output from the FPGA FIFO is displayed using gnuplot commands in plotcommands_dsp

./hps_adc_corr and ./plot_corr were preliminary routines used to display FPGA FIFO data after pressing a pushbutton, for development of the target16ms.dat file used by ./hps_adc_dsp during HPS DSP evaluations.

azure.sh calls modules/RfsModule/overlay/cmf_overlay.sh to load the FPGA soc-system.rbf for the DE10-Nano.  This file was previously transfered from Quartus output, after conversion.  At the end of the script python3.7 runs modules/RfsModule/main1.py, Azure IoT monitoring.  The status of the FPGA DSP upset detections is read by Python at the Linux address map of the FPGA byte FPGA_DSP_BYTE_ADDR = 0x00000010, and the content '123' = 'SAFE: CONDITIONS' or otherwise 'WARNING': 'LOC IMMINENT!' and updated to the Azure Cloud every second.




