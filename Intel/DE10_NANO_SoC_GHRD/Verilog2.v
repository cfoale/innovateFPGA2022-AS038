////////////DSP/////////////////////////////
//baseline adc values recorded, without dynamic signals
reg signed [17:0] b0 = 1563;
reg signed [18:0] b1 = 1335;
reg signed [17:0] b2 = 1650;//not significant to dsp
reg signed [18:0] b3 = 2732;//not significant to dsp
reg signed [17:0] b4 = 2706;
reg signed [18:0] b5 = 2762;
reg signed [17:0] b6 = 2232;
reg signed [18:0] b7 = 2221;
//target correlation values -11.8572, -36.6111, -220.04, -378.985, -872.428,
reg signed [17:0] t0 = -12;
reg signed [17:0] t1 = -36;
reg signed [17:0] t2 = -220;
reg signed [17:0] t3 = -379;
reg signed [17:0] t4 = -872;
//output dsp
wire signed [31:0] cc;
wire triggerP;//for signal tap testing

//from hps, initialized fixed here..
reg signed [31:0] hps_dsp_threshold_reg = 1000; //if not zero, this is used by adc_dsp to detect targets

`ifdef HPS
wire [7:0] hps_dsp_byte;//INPUT from avalon, from hps_adc_dsp.c to hold result for access by hps Python, Azure
`endif

reg [7:0] hps_dsp_byte_reg=8'b0;//updated each always 

wire [7:0] fpga_dsp_byte;//output to Python of hps_dsp_byte
assign fpga_dsp_byte = hps_dsp_byte_reg;
wire [7:0] dsp_detected; //signal from adc_dsp that there is a dsp detection either by the fpga adc_dsp logic, or from the hps_dsp_byte_reg

adc_dsp  adc_dsp_inst(
		.clk(clk),
		.measure_dataread(measure_dataread),
		.measure_fifo_ch(measure_fifo_ch),
		.adc_fifo_write_rq(adc_fifo_write_rq),//when high, measure_dataread is good
		.b0(b0),.b1(b1),.b2(b2),.b3(b3),.b4(b4),.b5(b5),.b6(b6),.b7(b7),
		.t0(t0),.t1(t1),.t2(t2),.t3(t3),.t4(t4),
		.cc(cc),
		.hps_dsp_byte(hps_dsp_byte_reg),
		.threshold(hps_dsp_threshold_reg),
		.dsp_detected(dsp_detected),//connect the wire fpga_dsp_byte so it can be read by python Azure
		.triggerP(triggerP)
		);

reg [11:0] fifo_readout=0; //accessed by GHRD
reg [15:0] fifo_usedw32; //accessed by GHRD
reg [PAD-1:0] pad = 0;
reg [20:0] dsp_detected_countdown=0; //max 1,048,576, latches fpga_dsp_byte

always @(posedge clk) 
begin		
`ifdef SIM
	if( ~SW[3])
		led_reg <= measure_dataread_sw[7:0];
	else
		led_reg <= hps_word[7:0];
`else
	if( ~SW[3])
		led_reg <= measure_dataread_sw[11:4];
	else
		led_reg <= hps_word[7:0];
`endif

	if(hps_dsp_threshold_reg > 0) //using fpga dsp
		begin
		if(dsp_detected > 0) //could be quite a few clocks
			begin  
			dsp_detected_countdown <= 390000; //1s about 2.56us/clock
			hps_dsp_byte_reg <= 8'b1; //sets fpga_dsp_byte output to azure
			end
			
		if(dsp_detected_countdown > 0)
			dsp_detected_countdown <= dsp_detected_countdown - 1'b1;//delay before resetting hps_dsp_byte_reg
		else
			hps_dsp_byte_reg <= 8'b0; //sets fpga_dsp_byte output to azure
		end
`ifdef SIM
	else
		hps_dsp_byte_reg <=0;//connect the wire from hps to the reg, so it can be read by python Azure
`else

	`ifdef HPS
	else
		hps_dsp_byte_reg <=hps_dsp_byte;//connect the wire from hps to the reg, so it can be read by python Azure
	`else
		hps_dsp_byte_reg <=0;
	`endif
		
`endif

	
	fifo_usedw32 <= {pad, fifo_usedw};//[EXPONENT-1:0] fifo_usedw //hps Avalon exchange	
end

//	