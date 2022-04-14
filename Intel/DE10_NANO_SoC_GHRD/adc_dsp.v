//adc_dsp.v
module adc_dsp(
		clk,
		measure_dataread,
		measure_fifo_ch,
		adc_fifo_write_rq,//when high, measure_dataread is good
		b0, b1, b2, b3, b4, b5, b6, b7,//baseline values for subtraction, b2, b3 channels not included in dsp
		t0,t1,t2,t3,t4,  //target correlation values
		cc, //cross-correlation
		hps_dsp_byte,
		threshold,
		dsp_detected,
		triggerP
); 
 
//`define SIM  //this is required in each module that uses it, in modelsim, but not Quartus
input clk; 
input  [11:0] measure_dataread; //the current adc value measured
input [2:0] measure_fifo_ch; //the next channel to be measured
input adc_fifo_write_rq;//when high, measure_dataread is good

input  [17:0] b0;//baseline
input  [18:0] b1;//baseline
input  [17:0] b2;//baseline
input  [18:0] b3;//baseline
input  [17:0] b4;//baseline
input  [18:0] b5;//baseline
input  [17:0] b6;//baseline
input  [18:0] b7;//baseline

input signed [17:0] t0;
input signed [17:0] t1;
input signed [17:0] t2;
input signed [17:0] t3;
input signed [17:0] t4;

output wire signed [31:0] cc;

input hps_dsp_byte; //from hps, dsp annunciation by hps
input signed [31:0] threshold;// from hps, sets adc_dsp.v dsp annunciation threshold
output wire [7:0] dsp_detected; //set by adc_dsp logic, going to fpga_dsp_byte to Azure
output reg triggerP;//for signal tap only

reg signed [31:0] cc_out;//this could be wider..large enough to pass as two x 32 bits to Avalon
assign cc = cc_out;
reg signed [31:0] threshold_reg;
assign th = threshold_reg;

`ifdef SIM
reg [20:0] countdown = 0;
assign dsp_detected = ( (countdown < 6000) && (countdown > 5356) )?1:0;
`else
//assign dsp_detected = (((th > 0) && (cc > th ))|| ((th == 0) && (hps_dsp_byte > 0)) )?8'b1:8'b0;
assign dsp_detected = (((threshold > 0) && (cc > threshold ))|| ((threshold == 0) && (hps_dsp_byte > 0)) )?1:0;
`endif

parameter CH5_PRESSURE_THRESHOLD = 19'hc1c;//is 12 bits wide with value 0xac0h
reg  [18:0] thresholdP = CH5_PRESSURE_THRESHOLD;

reg [11:0] adc;
wire [17:0] adc18;
wire [18:0] adc19;
assign adc18 = {{6{1'b0}},adc};
assign adc19 = {{7{1'b0}},adc};

//these channels correlate with the last target index
reg signed [17:0] ch0; //even channels are 18 bits, odd channels are 19 bits for dsp
reg signed [18:0] ch1 ;
reg signed [17:0] ch2;
reg signed [18:0] ch3;
reg signed [17:0] ch4;
reg signed [18:0] ch5;
reg signed [17:0] ch6;
reg signed [18:0] ch7;

//history 1 values (penultimate target)
reg signed [17:0] ch0_1;
reg signed [18:0] ch1_1;
reg signed [17:0] ch2_1;
reg signed [18:0] ch3_1;
reg signed [17:0] ch4_1;
reg signed [18:0] ch5_1;
reg signed [17:0] ch6_1;
reg signed [18:0] ch7_1;

//history 2 values 
reg signed [17:0] ch0_2;
reg signed [18:0] ch1_2;
reg signed [17:0] ch2_2;
reg signed [18:0] ch3_2;
reg signed [17:0] ch4_2;
reg signed [18:0] ch5_2;
reg signed [17:0] ch6_2;
reg signed [18:0] ch7_2;

//history 3 values 
reg signed [17:0] ch0_3;
reg signed [18:0] ch1_3;
reg signed [17:0] ch2_3;
reg signed [18:0] ch3_3;
reg signed [17:0] ch4_3;
reg signed [18:0] ch5_3;
reg signed [17:0] ch6_3;
reg signed [18:0] ch7_3;

//history 4 values (first)
reg signed [17:0] ch0_4;
reg signed [18:0] ch1_4;
reg signed [17:0] ch2_4;
reg signed [18:0] ch3_4;
reg signed [17:0] ch4_4;
reg signed [18:0] ch5_4;
reg signed [17:0] ch6_4;
reg signed [18:0] ch7_4;

reg calculation_done = 1'b0; //triggle to prevent multiple calculation of the same variables

wire signed [36:0] corrA; 
wire signed [36:0] corrP;
wire signed [36:0] corrW;
wire signed [36:0] corrAP;
wire signed [36:0] corrWT;
//calculate history 0
assign corrA = (ch0*ch1) >>> 6;  //divide each quotient by 256 to keep from overflow
assign corrP = (ch4*ch5) >>> 6;
assign corrW = (ch6*ch7) >>> 6;
assign corrAP = (corrA*corrP) >>> 6;
assign corrWT = (corrW*t4) >>> 6;//last target index 

//history 1
wire signed [36:0] corrA_1; //
wire signed [36:0] corrP_1;
wire signed [36:0] corrW_1;
wire signed [36:0] corrAP_1;
wire signed [36:0] corrWT_1;
//calculate history 1
assign corrA_1 = (ch0_1*ch1_1) >>> 6;  //divide each quotient by 256 to keep from overflow
assign corrP_1 = (ch4_1*ch5_1) >>> 6;
assign corrW_1 = (ch6_1*ch7_1) >>> 6;
assign corrAP_1 = (corrA_1*corrP_1) >>> 6;
assign corrWT_1 = (corrW_1* t3) >>> 6;

//history 2
wire signed [36:0] corrA_2; //
wire signed [36:0] corrP_2;
wire signed [36:0] corrW_2;
wire signed [36:0] corrAP_2;
wire signed [36:0] corrWT_2;
//calculate history 2
assign corrA_2 = (ch0_2*ch1_2) >>> 6;  //divide each quotient by 256 to keep from overflow
assign corrP_2 = (ch4_2*ch5_2) >>> 6;
assign corrW_2 = (ch6_2*ch7_2) >>> 6;
assign corrAP_2 = (corrA_2*corrP_2) >>> 6;
assign corrWT_2 = (corrW_2* t2) >>> 6;

//history 3
wire signed [36:0] corrA_3; //
wire signed [36:0] corrP_3;
wire signed [36:0] corrW_3;
wire signed [36:0] corrAP_3;
wire signed [36:0] corrWT_3;
//calculate history 3
assign corrA_3 = (ch0_3*ch1_3) >>> 6;  //divide each quotient by 256 to keep from overflow
assign corrP_3 = (ch4_3*ch5_3) >>> 6;
assign corrW_3 = (ch6_3*ch7_3) >>> 6;
assign corrAP_3 = (corrA_3*corrP_3) >>> 6;
assign corrWT_3 = (corrW_3* t1) >>> 6;

//history 4
wire signed [36:0] corrA_4; //
wire signed [36:0] corrP_4;
wire signed [36:0] corrW_4;
wire signed [36:0] corrAP_4;
wire signed [36:0] corrWT_4;
//calculate history 3
assign corrA_4 = (ch0_4*ch1_4) >>> 6;  //divide each quotient by 256 to keep from overflow
assign corrP_4 = (ch4_4*ch5_4) >>> 6;
assign corrW_4 = (ch6_4*ch7_4) >>> 6;
assign corrAP_4 = (corrA_4*corrP_4) >>> 6;
assign corrWT_4 = (corrW_4* t0) >>> 6;

//total correlation
wire signed [36:0] corrAll;
wire signed [36:0] corrAll_0;
wire signed [36:0] corrAll_1;
wire signed [36:0] corrAll_2;
wire signed [36:0] corrAll_3;
wire signed [36:0] corrAll_4;

assign corrAll_0 = (corrAP  *corrWT  ) >>> 6;
assign corrAll_1 = (corrAP_1*corrWT_1) >>> 6;
assign corrAll_2 = (corrAP_2*corrWT_2) >>> 6;
assign corrAll_3 = (corrAP_3*corrWT_3) >>> 6;
assign corrAll_4 = (corrAP_4*corrWT_4) >>> 6;

always@ (posedge clk)
  begin
//update the channel registers when measure is done and not being updated
  if(adc_fifo_write_rq) //this is high for one clock only
  begin
    if(measure_fifo_ch == 3'b000)
	begin
	ch6 <= adc18 - b6;
	ch6_1 <= ch6;
	ch6_2 <= ch6_1;
	ch6_3 <= ch6_2;
	ch6_4 <= ch6_3;
	end
    else if(measure_fifo_ch == 3'b001)
	begin
	ch7 <= adc19 - b7;
	ch7_1 <= ch7;
	ch7_2 <= ch7_1;
	ch7_3 <= ch7_2;
	ch7_4 <= ch7_3;
	calculation_done <= 1'b0; //flag to move calculation result into output register
	end
    else if(measure_fifo_ch == 3'b010)
	begin
	ch0 <= adc18 - b0;
	ch0_1 <= ch0;
	ch0_2 <= ch0_1;
	ch0_3 <= ch0_2;
	ch0_4 <= ch0_3;
	end
    else if(measure_fifo_ch == 3'b011)
	begin
	ch1 <= adc19 - b1;
	ch1_1 <= ch1;
	ch1_2 <= ch1_1;
	ch1_3 <= ch1_2;
	ch1_4 <= ch1_3;
	end
    else if(measure_fifo_ch == 3'b100)
	begin
	ch2 <= adc18 - b2;
	ch2_1 <= ch2;
	ch2_2 <= ch2_1;
	ch2_3 <= ch2_2;
	ch2_4 <= ch2_3;
	end
    else if(measure_fifo_ch == 3'b101)
	begin
 	ch3 <= adc19 - b3;
	ch3_1 <= ch3;
	ch3_2 <= ch3_1;
	ch3_3 <= ch3_2;
	ch3_4 <= ch3_3;
	end
    else if(measure_fifo_ch == 3'b110)
	begin
 	ch4 <= adc18 - b4;
	ch4_1 <= ch4;
	ch4_2 <= ch4_1;
	ch4_3 <= ch4_2;
	ch4_4 <= ch4_3;
	end
    else if(measure_fifo_ch == 3'b111)
	begin
	ch5 <= adc19 - b5;
	ch5_1 <= ch5;
	ch5_2 <= ch5_1;
	ch5_3 <= ch5_2;
	ch5_4 <= ch5_3;
	//pressure check
	if(adc19 > CH5_PRESSURE_THRESHOLD)
			triggerP <= 1'b1;
		else
			triggerP <= 1'b0;
	end
  end//end of adc_fifo_write_rq

  if((measure_fifo_ch == 3'b010) && (adc_fifo_write_rq == 0) && (calculation_done ==0))
     begin
		calculation_done <= 1'b1;
		cc_out <= corrAll_0 + corrAll_1 + corrAll_2 + corrAll_3 + corrAll_4;
     end 

  adc <= measure_dataread; //lags by one clock - all we need
  //update the threshold using the reg
  threshold_reg <= threshold;
 
`ifdef SIM
  countdown <= countdown + 1'b1;
`endif

  end

endmodule
