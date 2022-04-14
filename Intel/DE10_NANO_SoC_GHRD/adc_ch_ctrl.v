//adc_ch_ctrl.v to adc_ltc2308.v
module adc_ch_ctrl(
	adc_clk,
	SW,
	measure_dataread,
	measure_done,//from adc_ltc2308
	fifo_usedw,//from fifo, MUST BE THE SAME SIZE AS DEPTH, eg EXPONENT-1
	fifo_wrfull,//from fifo
	hps_rdrq, //from hps
	measure_dataread_sw,//to LED
	measure_start,// to adc_ltc2308
	measure_fifo_ch,//to adc_ltc2308
	adc_fifo_write_rq,//to adc_fifo
	adc_fifo_read_rq,//to adc_fifo
	adc_ctrl_state //to hps_read_fifo
);

parameter FIFO_WORDS = 256;
parameter EXPONENT = 8;

	input wire adc_clk;
	input wire [3:0] SW;
	input wire [11:0] measure_dataread;
	input wire measure_done;//from adc_ltc2308
	input wire [EXPONENT-1:0] fifo_usedw;//from fifo, MUST BE THE SAME SIZE AS DEPTH, eg EXPONENT-1
	input wire fifo_wrfull;
	input wire hps_rdrq; //from hps
	output  reg [11:0] measure_dataread_sw;//to LED
	output reg measure_start;// to adc_ltc2308
	output reg [2:0] measure_fifo_ch;//to adc_ltc2308
	output reg adc_fifo_write_rq;//to adc_fifo
	output reg adc_fifo_read_rq;//to adc_fifo
	output wire [2:0] adc_ctrl_state; //to hps_read_fifo
	
reg 	measure_fifo_start = 1;//TBD to repeat //this is a requred part of the adc_reset_n
reg 	[2:0] measure_done_fifo_ch = 3'b111;//the channel being converted is one behind the one being set via SDI to the ltc2308, after conversion

// Declare states
reg	[2:0] state;//= 3'b0;
parameter START = 0, WRITE = 1, MAKEROOM = 2, WAIT= 3, DONE = 4, READ = 5;
assign adc_ctrl_state = state;
////////////////////////////////////
// create triggle message: adc_reset_n
reg pre_measure_fifo_start=0;
always @ (posedge adc_clk)	
begin
	pre_measure_fifo_start <= measure_fifo_start;
end

wire adc_reset_n;
assign adc_reset_n = (~pre_measure_fifo_start & measure_fifo_start )?1'b0:1'b1;
// control measure_start 
reg [11:0] measure_count=0;
reg config_first=1'b1;//this is required to get the adc through one measurement, with the SDI configured
reg wait_measure_done=0;
reg [2:0] delaycount=0;

always @ (posedge adc_clk or negedge adc_reset_n ) 
begin
	if (~adc_reset_n) //happens when measure_fifo_start 
	begin					//initial reset
		measure_start <= 1'b0;
		config_first <= 1'b1;
		measure_count <= 0;
		wait_measure_done <= 1'b0;
		measure_fifo_ch <= 3'b000; //this is the channel being requested to be converted during the next conversion
		measure_done_fifo_ch <= 3'b111;//the channel being converted is one behind the one being set via SDI to the ltc2308, after conversion
		adc_fifo_write_rq <= 1'b0;
		adc_fifo_read_rq <= 1'b0;
		delaycount <= 3'b0;
		state <= START;
	end
	else if (state==START)
	begin
				measure_start <= 1'b1;
				wait_measure_done <= 1'b1;
				adc_fifo_write_rq <= 1'b0;
				adc_fifo_read_rq <= 1'b0;
				state <= WAIT;
	end
	else if(state==WRITE)
	begin
				//fifo control - read out to keep just full
				if(fifo_wrfull)//FIFO_WORDS-1
					state <= MAKEROOM;
				else
				begin
					adc_fifo_write_rq <= 1'b1;
					state <= START;
				end
	end
	else if(state==MAKEROOM)
		begin					
				if(fifo_wrfull ) //room was made
				begin
					adc_fifo_read_rq <= 1'b1;	//keeps fifo just full, without going to READ state
				end
				else
				begin
					state <= WRITE;
					adc_fifo_read_rq <= 1'b0;
				end
		end
		
	else if(state==WAIT)
			//while adc_ltc2308 is ticking
			begin
				measure_start <= 1'b0;
				if (measure_done & wait_measure_done)          //signal that measure is complete
					state <= DONE;
			end
			
	else if(state==DONE)
		 	begin	
		  		if(config_first)
				begin
					config_first <= 1'b0; //we have the SDI loaded with channel 0 request
					state <= START;//get on with with the correct SDI loaded, skip the WRITE
					measure_fifo_ch <= measure_fifo_ch + 1; //target the next channel after this current conversion
					measure_done_fifo_ch <= measure_done_fifo_ch + 1;
				end
				else
				begin
					//save read data to the LED's if SW channel and increment channel			
					if (SW[2:0] == measure_done_fifo_ch)
						measure_dataread_sw <= measure_dataread;//

					measure_count <= measure_count + 1;//so far, not necessary
					wait_measure_done <= 1'b0; 

					//has hps_read_fifo requested read?  It needs to be present for as long as up to 7 measurements, to begin
					if(hps_rdrq && (measure_done_fifo_ch==3'b000)) //ch 0 has not been written at this point, the last being 7
						begin
						adc_fifo_read_rq <= 1'b1;	//keep this high while the read_clk clocks out the FIFO_WORDS via hps_read_fifo
						state <= READ;
						end
					else
						begin
						state <= WRITE;
						measure_fifo_ch <= measure_fifo_ch + 1; //target the next channel
						measure_done_fifo_ch <= measure_done_fifo_ch + 1;//increment what we have done
						end
				end
			end
			
	else if(state==READ)			
	//hps trigger tbd
			begin
				if(!hps_rdrq) //when hps_rdrq is finally set low, this resets operation to START, but with 1st config reset
					begin
					state <= START; 
					config_first <= 1'b1;
					adc_fifo_read_rq  <= 1'b0;
					measure_fifo_ch <= 3'b000; //this is the channel being requested to be converted during the next conversion
					measure_done_fifo_ch <= 3'b111;//the channel being converted is one behind the one being set via SDI to the ltc2308, after conversion
					end
			end
end		
endmodule
