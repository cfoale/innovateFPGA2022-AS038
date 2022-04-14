//hps_read_fifo.v
module hps_read_fifo(
	clk ,
	fifo_q ,//from adc_fifo
	KEY,
	SW3,
	fifo_empty,//from adc_fifo
	hps_read_rq,//from hps
	hps_read_clk,//from hps
	fifo_usedw, //from adc_fifo EXPONENT-1
	adc_ctrl_state, //from adc_ch_ctrl
	fifo_wrfull,
	hps_rdrq ,//to adc_ch_ctrl
	hps_word ,//to hps
	hps_read_status, //to hps
	read_clk //to fifo
); 

parameter FIFO_WORDS = 256;
parameter EXPONENT = 8;

	input wire clk ;
	input  wire [11:0] fifo_q;//from adc_fifo
	input wire [1:0] KEY;
	input wire SW3;
	input fifo_empty;//from adc_fifo
	input wire hps_read_rq;//from hp
	input wire hps_read_clk;
	input wire [EXPONENT-1:0] fifo_usedw; //from adc_fifo EXPONENT-1
	input wire [2:0] adc_ctrl_state; //from adc_ch_ctrl
	input wire fifo_wrfull;
	output  wire hps_rdrq ;//to adc_ch_ctrl
	output  reg [15:0] hps_word;//to hps
	output reg hps_read_status; //to hps
	output wire read_clk; //to fifo

reg [EXPONENT-1:0] previous_usedw;
reg enable_readout = 1'b0;//goes high when Key[1] is pressed,latched
reg  read_clk_reg=0;
reg [1:0] fast_read_clk = 2'b0;
reg [3:0] slow_read_clk = 4'b0;
reg hps_rdrq_reg = 1'b0;//to adc_ch_ctrl via wire
assign hps_rdrq = hps_rdrq_reg;

// Declare states
parameter START = 0, WRITE = 1, MAKEROOM = 2, WAIT= 3, DONE = 4, READ = 5;
assign read_clk = read_clk_reg;

//definge the read clock, slower than half the adc clock
always @(posedge clk)
begin
	fast_read_clk <= fast_read_clk + 1;
	slow_read_clk <= slow_read_clk + 1;
	if(~SW3) //automatic reading
		begin
		if((adc_ctrl_state != READ) && (fast_read_clk==2'b00) )
			read_clk_reg <= ~read_clk_reg;//1/4 adc_clk
		else if((adc_ctrl_state == READ ) && (slow_read_clk == 4'b0))
			read_clk_reg <= ~read_clk_reg;
		end
	else //manual reading
		begin
		if(adc_ctrl_state != READ && (fast_read_clk==2'b00)) //use the fast clock to get to frame end
			read_clk_reg <= ~read_clk_reg;//1/4 adc_clk
		if(adc_ctrl_state == READ ) 
			read_clk_reg <= KEY[0] & hps_read_clk; //we can use the manual clocks
		end	
end
reg enable_pending=1'b0; //set when fifo_usedw are less than FIFO_WORDS-1 (after a readout)

always @(posedge clk) 
begin		
//readout the fifo	
	if((KEY[1]==1'b0 | hps_read_rq | enable_pending) & ~enable_readout) // key is pressed or hps requests
	begin
		if(fifo_usedw == FIFO_WORDS-1 )//8'b11111101
			enable_readout <= 1'b1;
		else
			 enable_pending <= 1'b1;	
	end
	else if(enable_readout ) 	
		//read out the fifo if enable_readout, at the read clock rate
		if( !fifo_empty )
			begin
			hps_rdrq_reg <= 1'b1;//to adc_ctrl, and then adc_fifo, and kept high while monitoring
			if((adc_ctrl_state == READ) && fifo_wrfull)//(FIFO_WORDS - 1)
				begin
				hps_word <= {4'b0, fifo_q}; //convert the first word to be read out
				hps_read_status <= 1'b1;
				previous_usedw <= fifo_usedw;
				end
			else if(adc_ctrl_state == READ && (previous_usedw != fifo_usedw )) 
				begin
				 	//we have a new word
					hps_word <= {4'b0, fifo_q}; //convert 12 bit word 16 bits
					previous_usedw <= fifo_usedw;
				end
			end
		else //empty, readout complete
			begin
			hps_rdrq_reg <= 1'b0;
			enable_readout <= 1'b0;
			enable_pending <= 1'b0;
			hps_word <= {4'b0, fifo_q}; //convert last word
			previous_usedw <= fifo_usedw;
			hps_read_status <= 0;
			end
end
endmodule
