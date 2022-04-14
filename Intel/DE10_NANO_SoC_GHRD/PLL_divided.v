//simulate PLL, reducing frequency by inclk0 64
//tb.v input frequency is 50 Mhz from tb i.e. 20ns period, output frequency is .39Mhz
module PLL_divided(
		inclk0,
		c0
);
//`define SIM

parameter PLL_POWER = 2;//divisor =2*2^pll_power

input inclk0; // from the test bench clock
output c0; // divided  in phase

wire pll_clk;

`ifdef SIM
PLL_SIM	pll(
	.inclk0(inclk0), //50 MHz
	.c0(pll_clk)//6.25Mhz
);
`else
PLL pll(
	.refclk(inclk0), //50 MHz
	.outclk_0(pll_clk)//6.25Mhz is the lowest possible in one stage
);
`endif

reg [PLL_POWER:0]  counter = 1'b0;//combined divide by 2*2^3 = 16;
reg c0_reg = 1'b0;
assign c0 = c0_reg;

always@ (posedge pll_clk)
  begin
    counter <= counter + 1;
    if(counter == 0)
      begin
        c0_reg <= ~ c0_reg;//this divides by 2
      end
  end

endmodule
