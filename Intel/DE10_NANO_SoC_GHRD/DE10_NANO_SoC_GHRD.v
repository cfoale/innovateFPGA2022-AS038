//DE10_NANO_SoC_GHRD.v adapted by C.M.Foale 2022
//1/22/2022 cmf removed KEY input to hps_read_fifo.v, to disable manual PB readout of fifo, and to enable KEY input to HPS for 
//hps_adc_dsp program control
module DE10_NANO_SoC_GHRD(

    //////////// CLOCK //////////
    input               FPGA_CLK1_50,
    input               FPGA_CLK2_50,
    input               FPGA_CLK3_50,

    //////////// HDMI //////////
    inout               HDMI_I2C_SCL,
    inout               HDMI_I2C_SDA,
    inout               HDMI_I2S,
    inout               HDMI_LRCLK,
    inout               HDMI_MCLK,
    inout               HDMI_SCLK,
    output              HDMI_TX_CLK,
    output   [23: 0]    HDMI_TX_D,
    output              HDMI_TX_DE,
    output              HDMI_TX_HS,
    input               HDMI_TX_INT,
    output              HDMI_TX_VS,

    //////////// HPS //////////
    inout               HPS_CONV_USB_N,
    output   [14: 0]    HPS_DDR3_ADDR,
    output   [ 2: 0]    HPS_DDR3_BA,
    output              HPS_DDR3_CAS_N,
    output              HPS_DDR3_CK_N,
    output              HPS_DDR3_CK_P,
    output              HPS_DDR3_CKE,
    output              HPS_DDR3_CS_N,
    output   [ 3: 0]    HPS_DDR3_DM,
    inout    [31: 0]    HPS_DDR3_DQ,
    inout    [ 3: 0]    HPS_DDR3_DQS_N,
    inout    [ 3: 0]    HPS_DDR3_DQS_P,
    output              HPS_DDR3_ODT,
    output              HPS_DDR3_RAS_N,
    output              HPS_DDR3_RESET_N,
    input               HPS_DDR3_RZQ,
    output              HPS_DDR3_WE_N,
    output              HPS_ENET_GTX_CLK,
    inout               HPS_ENET_INT_N,
    output              HPS_ENET_MDC,
    inout               HPS_ENET_MDIO,
    input               HPS_ENET_RX_CLK,
    input    [ 3: 0]    HPS_ENET_RX_DATA,
    input               HPS_ENET_RX_DV,
    output   [ 3: 0]    HPS_ENET_TX_DATA,
    output              HPS_ENET_TX_EN,
    inout               HPS_GSENSOR_INT,
    inout               HPS_I2C0_SCLK,
    inout               HPS_I2C0_SDAT,
    inout               HPS_I2C1_SCLK,
    inout               HPS_I2C1_SDAT,
    inout               HPS_KEY,
    inout               HPS_LED,
    inout               HPS_LTC_GPIO,
    output              HPS_SD_CLK,
    inout               HPS_SD_CMD,
    inout    [ 3: 0]    HPS_SD_DATA,
    output              HPS_SPIM_CLK,
    input               HPS_SPIM_MISO,
    output              HPS_SPIM_MOSI,
    inout               HPS_SPIM_SS,
    input               HPS_UART_RX,
    output              HPS_UART_TX,
    input               HPS_USB_CLKOUT,
    inout    [ 7: 0]    HPS_USB_DATA,
    input               HPS_USB_DIR,
    input               HPS_USB_NXT,
    output              HPS_USB_STP,

    //////////// KEY //////////
    input    [ 1: 0]    KEY,

    //////////// LED //////////
    output   [ 7: 0]    LED,

    //////////// SW //////////
    input    [ 3: 0]    SW,
	 
	 ///Mike add///
	 input	[3:0] gpio_1_d34_d32_d30_d28, //for testing access to these GPIO's
	 input			ADC_SDO,
	 output 			ADC_CONVST,
	 output 			ADC_SDI,
	 output 			ADC_SCK
);



//=======================================================
//  REG/WIRE declarations
//=======================================================
wire hps_fpga_reset_n;
wire     [1: 0]     fpga_debounced_buttons;

wire     [6: 0]     fpga_led_internal;//it is normally assigned to LED[6:0]

wire     [2: 0]     hps_reset_req;
wire                hps_cold_reset;
wire                hps_warm_reset;
wire                hps_debug_reset;
wire     [27: 0]    stm_hw_events;
wire                fpga_clk_50;


// connection of internal logic
assign stm_hw_events = {{15{1'b0}}, SW, fpga_led_internal, fpga_debounced_buttons};//Mike - This is not something you need to change-Altera UPS specific

///////////////////////////////mike adds///////////////////////////////////
//`define SIM ////////////////////////12/20/2021 cmf for simulation////////////////////////
`define HPS

wire	[2:0] measure_fifo_ch;
parameter FIFO_WORDS = 256;//the size of the fifo buffer, room for 8x4 complete measurements
parameter EXPONENT = 8; //2**EXPONENT = FIFO_WORDS
parameter PAD = 16-EXPONENT;//to pad out fifo_usedw32
parameter DSP_BYTE_COUNTDOWN = 2*390000; //delay is x clk freq to reset fpga_dsp_byte 

wire 	[11:0] measure_dataread_sw; //this contains the measured value of the channel pointed to by the switches
wire	[11:0] measure_dataread;
wire	clk;//from the pll
wire	measure_start;
wire measure_done;

reg [7:0] led_reg=0;
assign LED = led_reg;

parameter PLL_POWER = 2;//.39mhz

wire CLOCK_50;
assign CLOCK_50 = FPGA_CLK2_50;//to look like adc_corr.v

PLL_divided	#(.PLL_POWER(PLL_POWER)) pll_divided(
	.inclk0(CLOCK_50), //50 MHz
	.c0(clk)//.39Mhz
);

wire [EXPONENT-1:0] fifo_usedw;//register must be able to index FIFO_WORDS
wire adc_fifo_read_rq ;
wire adc_fifo_write_rq;
wire fifo_wrfull;
wire [2:0] adc_ctrl_state; //between adc_ch_ctrl and hps_read_fifo

adc_ch_ctrl #(.FIFO_WORDS(FIFO_WORDS), .EXPONENT(EXPONENT))	adc_ch_ctrl_inst(
	.adc_clk(clk),
	.SW(SW),
	.measure_done(measure_done),//from adc_ltc2308
	.measure_dataread(measure_dataread),//from adc_ltc2308
	.fifo_usedw(fifo_usedw), //from adc_fifo
	.fifo_wrfull(fifo_wrfull),//from adc_fifo
	.hps_rdrq(hps_rdrq),//from hps_read_fifo
	.adc_ctrl_state(adc_ctrl_state),
	.measure_dataread_sw(measure_dataread_sw),//to LED
	.measure_start(measure_start),//to adc_ltc2308
	.measure_fifo_ch(measure_fifo_ch),// to adc_ltc2308
	.adc_fifo_write_rq(adc_fifo_write_rq),//to adc_fifo
	.adc_fifo_read_rq(adc_fifo_read_rq)//to adc_fifo
);

adc_ltc2308	adc_inst(
	.clk(clk),
	.measure_start(measure_start),//from adc_ch_ctrl
	.ADC_SDO(ADC_SDO),//from the ltc2308 pin
	.measure_ch(measure_fifo_ch),//from adc_ch_ctrl
	.measure_done(measure_done),//to adc_ch_ctrl
	.ADC_CONVST(ADC_CONVST),//to ltc2308 pin
	.ADC_SCK(ADC_SCK),//to ltc2308 pin
	.ADC_SDI(ADC_SDI),//to ltc2308 pin
	.measure_dataread(measure_dataread)//to adc_ch_ctrl
);

wire [11:0] fifo_q;
reg fifo_wr_done=0;
wire fifo_empty;
wire read_clk;

adc_fifo  adc_fifo_inst ( //12bits x 32 words is enough for 4x 8 channels
	.data ( measure_dataread ),//from adc_ltc2308
	.rdclk(read_clk),//required to be continuous
	.rdreq(adc_fifo_read_rq),//from adc_ch_ctrl which came from hps_read_fifo
	.wrclk ( clk ),
	.wrreq ( adc_fifo_write_rq ),//from adc_ch_ctrl
	.q (fifo_q),//to hps_read_fifo
	.rdempty ( fifo_empty ),//to hps_read_fifo
	.wrusedw ( fifo_usedw ),//to adc_ch_ctrl
	.wrfull(fifo_wrfull)//to adc_ch_ctrl
	//.aclr(reset)
	);
	
wire [15:0] hps_word;//to hps
`ifndef SIM
wire hps_read_status;//to hps
wire [7:0] hps_read_clk_byte;//INPUT from avalon bus (must be a wire)
wire [7:0] hps_read_rq_byte;//INPUT from avalon bus (must be a wire)
`endif
wire hps_read_rq;//to hps_read_fifo
assign hps_read_rq =  hps_read_rq_byte[0];//we only need the LSB
wire hps_read_clk;//to hps_read_fifo
assign hps_read_clk = hps_read_clk_byte[0];//we only need the LSB

//replace manual KEY readout of hps_read_fifo with a disconnected register
reg [1:0] key_reg = {1'b1,1'b1};
wire [1:0] key_reg_wire;
assign key_reg_wire = key_reg;

hps_read_fifo #(.FIFO_WORDS(FIFO_WORDS), .EXPONENT(EXPONENT)) hps_read_fifo_inst(
	.clk(clk) ,
	.fifo_q(fifo_q) ,//from adc_fifo
	.KEY(key_reg_wire),//replaced manual control of reading fifo in place of HPS with a dummy reg
	.SW3(SW[3]),//manual control of reading fifo in place of HPS
	.fifo_empty(fifo_empty),//from adc_fifo
	.hps_read_rq(hps_read_rq),//from hps
	.hps_read_clk(hps_read_clk),
	.fifo_usedw(fifo_usedw), //from adc_fifo
	.adc_ctrl_state(adc_ctrl_state), //from adc_ch_ctrl
	.fifo_wrfull(fifo_wrfull), //from fifo
	.hps_rdrq(hps_rdrq), //to adc_ch_ctrl
	.hps_word(hps_word), //to hps
	.hps_read_status(hps_read_status), //to hps
	.read_clk(read_clk)//to fifo
);

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
wire signed [31:0] cc;//from adc_dsp
reg signed [31:0] cc_out_reg; //to wire to hps

wire triggerP;//for signal tap testing

//from hps, initialized fixed here..
reg signed [31:0] hps_dsp_threshold_reg = 25000; //if not zero, this is used by adc_dsp to detect targets

`ifdef HPS
wire [7:0] hps_dsp_byte;//INPUT from avalon, from hps_adc_dsp.c to hold result for access by hps Python, Azure
wire signed [31:0] hps_dsp_threshold; //input from hps_adc_dsp.c
`endif

reg [7:0] hps_dsp_byte_reg= 0;//updated each always 

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
	   //if(hps_dsp_byte_reg > 0)
			//led_reg <= hps_dsp_byte_reg;//light up the LED's
		//else
			led_reg <= measure_dataread_sw[11:4];
	else
		led_reg <= hps_word[7:0];
`endif

	if(hps_dsp_threshold_reg > 0) //using fpga dsp
		begin
		if(dsp_detected > 0) //could be quite a few clocks
			begin  
			dsp_detected_countdown <= DSP_BYTE_COUNTDOWN; //1s about 2.56us/clock
			hps_dsp_byte_reg <= 255; //sets fpga_dsp_byte output to azure
			end
			
		if(dsp_detected_countdown > 0)
			dsp_detected_countdown <= dsp_detected_countdown - 1'b1;//delay before resetting hps_dsp_byte_reg
		else
			hps_dsp_byte_reg <= 123; //sets fpga_dsp_byte output to azure
		end
`ifdef SIM
	//else
		hps_dsp_byte_reg <=0;//connect the wire from hps to the reg, so it can be read by python Azure
`else

	`ifdef HPS
	else
	begin
		hps_dsp_byte_reg <= hps_dsp_byte;//connect the wire from hps to the reg, so it can be read by python Azure
	end
	`else
	else
	begin
		hps_dsp_byte_reg <=0;
	end
	`endif
		
`endif

	
	fifo_usedw32 <= {pad, fifo_usedw};//[EXPONENT-1:0] fifo_usedw //hps Avalon exchange
	
`ifdef HPS	
	hps_dsp_threshold_reg <= hps_dsp_threshold;//updated from hps_adc_dsp.c
`endif

	cc_out_reg	<= cc;
end

//=======================================================
//  Structural coding
//=======================================================
soc_system u0(
               //Clock&Reset
               .clk_clk(FPGA_CLK1_50),                                      //                            clk.clk
               .reset_reset_n(hps_fpga_reset_n),                            //                          reset.reset_n
               //HPS ddr3
               .memory_mem_a(HPS_DDR3_ADDR),                                //                         memory.mem_a
               .memory_mem_ba(HPS_DDR3_BA),                                 //                               .mem_ba
               .memory_mem_ck(HPS_DDR3_CK_P),                               //                               .mem_ck
               .memory_mem_ck_n(HPS_DDR3_CK_N),                             //                               .mem_ck_n
               .memory_mem_cke(HPS_DDR3_CKE),                               //                               .mem_cke
               .memory_mem_cs_n(HPS_DDR3_CS_N),                             //                               .mem_cs_n
               .memory_mem_ras_n(HPS_DDR3_RAS_N),                           //                               .mem_ras_n
               .memory_mem_cas_n(HPS_DDR3_CAS_N),                           //                               .mem_cas_n
               .memory_mem_we_n(HPS_DDR3_WE_N),                             //                               .mem_we_n
               .memory_mem_reset_n(HPS_DDR3_RESET_N),                       //                               .mem_reset_n
               .memory_mem_dq(HPS_DDR3_DQ),                                 //                               .mem_dq
               .memory_mem_dqs(HPS_DDR3_DQS_P),                             //                               .mem_dqs
               .memory_mem_dqs_n(HPS_DDR3_DQS_N),                           //                               .mem_dqs_n
               .memory_mem_odt(HPS_DDR3_ODT),                               //                               .mem_odt
               .memory_mem_dm(HPS_DDR3_DM),                                 //                               .mem_dm
               .memory_oct_rzqin(HPS_DDR3_RZQ),                             //                               .oct_rzqin
               //HPS ethernet
               .hps_0_hps_io_hps_io_emac1_inst_TX_CLK(HPS_ENET_GTX_CLK),    //                   hps_0_hps_io.hps_io_emac1_inst_TX_CLK
               .hps_0_hps_io_hps_io_emac1_inst_TXD0(HPS_ENET_TX_DATA[0]),   //                               .hps_io_emac1_inst_TXD0
               .hps_0_hps_io_hps_io_emac1_inst_TXD1(HPS_ENET_TX_DATA[1]),   //                               .hps_io_emac1_inst_TXD1
               .hps_0_hps_io_hps_io_emac1_inst_TXD2(HPS_ENET_TX_DATA[2]),   //                               .hps_io_emac1_inst_TXD2
               .hps_0_hps_io_hps_io_emac1_inst_TXD3(HPS_ENET_TX_DATA[3]),   //                               .hps_io_emac1_inst_TXD3
               .hps_0_hps_io_hps_io_emac1_inst_RXD0(HPS_ENET_RX_DATA[0]),   //                               .hps_io_emac1_inst_RXD0
               .hps_0_hps_io_hps_io_emac1_inst_MDIO(HPS_ENET_MDIO),         //                               .hps_io_emac1_inst_MDIO
               .hps_0_hps_io_hps_io_emac1_inst_MDC(HPS_ENET_MDC),           //                               .hps_io_emac1_inst_MDC
               .hps_0_hps_io_hps_io_emac1_inst_RX_CTL(HPS_ENET_RX_DV),      //                               .hps_io_emac1_inst_RX_CTL
               .hps_0_hps_io_hps_io_emac1_inst_TX_CTL(HPS_ENET_TX_EN),      //                               .hps_io_emac1_inst_TX_CTL
               .hps_0_hps_io_hps_io_emac1_inst_RX_CLK(HPS_ENET_RX_CLK),     //                               .hps_io_emac1_inst_RX_CLK
               .hps_0_hps_io_hps_io_emac1_inst_RXD1(HPS_ENET_RX_DATA[1]),   //                               .hps_io_emac1_inst_RXD1
               .hps_0_hps_io_hps_io_emac1_inst_RXD2(HPS_ENET_RX_DATA[2]),   //                               .hps_io_emac1_inst_RXD2
               .hps_0_hps_io_hps_io_emac1_inst_RXD3(HPS_ENET_RX_DATA[3]),   //                               .hps_io_emac1_inst_RXD3
               //HPS SD card
               .hps_0_hps_io_hps_io_sdio_inst_CMD(HPS_SD_CMD),              //                               .hps_io_sdio_inst_CMD
               .hps_0_hps_io_hps_io_sdio_inst_D0(HPS_SD_DATA[0]),           //                               .hps_io_sdio_inst_D0
               .hps_0_hps_io_hps_io_sdio_inst_D1(HPS_SD_DATA[1]),           //                               .hps_io_sdio_inst_D1
               .hps_0_hps_io_hps_io_sdio_inst_CLK(HPS_SD_CLK),              //                               .hps_io_sdio_inst_CLK
               .hps_0_hps_io_hps_io_sdio_inst_D2(HPS_SD_DATA[2]),           //                               .hps_io_sdio_inst_D2
               .hps_0_hps_io_hps_io_sdio_inst_D3(HPS_SD_DATA[3]),           //                               .hps_io_sdio_inst_D3
               //HPS USB
               .hps_0_hps_io_hps_io_usb1_inst_D0(HPS_USB_DATA[0]),          //                               .hps_io_usb1_inst_D0
               .hps_0_hps_io_hps_io_usb1_inst_D1(HPS_USB_DATA[1]),          //                               .hps_io_usb1_inst_D1
               .hps_0_hps_io_hps_io_usb1_inst_D2(HPS_USB_DATA[2]),          //                               .hps_io_usb1_inst_D2
               .hps_0_hps_io_hps_io_usb1_inst_D3(HPS_USB_DATA[3]),          //                               .hps_io_usb1_inst_D3
               .hps_0_hps_io_hps_io_usb1_inst_D4(HPS_USB_DATA[4]),          //                               .hps_io_usb1_inst_D4
               .hps_0_hps_io_hps_io_usb1_inst_D5(HPS_USB_DATA[5]),          //                               .hps_io_usb1_inst_D5
               .hps_0_hps_io_hps_io_usb1_inst_D6(HPS_USB_DATA[6]),          //                               .hps_io_usb1_inst_D6
               .hps_0_hps_io_hps_io_usb1_inst_D7(HPS_USB_DATA[7]),          //                               .hps_io_usb1_inst_D7
               .hps_0_hps_io_hps_io_usb1_inst_CLK(HPS_USB_CLKOUT),          //                               .hps_io_usb1_inst_CLK
               .hps_0_hps_io_hps_io_usb1_inst_STP(HPS_USB_STP),             //                               .hps_io_usb1_inst_STP
               .hps_0_hps_io_hps_io_usb1_inst_DIR(HPS_USB_DIR),             //                               .hps_io_usb1_inst_DIR
               .hps_0_hps_io_hps_io_usb1_inst_NXT(HPS_USB_NXT),             //                               .hps_io_usb1_inst_NXT
               //HPS SPI
               .hps_0_hps_io_hps_io_spim1_inst_CLK(HPS_SPIM_CLK),           //                               .hps_io_spim1_inst_CLK
               .hps_0_hps_io_hps_io_spim1_inst_MOSI(HPS_SPIM_MOSI),         //                               .hps_io_spim1_inst_MOSI
               .hps_0_hps_io_hps_io_spim1_inst_MISO(HPS_SPIM_MISO),         //                               .hps_io_spim1_inst_MISO
               .hps_0_hps_io_hps_io_spim1_inst_SS0(HPS_SPIM_SS),            //                               .hps_io_spim1_inst_SS0
               //HPS UART
               .hps_0_hps_io_hps_io_uart0_inst_RX(HPS_UART_RX),             //                               .hps_io_uart0_inst_RX
               .hps_0_hps_io_hps_io_uart0_inst_TX(HPS_UART_TX),             //                               .hps_io_uart0_inst_TX
               //HPS I2C1
               .hps_0_hps_io_hps_io_i2c0_inst_SDA(HPS_I2C0_SDAT),           //                               .hps_io_i2c0_inst_SDA
               .hps_0_hps_io_hps_io_i2c0_inst_SCL(HPS_I2C0_SCLK),           //                               .hps_io_i2c0_inst_SCL
               //HPS I2C2
               .hps_0_hps_io_hps_io_i2c1_inst_SDA(HPS_I2C1_SDAT),           //                               .hps_io_i2c1_inst_SDA
               .hps_0_hps_io_hps_io_i2c1_inst_SCL(HPS_I2C1_SCLK),           //                               .hps_io_i2c1_inst_SCL
               //GPIO
               .hps_0_hps_io_hps_io_gpio_inst_GPIO09(HPS_CONV_USB_N),       //                               .hps_io_gpio_inst_GPIO09
               .hps_0_hps_io_hps_io_gpio_inst_GPIO35(HPS_ENET_INT_N),       //                               .hps_io_gpio_inst_GPIO35
               .hps_0_hps_io_hps_io_gpio_inst_GPIO40(HPS_LTC_GPIO),         //                               .hps_io_gpio_inst_GPIO40
               .hps_0_hps_io_hps_io_gpio_inst_GPIO53(HPS_LED),              //                               .hps_io_gpio_inst_GPIO53
               .hps_0_hps_io_hps_io_gpio_inst_GPIO54(HPS_KEY),              //                               .hps_io_gpio_inst_GPIO54
               .hps_0_hps_io_hps_io_gpio_inst_GPIO61(HPS_GSENSOR_INT),      //                               .hps_io_gpio_inst_GPIO61
               //FPGA Partion
               .led_pio_external_connection_export(fpga_led_internal),      // cmf - this could be your own [6:0] wires, not fpga_led_internal...   led_pio_external_connection.export
               .dipsw_pio_external_connection_export(SW),                   //  dipsw_pio_external_connection.export
					//cmf bypass of fpga_debounced_buttons
               //.button_pio_external_connection_export(fpga_debounced_buttons),// button_pio_external_connection.export
					.button_pio_external_connection_export(KEY),// button_pio_external_connection.export
					//cmf adds
					.gpio_1_d34_d32_d30_d28_external_connection_export(gpio_1_d34_d32_d30_d28),
					.hps_word16_external_connection_export(hps_word),//hps_word_external_connection.export
					.hps_read_rq_external_connection_export(hps_read_rq_byte),         //hps_read_rq_external_connection.export
					.hps_read_status_external_connection_export(hps_read_status),  //hps_read_status_external_connection.export
					.hps_read_clk_external_connection_export(hps_read_clk_byte),
					.fifo_usedw32_external_connection_export(fifo_usedw32),  //conduit to hps of fifo_usedw
					.hps_dsp_byte_external_connection_export(hps_dsp_byte), //from hps_adc_dps.c
					.hps_dsp_threshold_external_connection_export(hps_dsp_threshold), //from hps_adc_dps.c to fpga to set dsp threshold
					.fpga_dsp_byte_external_connection_export(fpga_dsp_byte), //from fpga to Python Azure
					.hps_fifo_wrfull_external_connection_export(fifo_wrfull),  //hps_fifo_wrfull_external_connection.export
					.cc_out_external_connection_export(cc_out_reg),
	
					//end of cmf adds
               .hps_0_h2f_reset_reset_n(hps_fpga_reset_n),                  //                hps_0_h2f_reset.reset_n
               .hps_0_f2h_cold_reset_req_reset_n(~hps_cold_reset),          //       hps_0_f2h_cold_reset_req.reset_n
               .hps_0_f2h_debug_reset_req_reset_n(~hps_debug_reset),        //      hps_0_f2h_debug_reset_req.reset_n
               .hps_0_f2h_stm_hw_events_stm_hwevents(stm_hw_events),        //        hps_0_f2h_stm_hw_events.stm_hwevents
               .hps_0_f2h_warm_reset_req_reset_n(~hps_warm_reset),          //       hps_0_f2h_warm_reset_req.reset_n

           );

// Debounce logic to clean out glitches within 1ms

debounce debounce_inst(
             .clk(fpga_clk_50),
             .reset_n(hps_fpga_reset_n),
             .data_in(KEY),//replaced KEY with static key_reg_wire
             .data_out(fpga_debounced_buttons)
         );
			
defparam debounce_inst.WIDTH = 2;
defparam debounce_inst.POLARITY = "LOW";
defparam debounce_inst.TIMEOUT = 50000;               // at 50Mhz this is a debounce time of 1ms
defparam debounce_inst.TIMEOUT_WIDTH = 16;            // ceil(log2(TIMEOUT))

// Source/Probe megawizard instance
hps_reset hps_reset_inst(
              .source_clk(fpga_clk_50),
              .source(hps_reset_req)
          );

altera_edge_detector pulse_cold_reset(
                         .clk(fpga_clk_50),
                         .rst_n(hps_fpga_reset_n),
                         .signal_in(hps_reset_req[0]),
                         .pulse_out(hps_cold_reset)
                     );
defparam pulse_cold_reset.PULSE_EXT = 6;
defparam pulse_cold_reset.EDGE_TYPE = 1;
defparam pulse_cold_reset.IGNORE_RST_WHILE_BUSY = 1;

altera_edge_detector pulse_warm_reset(
                         .clk(fpga_clk_50),
                         .rst_n(hps_fpga_reset_n),
                         .signal_in(hps_reset_req[1]),
                         .pulse_out(hps_warm_reset)
                     );
defparam pulse_warm_reset.PULSE_EXT = 2;
defparam pulse_warm_reset.EDGE_TYPE = 1;
defparam pulse_warm_reset.IGNORE_RST_WHILE_BUSY = 1;

altera_edge_detector pulse_debug_reset(
                         .clk(fpga_clk_50),
                         .rst_n(hps_fpga_reset_n),
                         .signal_in(hps_reset_req[2]),
                         .pulse_out(hps_debug_reset)
                     );
defparam pulse_debug_reset.PULSE_EXT = 32;
defparam pulse_debug_reset.EDGE_TYPE = 1;
defparam pulse_debug_reset.IGNORE_RST_WHILE_BUSY = 1;


endmodule
