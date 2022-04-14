#ifndef _ALTERA_HPS_0_H_
#define _ALTERA_HPS_0_H_

/*
 * This file was automatically generated by the swinfo2header utility.
 * 
 * Created from SOPC Builder system 'soc_system' in
 * file './soc_system.sopcinfo'.
 */

/*
 * This file contains macros for module 'hps_0' and devices
 * connected to the following masters:
 *   h2f_axi_master
 *   h2f_lw_axi_master
 * 
 * Do not include this header file and another header file created for a
 * different module or master group at the same time.
 * Doing so may result in duplicate macro names.
 * Instead, use the system header file which has macros with unique names.
 */

/*
 * Macros for device 'hps_dsp_threshold', class 'altera_avalon_pio'
 * The macros are prefixed with 'HPS_DSP_THRESHOLD_'.
 * The prefix is the slave descriptor.
 */
#define HPS_DSP_THRESHOLD_COMPONENT_TYPE altera_avalon_pio
#define HPS_DSP_THRESHOLD_COMPONENT_NAME hps_dsp_threshold
#define HPS_DSP_THRESHOLD_BASE 0x0
#define HPS_DSP_THRESHOLD_SPAN 32
#define HPS_DSP_THRESHOLD_END 0x1f
#define HPS_DSP_THRESHOLD_BIT_CLEARING_EDGE_REGISTER 1
#define HPS_DSP_THRESHOLD_BIT_MODIFYING_OUTPUT_REGISTER 1
#define HPS_DSP_THRESHOLD_CAPTURE 1
#define HPS_DSP_THRESHOLD_DATA_WIDTH 32
#define HPS_DSP_THRESHOLD_DO_TEST_BENCH_WIRING 0
#define HPS_DSP_THRESHOLD_DRIVEN_SIM_VALUE 0
#define HPS_DSP_THRESHOLD_EDGE_TYPE RISING
#define HPS_DSP_THRESHOLD_FREQ 50000000
#define HPS_DSP_THRESHOLD_HAS_IN 0
#define HPS_DSP_THRESHOLD_HAS_OUT 0
#define HPS_DSP_THRESHOLD_HAS_TRI 1
#define HPS_DSP_THRESHOLD_IRQ_TYPE NONE
#define HPS_DSP_THRESHOLD_RESET_VALUE 0

/*
 * Macros for device 'cc_out', class 'altera_avalon_pio'
 * The macros are prefixed with 'CC_OUT_'.
 * The prefix is the slave descriptor.
 */
#define CC_OUT_COMPONENT_TYPE altera_avalon_pio
#define CC_OUT_COMPONENT_NAME cc_out
#define CC_OUT_BASE 0x20
#define CC_OUT_SPAN 16
#define CC_OUT_END 0x2f
#define CC_OUT_BIT_CLEARING_EDGE_REGISTER 0
#define CC_OUT_BIT_MODIFYING_OUTPUT_REGISTER 0
#define CC_OUT_CAPTURE 1
#define CC_OUT_DATA_WIDTH 32
#define CC_OUT_DO_TEST_BENCH_WIRING 0
#define CC_OUT_DRIVEN_SIM_VALUE 0
#define CC_OUT_EDGE_TYPE ANY
#define CC_OUT_FREQ 50000000
#define CC_OUT_HAS_IN 1
#define CC_OUT_HAS_OUT 0
#define CC_OUT_HAS_TRI 0
#define CC_OUT_IRQ_TYPE NONE
#define CC_OUT_RESET_VALUE 0

/*
 * Macros for device 'hps_fifo_wrfull', class 'altera_avalon_pio'
 * The macros are prefixed with 'HPS_FIFO_WRFULL_'.
 * The prefix is the slave descriptor.
 */
#define HPS_FIFO_WRFULL_COMPONENT_TYPE altera_avalon_pio
#define HPS_FIFO_WRFULL_COMPONENT_NAME hps_fifo_wrfull
#define HPS_FIFO_WRFULL_BASE 0x30
#define HPS_FIFO_WRFULL_SPAN 16
#define HPS_FIFO_WRFULL_END 0x3f
#define HPS_FIFO_WRFULL_BIT_CLEARING_EDGE_REGISTER 0
#define HPS_FIFO_WRFULL_BIT_MODIFYING_OUTPUT_REGISTER 0
#define HPS_FIFO_WRFULL_CAPTURE 1
#define HPS_FIFO_WRFULL_DATA_WIDTH 8
#define HPS_FIFO_WRFULL_DO_TEST_BENCH_WIRING 0
#define HPS_FIFO_WRFULL_DRIVEN_SIM_VALUE 0
#define HPS_FIFO_WRFULL_EDGE_TYPE ANY
#define HPS_FIFO_WRFULL_FREQ 50000000
#define HPS_FIFO_WRFULL_HAS_IN 1
#define HPS_FIFO_WRFULL_HAS_OUT 0
#define HPS_FIFO_WRFULL_HAS_TRI 0
#define HPS_FIFO_WRFULL_IRQ_TYPE NONE
#define HPS_FIFO_WRFULL_RESET_VALUE 0

/*
 * Macros for device 'fpga_dsp_byte', class 'altera_avalon_pio'
 * The macros are prefixed with 'FPGA_DSP_BYTE_'.
 * The prefix is the slave descriptor.
 */
#define FPGA_DSP_BYTE_COMPONENT_TYPE altera_avalon_pio
#define FPGA_DSP_BYTE_COMPONENT_NAME fpga_dsp_byte
#define FPGA_DSP_BYTE_BASE 0x40
#define FPGA_DSP_BYTE_SPAN 16
#define FPGA_DSP_BYTE_END 0x4f
#define FPGA_DSP_BYTE_BIT_CLEARING_EDGE_REGISTER 0
#define FPGA_DSP_BYTE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define FPGA_DSP_BYTE_CAPTURE 1
#define FPGA_DSP_BYTE_DATA_WIDTH 8
#define FPGA_DSP_BYTE_DO_TEST_BENCH_WIRING 0
#define FPGA_DSP_BYTE_DRIVEN_SIM_VALUE 0
#define FPGA_DSP_BYTE_EDGE_TYPE RISING
#define FPGA_DSP_BYTE_FREQ 50000000
#define FPGA_DSP_BYTE_HAS_IN 1
#define FPGA_DSP_BYTE_HAS_OUT 0
#define FPGA_DSP_BYTE_HAS_TRI 0
#define FPGA_DSP_BYTE_IRQ_TYPE NONE
#define FPGA_DSP_BYTE_RESET_VALUE 0

/*
 * Macros for device 'hps_dsp_byte', class 'altera_avalon_pio'
 * The macros are prefixed with 'HPS_DSP_BYTE_'.
 * The prefix is the slave descriptor.
 */
#define HPS_DSP_BYTE_COMPONENT_TYPE altera_avalon_pio
#define HPS_DSP_BYTE_COMPONENT_NAME hps_dsp_byte
#define HPS_DSP_BYTE_BASE 0x50
#define HPS_DSP_BYTE_SPAN 16
#define HPS_DSP_BYTE_END 0x5f
#define HPS_DSP_BYTE_BIT_CLEARING_EDGE_REGISTER 0
#define HPS_DSP_BYTE_BIT_MODIFYING_OUTPUT_REGISTER 0
#define HPS_DSP_BYTE_CAPTURE 0
#define HPS_DSP_BYTE_DATA_WIDTH 8
#define HPS_DSP_BYTE_DO_TEST_BENCH_WIRING 0
#define HPS_DSP_BYTE_DRIVEN_SIM_VALUE 0
#define HPS_DSP_BYTE_EDGE_TYPE NONE
#define HPS_DSP_BYTE_FREQ 50000000
#define HPS_DSP_BYTE_HAS_IN 0
#define HPS_DSP_BYTE_HAS_OUT 1
#define HPS_DSP_BYTE_HAS_TRI 0
#define HPS_DSP_BYTE_IRQ_TYPE NONE
#define HPS_DSP_BYTE_RESET_VALUE 0

/*
 * Macros for device 'fifo_usedw32', class 'altera_avalon_pio'
 * The macros are prefixed with 'FIFO_USEDW32_'.
 * The prefix is the slave descriptor.
 */
#define FIFO_USEDW32_COMPONENT_TYPE altera_avalon_pio
#define FIFO_USEDW32_COMPONENT_NAME fifo_usedw32
#define FIFO_USEDW32_BASE 0x60
#define FIFO_USEDW32_SPAN 16
#define FIFO_USEDW32_END 0x6f
#define FIFO_USEDW32_BIT_CLEARING_EDGE_REGISTER 0
#define FIFO_USEDW32_BIT_MODIFYING_OUTPUT_REGISTER 0
#define FIFO_USEDW32_CAPTURE 1
#define FIFO_USEDW32_DATA_WIDTH 32
#define FIFO_USEDW32_DO_TEST_BENCH_WIRING 0
#define FIFO_USEDW32_DRIVEN_SIM_VALUE 0
#define FIFO_USEDW32_EDGE_TYPE ANY
#define FIFO_USEDW32_FREQ 50000000
#define FIFO_USEDW32_HAS_IN 1
#define FIFO_USEDW32_HAS_OUT 0
#define FIFO_USEDW32_HAS_TRI 0
#define FIFO_USEDW32_IRQ_TYPE NONE
#define FIFO_USEDW32_RESET_VALUE 0

/*
 * Macros for device 'hps_read_clk', class 'altera_avalon_pio'
 * The macros are prefixed with 'HPS_READ_CLK_'.
 * The prefix is the slave descriptor.
 */
#define HPS_READ_CLK_COMPONENT_TYPE altera_avalon_pio
#define HPS_READ_CLK_COMPONENT_NAME hps_read_clk
#define HPS_READ_CLK_BASE 0x70
#define HPS_READ_CLK_SPAN 16
#define HPS_READ_CLK_END 0x7f
#define HPS_READ_CLK_BIT_CLEARING_EDGE_REGISTER 0
#define HPS_READ_CLK_BIT_MODIFYING_OUTPUT_REGISTER 0
#define HPS_READ_CLK_CAPTURE 0
#define HPS_READ_CLK_DATA_WIDTH 8
#define HPS_READ_CLK_DO_TEST_BENCH_WIRING 0
#define HPS_READ_CLK_DRIVEN_SIM_VALUE 0
#define HPS_READ_CLK_EDGE_TYPE NONE
#define HPS_READ_CLK_FREQ 50000000
#define HPS_READ_CLK_HAS_IN 0
#define HPS_READ_CLK_HAS_OUT 1
#define HPS_READ_CLK_HAS_TRI 0
#define HPS_READ_CLK_IRQ_TYPE NONE
#define HPS_READ_CLK_RESET_VALUE 0

/*
 * Macros for device 'hps_read_rq', class 'altera_avalon_pio'
 * The macros are prefixed with 'HPS_READ_RQ_'.
 * The prefix is the slave descriptor.
 */
#define HPS_READ_RQ_COMPONENT_TYPE altera_avalon_pio
#define HPS_READ_RQ_COMPONENT_NAME hps_read_rq
#define HPS_READ_RQ_BASE 0x80
#define HPS_READ_RQ_SPAN 16
#define HPS_READ_RQ_END 0x8f
#define HPS_READ_RQ_BIT_CLEARING_EDGE_REGISTER 0
#define HPS_READ_RQ_BIT_MODIFYING_OUTPUT_REGISTER 0
#define HPS_READ_RQ_CAPTURE 0
#define HPS_READ_RQ_DATA_WIDTH 8
#define HPS_READ_RQ_DO_TEST_BENCH_WIRING 0
#define HPS_READ_RQ_DRIVEN_SIM_VALUE 0
#define HPS_READ_RQ_EDGE_TYPE NONE
#define HPS_READ_RQ_FREQ 50000000
#define HPS_READ_RQ_HAS_IN 0
#define HPS_READ_RQ_HAS_OUT 1
#define HPS_READ_RQ_HAS_TRI 0
#define HPS_READ_RQ_IRQ_TYPE NONE
#define HPS_READ_RQ_RESET_VALUE 0

/*
 * Macros for device 'hps_word16', class 'altera_avalon_pio'
 * The macros are prefixed with 'HPS_WORD16_'.
 * The prefix is the slave descriptor.
 */
#define HPS_WORD16_COMPONENT_TYPE altera_avalon_pio
#define HPS_WORD16_COMPONENT_NAME hps_word16
#define HPS_WORD16_BASE 0x90
#define HPS_WORD16_SPAN 16
#define HPS_WORD16_END 0x9f
#define HPS_WORD16_BIT_CLEARING_EDGE_REGISTER 0
#define HPS_WORD16_BIT_MODIFYING_OUTPUT_REGISTER 0
#define HPS_WORD16_CAPTURE 1
#define HPS_WORD16_DATA_WIDTH 16
#define HPS_WORD16_DO_TEST_BENCH_WIRING 0
#define HPS_WORD16_DRIVEN_SIM_VALUE 0
#define HPS_WORD16_EDGE_TYPE ANY
#define HPS_WORD16_FREQ 50000000
#define HPS_WORD16_HAS_IN 1
#define HPS_WORD16_HAS_OUT 0
#define HPS_WORD16_HAS_TRI 0
#define HPS_WORD16_IRQ_TYPE NONE
#define HPS_WORD16_RESET_VALUE 0

/*
 * Macros for device 'hps_read_status', class 'altera_avalon_pio'
 * The macros are prefixed with 'HPS_READ_STATUS_'.
 * The prefix is the slave descriptor.
 */
#define HPS_READ_STATUS_COMPONENT_TYPE altera_avalon_pio
#define HPS_READ_STATUS_COMPONENT_NAME hps_read_status
#define HPS_READ_STATUS_BASE 0xa0
#define HPS_READ_STATUS_SPAN 16
#define HPS_READ_STATUS_END 0xaf
#define HPS_READ_STATUS_BIT_CLEARING_EDGE_REGISTER 0
#define HPS_READ_STATUS_BIT_MODIFYING_OUTPUT_REGISTER 0
#define HPS_READ_STATUS_CAPTURE 1
#define HPS_READ_STATUS_DATA_WIDTH 8
#define HPS_READ_STATUS_DO_TEST_BENCH_WIRING 0
#define HPS_READ_STATUS_DRIVEN_SIM_VALUE 0
#define HPS_READ_STATUS_EDGE_TYPE ANY
#define HPS_READ_STATUS_FREQ 50000000
#define HPS_READ_STATUS_HAS_IN 1
#define HPS_READ_STATUS_HAS_OUT 0
#define HPS_READ_STATUS_HAS_TRI 0
#define HPS_READ_STATUS_IRQ_TYPE NONE
#define HPS_READ_STATUS_RESET_VALUE 0

/*
 * Macros for device 'gpio_1_d34_d32_d30_d28', class 'altera_avalon_pio'
 * The macros are prefixed with 'GPIO_1_D34_D32_D30_D28_'.
 * The prefix is the slave descriptor.
 */
#define GPIO_1_D34_D32_D30_D28_COMPONENT_TYPE altera_avalon_pio
#define GPIO_1_D34_D32_D30_D28_COMPONENT_NAME gpio_1_d34_d32_d30_d28
#define GPIO_1_D34_D32_D30_D28_BASE 0xb0
#define GPIO_1_D34_D32_D30_D28_SPAN 16
#define GPIO_1_D34_D32_D30_D28_END 0xbf
#define GPIO_1_D34_D32_D30_D28_BIT_CLEARING_EDGE_REGISTER 0
#define GPIO_1_D34_D32_D30_D28_BIT_MODIFYING_OUTPUT_REGISTER 0
#define GPIO_1_D34_D32_D30_D28_CAPTURE 1
#define GPIO_1_D34_D32_D30_D28_DATA_WIDTH 4
#define GPIO_1_D34_D32_D30_D28_DO_TEST_BENCH_WIRING 0
#define GPIO_1_D34_D32_D30_D28_DRIVEN_SIM_VALUE 0
#define GPIO_1_D34_D32_D30_D28_EDGE_TYPE ANY
#define GPIO_1_D34_D32_D30_D28_FREQ 50000000
#define GPIO_1_D34_D32_D30_D28_HAS_IN 1
#define GPIO_1_D34_D32_D30_D28_HAS_OUT 0
#define GPIO_1_D34_D32_D30_D28_HAS_TRI 0
#define GPIO_1_D34_D32_D30_D28_IRQ_TYPE NONE
#define GPIO_1_D34_D32_D30_D28_RESET_VALUE 0

/*
 * Macros for device 'sysid_qsys', class 'altera_avalon_sysid_qsys'
 * The macros are prefixed with 'SYSID_QSYS_'.
 * The prefix is the slave descriptor.
 */
#define SYSID_QSYS_COMPONENT_TYPE altera_avalon_sysid_qsys
#define SYSID_QSYS_COMPONENT_NAME sysid_qsys
#define SYSID_QSYS_BASE 0x1000
#define SYSID_QSYS_SPAN 8
#define SYSID_QSYS_END 0x1007
#define SYSID_QSYS_ID 2899645186
#define SYSID_QSYS_TIMESTAMP 1643766603

/*
 * Macros for device 'jtag_uart', class 'altera_avalon_jtag_uart'
 * The macros are prefixed with 'JTAG_UART_'.
 * The prefix is the slave descriptor.
 */
#define JTAG_UART_COMPONENT_TYPE altera_avalon_jtag_uart
#define JTAG_UART_COMPONENT_NAME jtag_uart
#define JTAG_UART_BASE 0x2000
#define JTAG_UART_SPAN 8
#define JTAG_UART_END 0x2007
#define JTAG_UART_IRQ 0
#define JTAG_UART_READ_DEPTH 64
#define JTAG_UART_READ_THRESHOLD 8
#define JTAG_UART_WRITE_DEPTH 64
#define JTAG_UART_WRITE_THRESHOLD 8

/*
 * Macros for device 'led_pio', class 'altera_avalon_pio'
 * The macros are prefixed with 'LED_PIO_'.
 * The prefix is the slave descriptor.
 */
#define LED_PIO_COMPONENT_TYPE altera_avalon_pio
#define LED_PIO_COMPONENT_NAME led_pio
#define LED_PIO_BASE 0x3000
#define LED_PIO_SPAN 16
#define LED_PIO_END 0x300f
#define LED_PIO_BIT_CLEARING_EDGE_REGISTER 0
#define LED_PIO_BIT_MODIFYING_OUTPUT_REGISTER 0
#define LED_PIO_CAPTURE 0
#define LED_PIO_DATA_WIDTH 7
#define LED_PIO_DO_TEST_BENCH_WIRING 0
#define LED_PIO_DRIVEN_SIM_VALUE 0
#define LED_PIO_EDGE_TYPE NONE
#define LED_PIO_FREQ 50000000
#define LED_PIO_HAS_IN 0
#define LED_PIO_HAS_OUT 1
#define LED_PIO_HAS_TRI 0
#define LED_PIO_IRQ_TYPE NONE
#define LED_PIO_RESET_VALUE 127

/*
 * Macros for device 'dipsw_pio', class 'altera_avalon_pio'
 * The macros are prefixed with 'DIPSW_PIO_'.
 * The prefix is the slave descriptor.
 */
#define DIPSW_PIO_COMPONENT_TYPE altera_avalon_pio
#define DIPSW_PIO_COMPONENT_NAME dipsw_pio
#define DIPSW_PIO_BASE 0x4000
#define DIPSW_PIO_SPAN 16
#define DIPSW_PIO_END 0x400f
#define DIPSW_PIO_IRQ 2
#define DIPSW_PIO_BIT_CLEARING_EDGE_REGISTER 1
#define DIPSW_PIO_BIT_MODIFYING_OUTPUT_REGISTER 0
#define DIPSW_PIO_CAPTURE 1
#define DIPSW_PIO_DATA_WIDTH 4
#define DIPSW_PIO_DO_TEST_BENCH_WIRING 0
#define DIPSW_PIO_DRIVEN_SIM_VALUE 0
#define DIPSW_PIO_EDGE_TYPE ANY
#define DIPSW_PIO_FREQ 50000000
#define DIPSW_PIO_HAS_IN 1
#define DIPSW_PIO_HAS_OUT 0
#define DIPSW_PIO_HAS_TRI 0
#define DIPSW_PIO_IRQ_TYPE EDGE
#define DIPSW_PIO_RESET_VALUE 0

/*
 * Macros for device 'button_pio', class 'altera_avalon_pio'
 * The macros are prefixed with 'BUTTON_PIO_'.
 * The prefix is the slave descriptor.
 */
#define BUTTON_PIO_COMPONENT_TYPE altera_avalon_pio
#define BUTTON_PIO_COMPONENT_NAME button_pio
#define BUTTON_PIO_BASE 0x5000
#define BUTTON_PIO_SPAN 16
#define BUTTON_PIO_END 0x500f
#define BUTTON_PIO_IRQ 1
#define BUTTON_PIO_BIT_CLEARING_EDGE_REGISTER 1
#define BUTTON_PIO_BIT_MODIFYING_OUTPUT_REGISTER 0
#define BUTTON_PIO_CAPTURE 1
#define BUTTON_PIO_DATA_WIDTH 2
#define BUTTON_PIO_DO_TEST_BENCH_WIRING 0
#define BUTTON_PIO_DRIVEN_SIM_VALUE 0
#define BUTTON_PIO_EDGE_TYPE FALLING
#define BUTTON_PIO_FREQ 50000000
#define BUTTON_PIO_HAS_IN 1
#define BUTTON_PIO_HAS_OUT 0
#define BUTTON_PIO_HAS_TRI 0
#define BUTTON_PIO_IRQ_TYPE EDGE
#define BUTTON_PIO_RESET_VALUE 0

/*
 * Macros for device 'ILC', class 'interrupt_latency_counter'
 * The macros are prefixed with 'ILC_'.
 * The prefix is the slave descriptor.
 */
#define ILC_COMPONENT_TYPE interrupt_latency_counter
#define ILC_COMPONENT_NAME ILC
#define ILC_BASE 0x30000
#define ILC_SPAN 256
#define ILC_END 0x300ff


#endif /* _ALTERA_HPS_0_H_ */
