/*
This program demonstrate how to use hps communicate with FPGA through light AXI Bridge.
uses should program the FPGA by GHRD project before executing the program
refer to user manual chapter 7 for details about the demo
*/


#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "hps_0.h"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
#define FIFO_READOUT_COUNT	32
int main() {

	void *virtual_base;
	int fd;
	int loop_count;
	
	//mike gpio add
	uint8_t * h2p_lw_gpio_addr; 
	int gpio_1_d34;
	uint8_t * h2p_lw_dipsw_addr;
	uint32_t dipsw_mask;
	//mike adc add
	uint8_t * adc_measured_count_addr;
	uint32_t adc_measured_count;
	uint32_t last_measured_count=0;
	uint8_t * adc_measured_32_addr;
	uint32_t  adc_measured_32;
	uint8_t * fifo_readout_32_addr;
	uint32_t  fifo_readout_32;
	int fifo_readout_count = 0; //set when adc_measured_count==0 //general counter
	uint32_t max_value=0xab0; //roughly no pressure difference on channel 7 
	uint32_t fifo[FIFO_READOUT_COUNT];
	
	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );
	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	//mike adds to read gpio_1_d34_pin_ae19 as first element of gpio_1_d34_d32_d30_d28
	h2p_lw_gpio_addr=(uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + GPIO_1_D34_D32_D30_D28_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_dipsw_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DIPSW_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	adc_measured_count_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_MEASURED_COUNT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	adc_measured_32_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + ADC_MEASURED_32_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	fifo_readout_32_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FIFO_READOUT_32_BASE ) & ( unsigned long)( HW_REGS_MASK ) );

	loop_count = 0;
	while( loop_count < 60 ) {
		gpio_1_d34 =  *(uint32_t*) h2p_lw_gpio_addr;
		dipsw_mask = *(uint32_t *) h2p_lw_dipsw_addr;
		adc_measured_count = *(uint32_t*) adc_measured_count_addr;
		adc_measured_32 = *(uint32_t*) adc_measured_32_addr;
		fifo_readout_32 = *(uint32_t*) fifo_readout_32_addr;
		if(adc_measured_32 > max_value) max_value = adc_measured_32;
		
		//printf("gpio_1_d34=%x, dipsw=%x, measured_count=%x, measured=%x ,fifo=%x\n",gpio_1_d34,dipsw_mask,adc_measured_count,adc_measured_32, fifo_readout_32);
		// wait 100ms if not fifo readout
		if(adc_measured_count==0 && fifo_readout_count==0){ //here comes fifo fast
			while(fifo_readout_count < FIFO_READOUT_COUNT ){ //fast polling
				if(last_measured_count != adc_measured_count){
					fifo_readout_32 = *(uint32_t*) fifo_readout_32_addr;
					//printf("measured_count=%x,fifo=%x\n",adc_measured_count, fifo_readout_32);
					fifo[fifo_readout_count] = fifo_readout_32;
					last_measured_count = adc_measured_count;
					fifo_readout_count++;
				}
				adc_measured_count = *(uint32_t*) adc_measured_count_addr;
			}
			break;
		}//else{
			//usleep( 1000*1000 );
		//}
	} // while
	int i;
	for(i=0;i<FIFO_READOUT_COUNT;i++)
		printf("i=%x,fifo=%x\n",i, fifo[i]);
	// clean up our memory mapping and exit
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	close( fd );
	return( 0 );
}
