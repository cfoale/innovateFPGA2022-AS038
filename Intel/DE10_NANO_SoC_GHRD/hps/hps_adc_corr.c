/*
Modified prototype (not final) program by Colin Michael Foale 2022.
One time read of FPGA 256 12bit ADC word blocks. The read is triggered by FPGA push button

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
#include <time.h>
#include <string.h>

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define FIFO_READOUT_COUNT	256

void send_read_clks(int n, uint8_t *hps_read_clk_byte_addr){
	int i;
	for(i=0;i<n;i++){
		*hps_read_clk_byte_addr = 0x00;
		usleep( 100*1000 );
		*hps_read_clk_byte_addr = 0xff;//end high
	}
}

int main() {
	struct timespec start, request, stop;
	struct tm * ti; //for date time
	time_t rawtime;
	double accum,accumrq;
	FILE *fptr;
	void *virtual_base;
	int fd;//hps address handl
	int loop_count;
	
	//mike gpio add
	//uint8_t * h2p_lw_gpio_addr; 
	//int gpio_1_d34;
	uint8_t * h2p_lw_dipsw_addr;
	uint32_t dipsw_mask;
	//mike adc add
	uint8_t * fifo_usedw32_addr;
	uint32_t fifo_usedw32;
	uint32_t previous_fifo_used;
	
	uint8_t * hps_word_addr;
	uint32_t hps_word;

	uint8_t * hps_read_status_addr;
	uint8_t  hps_read_status;
	
	uint8_t * hps_fifo_wrfull_addr;
	uint8_t  hps_fifo_wrfull;
	
	uint8_t * hps_read_clk_byte_addr;
	uint8_t  hps_read_clk_byte;
	
	uint8_t * hps_read_rq_byte_addr;
	uint8_t  hps_read_rq_byte;
	
	int fifo_readout_count = 0; //set when adc_measured_count==0 //general counter
	uint32_t max_value=0x0; //
	uint32_t min_value=0xfff;
	uint32_t fifo[FIFO_READOUT_COUNT];
	
	float dtus = 10.0;//20us is the clock time of the fpga fast_read_clk, which is dividing the PLL_divisor by 4 

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
	//h2p_lw_gpio_addr=(uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + GPIO_1_D34_D32_D30_D28_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_dipsw_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DIPSW_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_read_clk_byte_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_READ_CLK_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_read_rq_byte_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_READ_RQ_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_read_status_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_READ_STATUS_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_fifo_wrfull_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_FIFO_WRFULL_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_word_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_WORD16_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	fifo_usedw32_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FIFO_USEDW32_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	
	//gpio_1_d34 =  *(uint32_t*) h2p_lw_gpio_addr;
	dipsw_mask = *(uint32_t *) h2p_lw_dipsw_addr;
	hps_read_clk_byte = *(uint8_t*) hps_read_clk_byte_addr;//need to write to adddress location..
	hps_read_rq_byte = *(uint8_t*) hps_read_rq_byte_addr;//need to write to addresss...
	hps_read_status = *(uint8_t*) hps_read_status_addr;
	hps_fifo_wrfull = *(uint8_t*) hps_fifo_wrfull_addr;
	hps_word = *(uint32_t*) hps_word_addr;
	fifo_usedw32 =  *(uint32_t*) fifo_usedw32_addr;
	
	printf("dipsw=%d,  hps_read_status=%d, hps_word=%d, hps_read_rq_byte=%d, hps_read_clk_byte=%d, fifo_used %d, wrfull %d\n", 
	dipsw_mask, hps_read_status,hps_word, hps_read_rq_byte, hps_read_clk_byte,fifo_usedw32,hps_fifo_wrfull);
				 
	while(hps_read_status==1){ //incase we have to clean out fifo.. not good
				printf("sending hps_read_clk to terminate hps_read_status..\n");
				//cycle the hps_read_clk
				*hps_read_clk_byte_addr = 0x00;
				usleep( dtus );
				*hps_read_clk_byte_addr = 0xff;
				usleep( dtus );
				//put a byte into hps_read_rq_byte_addr
				*hps_read_rq_byte_addr = 0x0;
				hps_read_status = *(uint8_t*) hps_read_status_addr;
	}
	hps_read_clk_byte = *(uint8_t*) hps_read_clk_byte_addr;//need to write to adddress location..
	hps_read_rq_byte = *(uint8_t*) hps_read_rq_byte_addr;//need to write to addresss...
	hps_read_status = *(uint8_t*) hps_read_status_addr;
	hps_fifo_wrfull = *(uint8_t*) hps_fifo_wrfull_addr;
	hps_word = *(uint32_t*) hps_word_addr;
	fifo_usedw32 =  *(uint32_t*) fifo_usedw32_addr;
	printf("dipsw=%d,  hps_read_status=%d, hps_word=%d, hps_read_rq_byte=%d, hps_read_clk_byte=%d, fifo_used %d, wrfull %d\n", 
	dipsw_mask, hps_read_status,hps_word, hps_read_rq_byte, hps_read_clk_byte,fifo_usedw32,hps_fifo_wrfull);
	clock_gettime( CLOCK_REALTIME, &start);			 
	//put a byte into hps_read_rq_byte_addr
	*hps_read_rq_byte_addr = 0xff;
	*hps_read_clk_byte_addr = 0xff; //initialize hi
	usleep( dtus ); //delay
	loop_count = 0;
	
	while( loop_count < 50000 ) { // >  FIFO_READOUT_COUNT*0.243*8/(0.001*10) = 49,776
		loop_count++;
		//gpio_1_d34 =  *h2p_lw_gpio_addr;
		dipsw_mask = *h2p_lw_dipsw_addr;
		hps_read_clk_byte = *hps_read_clk_byte_addr;//need to write to adddress location..
		hps_read_rq_byte = *hps_read_rq_byte_addr;//need to write to addresss...
		hps_read_status = *hps_read_status_addr;
		hps_word = *(uint32_t*) hps_word_addr;
		fifo_usedw32 =  *(uint32_t*) fifo_usedw32_addr;
		
		if(hps_word > max_value) max_value = hps_word;
		if(hps_word < min_value) min_value = hps_word;
		
		//printf("loop_count %d,  hps_read_status=%d, hps_word=%d, hps_read_rq_byte=%d, hps_read_clk_byte=%d, fifo_used %d\n",loop_count,  hps_read_status,
				 //hps_word, hps_read_rq_byte, hps_read_clk_byte,fifo_usedw32);
		if((hps_read_status==1) && (fifo_readout_count==0) && (hps_fifo_wrfull==0x1)){ //here comes fifo 
			previous_fifo_used = fifo_usedw32;//we will look for changes to know we have the next value
			//cycle the hps_read_clk, begins high
			*hps_read_clk_byte_addr = 0x00;//clk low
			usleep( dtus);
			*hps_read_clk_byte_addr = 0xff;//clk high
			usleep( dtus);
			//fifo_used should change to FIFO_WORDS-1
			*hps_read_rq_byte_addr = 0x00;//set read request low since acknowleged
			clock_gettime( CLOCK_REALTIME, &request);//timing
			while(fifo_readout_count < FIFO_READOUT_COUNT && hps_read_status==1 ){ //fast polling
					hps_word = *(uint32_t*) hps_word_addr;
					hps_read_status = *hps_read_status_addr;
					hps_read_rq_byte = *hps_read_rq_byte_addr;
					fifo_usedw32 =  *(uint32_t*) fifo_usedw32_addr;
					//printf("i:%d, dipsw=%d,  hps_read_status=%d, hps_word=%d, hps_read_rq_byte=%d, hps_read_clk_byte=%d, fifo_used %d\n", 
						//fifo_readout_count,dipsw_mask, hps_read_status,
						//hps_word, hps_read_rq_byte, hps_read_clk_byte,fifo_usedw32);
					if(fifo_usedw32 != previous_fifo_used){
						fifo[fifo_readout_count] = hps_word;
						previous_fifo_used = fifo_usedw32;
						fifo_readout_count++;
					}
					previous_fifo_used = fifo_usedw32;;
					//cycle the hps_read_clk
					*hps_read_clk_byte_addr = 0x00;
					usleep( dtus);
					*hps_read_clk_byte_addr = 0xff;//end high
					usleep( dtus);
			}
			clock_gettime( CLOCK_REALTIME, &stop);
			accum = ( stop.tv_sec - start.tv_sec )
					+ ( stop.tv_nsec - start.tv_nsec )/ 1000000000.0;
			accumrq = ( request.tv_sec - start.tv_sec )
					+ ( request.tv_nsec - start.tv_nsec )/ 1000000000.0;
			while(hps_read_status==1){ //incase we have to clean out.. not good
				printf("sending hps_read_clk to terminate hps_read_status..\n");
				hps_read_status = *hps_read_status_addr;
				//cycle the hps_read_clk
				*hps_read_clk_byte_addr = 0x00;
				usleep( dtus );
				*hps_read_clk_byte_addr = 0xff;
				usleep( dtus );
			}		
			hps_read_rq_byte = *hps_read_rq_byte_addr;
			fifo_usedw32 =  *(uint32_t*) fifo_usedw32_addr;
			break;
		}else{
			usleep( dtus );
		}
	} // while
	//get the date and time for a file name
	time ( &rawtime );
	ti = localtime ( &rawtime );
	char buff[256];
	sprintf(buff,"c%d%d%d%d%d%d.dat",ti->tm_year+1900,ti->tm_mon+1,ti->tm_mday,ti->tm_hour,ti->tm_min,ti->tm_sec);
	int i;
	if ((fptr = fopen(buff,"w")) == NULL){
       printf("Error! opening file");
    }
	for(i=0;i<FIFO_READOUT_COUNT/8;i++){ //print out what we got, 8 channels at a time
		printf("i%d %d %d %d %d %d %d %d %d\n",i, fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
		fprintf(fptr,"%d %d %d %d %d %d %d %d %d\n",i,fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
	}
	fclose(fptr);
	//do it again using corr.dat
	if ((fptr = fopen("corr.dat","w")) == NULL){
       printf("Error! opening file");
    }
	for(i=0;i<FIFO_READOUT_COUNT/8;i++){ //print out what we got, 8 channels at a time
		//printf("i%d %d %d %d %d %d %d %d %d\n",i, fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
		fprintf(fptr,"%d %d %d %d %d %d %d %d %d\n",i,fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
	}
	fclose(fptr);
	
	printf("dipsw=%d,  hps_read_status=%d, hps_word=%d, hps_read_rq_byte=%d, hps_read_clk_byte=%d, fifo_used %d\n", dipsw_mask, hps_read_status,
				 hps_word, hps_read_rq_byte, hps_read_clk_byte,fifo_usedw32);
	printf( "dtus %f, request %lfs, readout %lfs,\n", dtus,accumrq,accum-accumrq );
	printf ( "Current local time and date: %s\n", buff );
	// clean up our memory mapping and exit
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	close( fd );
	return( 0 );
}
