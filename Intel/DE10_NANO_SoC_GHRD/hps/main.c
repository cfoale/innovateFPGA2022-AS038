/*main.c make target hps_adc_corr reads 8 channel data frames from fpga, calculates crosscorrelation to target.dat
 and sends the result to fpga as a single byte.
*/

#include <stdio.h>
#include <stdlib.h>
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

#define _GNU_SOURCE
#define FIFO_READOUT_COUNT	8192
#define TARGET_ARRAY_SIZE	16
#define DTUS 				 10.0    //20us is the clock time of the fpga fast_read_clk, which is dividing the PLL_divisor by 4 
#LOOP_COUNT_MAX				10000

//global hps address pointers and variables
	uint8_t * h2p_lw_dipsw_addr;  //switches
	uint32_t dipsw_mask;

	uint8_t * fifo_usedw32_addr;//fifo
	uint32_t fifo_usedw32;
	uint32_t previous_fifo_used;//used to keep sycnchronized with fifo during read
	
	uint8_t * hps_word_addr;//fifo output q
	uint32_t hps_word;

	uint8_t * hps_read_status_addr; //fpga ready to send fifo status to hps
	uint8_t  hps_read_status;
	
	uint8_t * hps_read_clk_byte_addr;//hps to fifo read clock
	uint8_t  hps_read_clk_byte;
	
	uint8_t * hps_read_rq_byte_addr;//hps request to read fifo
	uint8_t  hps_read_rq_byte;
//end of global variables

void clk(float dtus){ //sends a single clock low, clock high with duration 2 * dtus
	*hps_read_clk_byte_addr = 0x00;
	usleep( dtus );
	*hps_read_clk_byte_addr = 0xff;//end high
	usleep( dtus );
}

int get_target_data(float* target){//read one float value per line
	FILE * fp;
    char * line = NULL;
    size_t len = 0;
    ssize_t read;
	int i=0;
	
    fp = fopen("target.dat", "r");
    if (fp == NULL){
		printf("Unable to open target.dat");
        exit(1);
	}
	
    while ((read = getline(&line, &len, fp)) != -1) {
        printf("%d:Retrieved line of length %zu:  %s\n",i, read, line);
		target[i] = atof(line);//returns double precision
		i++;
		if(i >= TARGET_ARRAY_SIZE)
			break;
    }
    fclose(fp);
    if (line)
        free(line);
	
	for( i=0;i<TARGET_ARRAY_SIZE/8;i++){ //print out what we got, 8 channels at a time
		printf("loading target.dat..\n");
		printf("i %f %f %f %f %f %f %f %f %f\n",i, target[i][8*i+0],target[i][8*i+1],target[i][8*i+2],
			target[i][8*i+3],target[i][8*i+4],target[i][8*i+5],target[i][8*i+6],target[i][8*i+7]);
	}
}

int get_hps_addresses(){//assigns the addresses from hps to our global addresses, returns memory handle
	void *virtual_base;
	int fd;//hps address handle
	// map the address space - the entire CSR span of the HPS since we want to access various registers within that span
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
	
	h2p_lw_dipsw_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DIPSW_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_read_clk_byte_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_READ_CLK_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_read_rq_byte_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_READ_RQ_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_read_status_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_READ_STATUS_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_word_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_WORD16_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	fifo_usedw32_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FIFO_USEDW32_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	return fd;//to be closed on exit
}

void update_read_hps(){ //updates our global variables with hps avalon values using global addresses
	dipsw_mask = *(uint32_t *) h2p_lw_dipsw_addr;
	hps_read_clk_byte = *(uint8_t*) hps_read_clk_byte_addr;//need to write to adddress location..
	hps_read_rq_byte = *(uint8_t*) hps_read_rq_byte_addr;//need to write to addresss...
	hps_read_status = *(uint8_t*) hps_read_status_addr;
	hps_word = *(uint32_t*) hps_word_addr;
	fifo_usedw32 =  *(uint32_t*) fifo_usedw32_addr;
}

void request_read_fifo(){//the fpga will read channels upto 7, with fifo full, and then pause to service the request
	//put a byte into hps_read_rq_byte_addr
	*hps_read_rq_byte_addr = 0xff;
	*hps_read_clk_byte_addr = 0xff; //initialize hi
}

void reset_read_fifo(){
			*hps_read_rq_byte_addr = 0x0;
}

void print_hps_variables(){//requires previous update
	printf("dipsw=%d,  hps_read_status=%d, hps_word=%d, hps_read_rq_byte=%d, hps_read_clk_byte=%d, fifo_used %d\n", dipsw_mask, hps_read_status,
				 hps_word, hps_read_rq_byte, hps_read_clk_byte,fifo_usedw32);
}

uint8_t get_read_status(){
	return = *(uint8_t*) hps_read_status_addr;}
	
int main() {
	//timing variables
	struct timespec start, request, stop;
	struct tm * ti; //for date time
	time_t rawtime;
	double accum,accumrq;

	int fd;//hps address handle to be closed on exit
	int loop_count;
	int fifo_readout_count = 0; //set when adc_measured_count==0 //general counter
	uint32_t fifo[FIFO_READOUT_COUNT];//our fifo readout

	fd = get_hps_addresses();//required before updating the global variables
	update_read_hps();//updates our global variables with what hps currently has over the avalon bus
	print_hps_variables();//requires previous update
				 
	clock_gettime( CLOCK_REALTIME, &start);//start timer			 
	request_read_fifo(); //sets the hps_read_rq_byte
	usleep( DTUS ); //delay for hps_read_status
	loop_count = 0;//exit after a few
	
	while( loop_count < LOOP_COUNT_MAX ) {
		update_read_hps();
		if((hps_read_status==1) && (fifo_readout_count==0) && (fifo_usedw32==0x00)){ //here comes fifo 
			previous_fifo_used = fifo_usedw32;//we will look for changes to know we have the next value
			clk(DTUS);//cycle the hps_read_clk
			//fifo_used should change to FIFO_WORDS-1
			reset_read_fifo();
			//mark when we requested
			clock_gettime( CLOCK_REALTIME, &request);
			while(fifo_readout_count < FIFO_READOUT_COUNT && hps_read_status==1 ){ //fast polling
					update_read_hps();
					if(fifo_usedw32 != previous_fifo_used){
						fifo[fifo_readout_count] = hps_word;
						previous_fifo_used = fifo_usedw32;
						fifo_readout_count++;
					}
					previous_fifo_used = fifo_usedw32;;
					clk(DTUS);//cycle the hps_read_clk
			}
			//when we got fifo
			clock_gettime( CLOCK_REALTIME, &stop);
			break;
		}else{
			usleep( DTUS );
		}
		loop_count++;
	} // end while
	
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
		printf("i%d %d %d %d %d %d %d %d %d\n",i, fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
		fprintf(fptr,"%d %d %d %d %d %d %d %d %d\n",i,fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
	}
	fclose(fptr);
	
	print_hps_variables();
	accum = ( stop.tv_sec - start.tv_sec )+ ( stop.tv_nsec - start.tv_nsec )/ 1000000000.0;
	accumrq = ( request.tv_sec - start.tv_sec )+ ( request.tv_nsec - start.tv_nsec )/ 1000000000.0;
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
