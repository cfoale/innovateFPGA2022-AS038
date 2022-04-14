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
#include <math.h>

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

#define _GNU_SOURCE
#define FIFO_READOUT_COUNT	8192
#define TARGET_ARRAY_SIZE	16
#define DTUS 				 10.0    //20us is the clock time of the fpga fast_read_clk, which is dividing the PLL_divisor by 4 
#define LOOP_COUNT_MAX				10000
#define N (FIFO_READOUT_COUNT/8)

//global hps address pointers and variables
	void *virtual_base;
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

int get_target_data(double target[]){//read one float value per line
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
	
	printf("loading target.dat..\n");
	for( i=0;i<TARGET_ARRAY_SIZE/8;i++){ //print out what we got, 8 channels at a time
		printf("i %d %f %f %f %f %f %f %f %f\n",i, target[8*i+0],target[8*i+1],target[8*i+2],
			target[8*i+3],target[8*i+4],target[8*i+5],target[8*i+6],target[8*i+7]);
	}
	return 0;
}

int get_hps_addresses(){//assigns the addresses from hps to our global addresses, returns memory handle
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
	return  *(uint8_t*) hps_read_status_addr;}
	
void save_fifo_data(uint32_t fifo[]){
	//get the date and time for a file name
	time_t rawtime;
	struct tm * ti; //for date time
	time ( &rawtime );
	ti = localtime ( &rawtime );
	char buff[256];
	sprintf(buff,"c%d%d%d%d%d%d.dat",ti->tm_year+1900,ti->tm_mon+1,ti->tm_mday,ti->tm_hour,ti->tm_min,ti->tm_sec);
	int i;
	FILE *fptr;
	if ((fptr = fopen(buff,"w")) == NULL){
       printf("Error! opening file");
    }
	for(i=0;i<N;i++){ //print out what we got, 8 channels at a time//FIFO_READOUT_COUNT/8
		printf("i%d %d %d %d %d %d %d %d %d\n",i, fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
		fprintf(fptr,"%d %d %d %d %d %d %d %d %d\n",i,fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
	}
	fclose(fptr);
	printf ( "saved fifo file name: %s\n", buff );
}

int cross_correlate(double target[],uint32_t fifo[]){
	int target_detected = 1;
	int i;
	//declare 6 arrays out of 8 channels
	double Az[N],Ax[N], Wz[N], Wy[N], Px[N], Py[N];
	//declare means, standard deviations, cross-correlations
	double maz=0, max=0, mwz=0, mwy=0, mpx=0, mpy=0;
	double saz=0, sax=0, swz=0, swy=0, spx=0, spy=0;
	double corrA=0, corrW=0, corrP=0;
	//fill 6 channels using fifo
	for(i=0; i < N; i++){
		Az[i] = (float) fifo[i*8+0];
		Ax[i] = (float) fifo[i*8+1];
		Wz[i] = (float) fifo[i*8+6];
		Wy[i] = (float) fifo[i*8+7];
		Px[i] = (float) fifo[i*8+5];
		Py[i] = (float) fifo[i*8+4];
	}
	//calculate means
	for(i=0; i < N; i++){
		maz += Az[i];
		max += Ax[i];
		mwz += Wz[i];
		mwy += Wy[i];;
		mpx += Px[i];
		mpy += Py[i];
	}
	maz /=N; max /=N; mwz /=N; mwy /=N; mpx /=N; mpy /=N;
	//calculate sd's
	for(i=0; i < N; i++){
		saz += (Az[i] - maz)* (Az[i] - maz);
		sax += (Ax[i] - max)* (Ax[i] - max);
		swz += (Wz[i] - mwz)* (Wz[i] - mwz);
		swy += (Wy[i] - mwy)* (Wy[i] - mwy);
		spx += (Px[i] - mpx)* (Px[i] - mpx);
		spy += (Py[i] - mpy)* (Py[i] - mpy);
	}
	saz = sqrt(saz)/N; sax = sqrt(sax)/N; swz = sqrt(swz)/N; 
	swy = sqrt(swy)/N; spx = sqrt(spx)/N; spy = sqrt(swy)/N;
	//create cross-correlations of acceleration, rates and pressures
	for(i=0; i < N; i++){
		corrA += (Az[i] - maz)* (Az[i] - maz);
		sax += (Ax[i] - max)* (Ax[i] - max);
		swz += (Wz[i] - mwz)* (Wz[i] - mwz);
		swy += (Wy[i] - mwy)* (Wy[i] - mwy);
		spx += (Px[i] - mpx)* (Px[i] - mpx);
		spy += (Py[i] - mpy)* (Py[i] - mpy);
	}	
	return target_detected;
}
	
int main() {
	//timing variables
	struct timespec start, request, stop;
	double accum,accumrq;
	int fd;//hps address handle to be closed on exit
	int loop_count;
	int fifo_readout_count = 0; //set when adc_measured_count==0 //general counter
	int target_detected = 0;
	uint32_t fifo[FIFO_READOUT_COUNT];//our fifo readout
	
	//load the target filter data
	double target[TARGET_ARRAY_SIZE];
	if(get_target_data(target)==0)
		printf("TARGET_ARRAY_SIZE %d, target[0] %f, target[TARGET_ARRAY_SIZE-1] %f\n",TARGET_ARRAY_SIZE, target[0],target[TARGET_ARRAY_SIZE-1]);

	fd = get_hps_addresses();//required before updating the global variables
	update_read_hps();//updates our global variables with what hps currently has over the avalon bus
	print_hps_variables();//requires previous update
				 
	clock_gettime( CLOCK_REALTIME, &start);//start timer			 
	request_read_fifo(); //sets the hps_read_rq_byte
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
			target_detected = cross_correlate(target,fifo);
			if(target_detected)
				break;
		}else{
			clock_gettime( CLOCK_REALTIME, &start);//start timer			 
			request_read_fifo(); //sets the hps_read_rq_byte
		}
		loop_count++;
	} // end while
	
	if(target_detected)
		save_fifo_data(fifo);
	
	//summarize 
	print_hps_variables();
	accum = ( stop.tv_sec - start.tv_sec )+ ( stop.tv_nsec - start.tv_nsec )/ 1000000000.0;
	accumrq = ( request.tv_sec - start.tv_sec )+ ( request.tv_nsec - start.tv_nsec )/ 1000000000.0;
	printf( "dtus %f, request %lfs, readout %lfs,\n", DTUS,accumrq,accum-accumrq );
	
	// clean up our memory mapping and exit
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	close( fd );
	return( 0 );
}
