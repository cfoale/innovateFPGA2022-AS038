/*hps_adc_dsp.c -- hps_adc_dsp reads 8 channel data frames from fpga, calculates crosscorrelation to target.dat
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

int get_target_data(double target[]);
int clk(unsigned int dtus); //sends a single clock low, clock high with duration 2 * dtus
int get_hps_addresses();//assigns the addresses from hps to our global addresses, returns memory handle
int update_read_hps(); //updates our global variables with hps avalon values using global addresses
int request_read_fifo();//the fpga will read channels upto 7, with fifo full, and then pause to service the request
int reset_read_fifo();
int print_hps_variables();//requires previous update
uint8_t get_read_status();
int save_fifo_data(uint32_t fifo[]);
int cross_correlate(double target[],uint32_t fifo[]);
int hps_write_dsp(int hps_dsp_byte32, int32_t fpga_threshold);//update the fpga via hps

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
#define _GNU_SOURCE

#define FIFO_READOUT_COUNT	256

#define TARGET_ARRAY_SIZE	16
//fpga fast readout takes FIFO_READOUT_COUNT*fast_read_clk = 0.0013s
#define DTUS 				 20    //5.12 us is the clock time of the fpga fast_read_clk, which is 1/4 of the adc_clk from PLL_divided
#define LOOP_COUNT_MAX		200 //  0.243*8/(0.001*10) is a limit to how long to look for  fifo read status
#define N (FIFO_READOUT_COUNT/8) //number of adc frames in fifo
#define TG 					16	//number of hps target.dat values 
#define TARGET_THRESHOLD	15  //used by hps_adc_dsp to detect target
#define FPGA_DSP_THRESHOLD  190000 //used by FPGA to detect target 
#define FPGA_ADC_FRAME_TIME 1900 //ms it takes to get 8 adc channels at PLL 0.39 MHz

//global hps address pointers and variables that receive data from fpga
void *virtual_base;
uint8_t * h2p_key_byte_addr;  //switches
uint8_t key_byte=0x3;
	
uint8_t * h2p_lw_dipsw_addr;  //switches
uint32_t dipsw_mask;

uint8_t * fifo_usedw32_addr;//fifo
uint32_t fifo_usedw32;
uint32_t previous_fifo_used;//used to keep sycnchronized with fifo during read
	
uint8_t * hps_word_addr;//fifo output q
uint32_t hps_word;

uint8_t * hps_read_status_addr; //fpga ready to send fifo status to hps
uint8_t  hps_read_status;

uint8_t * hps_fifo_wrfull_addr;
uint8_t  hps_fifo_wrfull;

uint8_t * hps_read_clk_byte_addr;//hps to fifo read clock
uint8_t  hps_read_clk_byte;
	
uint8_t * hps_read_rq_byte_addr;//hps request to read fifo
uint8_t  hps_read_rq_byte;
	
uint8_t * fpga_dsp_byte_addr;//byte sent to Azure via Python
uint8_t  fpga_dsp_byte;
//the following addresses are to be written to by hps	
uint8_t * hps_dsp_byte_addr;//hps notifies fpga of dsp status
	
uint8_t * hps_dsp_threshold_addr;//sets fpga dsp threshold
int32_t hps_dsp_threshold; //for verifying we set it in fpga

uint8_t * fpga_cc_out_addr;// from the fpga, 32 bits of cross correlation
uint32_t fpga_cc_out;
	
//declare an array of the frame cross_correlations to be accessible to save_fifo_data
double cross_corr_array[N-TG+1]={0};
//define a global set of means (may, mpz are not used but shown here for reference)
double maz=1563.51, max=1335.84,may = 1650, mpz = 2732 ,mpy=2706.6, mpx=2762.42,  mwz=2231.83, mwy=2221.06;
//declare standard deviations,
double saz=738.293, sax=719.648,                       spy=2271.14, spx=5225.74,  swz=1170.26, swy=1533.95;
//end of global variables

int clk(unsigned int dtus){ //sends a single clock low, clock high with duration 2 * dtus
	*hps_read_clk_byte_addr = 0x00;
	usleep( dtus );
	*hps_read_clk_byte_addr = 0xff;//end high
	usleep( dtus );
	return 0;
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
        //printf("%d:Retrieved line of length %zu:  %s\n",i, read, line);
		target[i] = atof(line);//returns double precision
		i++;
		if(i >= TARGET_ARRAY_SIZE)
			break;
    }
    fclose(fp);
    if (line)
        free(line);
	printf("loaded target.dat..\n");
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
	h2p_key_byte_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + BUTTON_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_dipsw_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DIPSW_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_read_clk_byte_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_READ_CLK_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_read_rq_byte_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_READ_RQ_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_dsp_byte_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_DSP_BYTE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_dsp_threshold_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_DSP_THRESHOLD_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	
	fpga_dsp_byte_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FPGA_DSP_BYTE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_read_status_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_READ_STATUS_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_fifo_wrfull_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_FIFO_WRFULL_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	hps_word_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + HPS_WORD16_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	fifo_usedw32_addr = (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + FIFO_USEDW32_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	fpga_cc_out_addr =  (uint8_t *) virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + CC_OUT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	return fd;//to be closed on exit
}

int update_read_hps(){ //updates our global variables with hps avalon values using global addresses
	key_byte = *(uint8_t*) h2p_key_byte_addr;
	dipsw_mask = *(uint32_t *) h2p_lw_dipsw_addr;
	hps_read_clk_byte = *(uint8_t*) hps_read_clk_byte_addr;//need to write to adddress location..
	hps_read_rq_byte = *(uint8_t*) hps_read_rq_byte_addr;//need to write to addresss...
	hps_read_status = *(uint8_t*) hps_read_status_addr;
	fpga_dsp_byte = *(uint8_t*) fpga_dsp_byte_addr;
	hps_fifo_wrfull = *(uint8_t*) hps_fifo_wrfull_addr;
	hps_word = *(uint32_t*) hps_word_addr;
	fifo_usedw32 =  *(uint32_t*) fifo_usedw32_addr;
	fpga_cc_out = *(uint32_t*) fpga_cc_out_addr;
	hps_dsp_threshold = *(int32_t*) hps_dsp_threshold_addr;
	return 0;
}

int request_read_fifo(){//the fpga will read channels upto 7, with fifo full, and then pause to service the request
	*hps_read_rq_byte_addr = 0xff; //put a byte into hps_read_rq_byte_addr
	*hps_read_clk_byte_addr = 0xff; //initialize hi
	return 0;
}

int reset_read_fifo(){
	*hps_read_rq_byte_addr = 0x00;
	return 0;
}

int print_hps_variables(){//requires previous update
	printf("dipsw=%d,  read_status=%d, hps_word=%d, read_rq_byte=%d, read_clk_byte=%d, fifo_used %d, wrfull %d, key %d\n", dipsw_mask, hps_read_status,
				 hps_word, hps_read_rq_byte, hps_read_clk_byte,fifo_usedw32,hps_fifo_wrfull,key_byte);
	return 0;
}

uint8_t get_read_status(){
	uint8_t ready = 0;
	hps_read_status = *(uint8_t*) hps_read_status_addr;
	hps_fifo_wrfull = *(uint8_t*) hps_fifo_wrfull_addr;
	ready = (uint8_t) (hps_read_status==0x1 && hps_fifo_wrfull==0x1);
	return ready;
}

int save_fifo_data(uint32_t fifo[]){
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
	   return 1;
    }
	for(i=0;i<N;i++){ //print out what we got, 8 channels at a time//FIFO_READOUT_COUNT/8
		//printf("i%d %d %d %d %d %d %d %d %d\n",i, fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
		fprintf(fptr,"%d %d %d %d %d %d %d %d %d\n",i,fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
	}
	fclose(fptr);
	printf ( "saved fifo file name: %s\n", buff );

	//do cross_corr_array.dat
	if ((fptr = fopen("cross_corr.dat","w")) == NULL){
       printf("Error! opening file");
	   return 1;
    }
	for(i=0;i<(N-TG+1);i++){
		fprintf(fptr,"%d %f\n",i,10*cross_corr_array[i]);
	}
	fclose(fptr);
	
	//data to corr.dat
	if ((fptr = fopen("corr.dat","w")) == NULL){
       printf("Error! opening file");
	   return 1;
    }
	for(i=0;i<N;i++){ //print out what we got, 8 channels at a time//FIFO_READOUT_COUNT/8
		//printf("i%d %d %d %d %d %d %d %d %d\n",i, fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
		fprintf(fptr,"%d %d %d %d %d %d %d %d %d\n",i,fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
	}
	fclose(fptr);
	i--;//to index the last line
	printf("last values..\ni%d %d %d %d %d %d %d %d %d\n",i, fifo[8*i+0],fifo[8*i+1],fifo[8*i+2],fifo[8*i+3],fifo[8*i+4],fifo[8*i+5],fifo[8*i+6],fifo[8*i+7]);
	return 0;
}

int cross_correlate(double target[],uint32_t fifo[]){
	int target_detected = 0;
	int i;
	//declare 6 arrays out of 8 channels
	double Az[N],Ax[N], Wz[N], Wy[N], Px[N], Py[N];
	double corrA[N], corrW[N], corrP[N],corrAll[N];
	double cm = 20480; //scales target correlations independent of N
	//fill 6 channels using fifo
	for(i=0; i < N; i++){
		Az[i] = (float) fifo[i*8+0];
		Ax[i] = (float) fifo[i*8+1];
		Wz[i] = (float) fifo[i*8+6];
		Wy[i] = (float) fifo[i*8+7];
		Px[i] = (float) fifo[i*8+5];
		Py[i] = (float) fifo[i*8+4];
	}
	#ifdef CALCULATE_STATISTICS
	printf("Calculating means and standard deviations..\n");
	//calculate means
	double maz=0, max=0, mwz=0, mwy=0, mpx=0, mpy=0;
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
	#endif
	//create cross-correlations of acceleration, rates and pressures
	for(i=0; i < N; i++){
		corrA[i] = (Az[i] - maz)* (Ax[i] - max) / (saz * sax);
		corrW[i] = (Wz[i] - mwz)* (Wy[i] - mwy) / (swz * swy);
		corrP[i] = (Px[i] - mpx)* (Py[i] - mpy) / (spx * spy);
		corrAll[i] = cm * corrA[i] * corrW[i] * corrP[i];
	}	
	int j;
	//run over i values up to N, correlating target values up i = N - TG
	for(i = 0; i < (N-TG+1); i++){
		cross_corr_array[i] = 0;
		for(j = 0; j < TG; j++){
			cross_corr_array[i] += target[j] * corrAll[i + j];
		}
		if( cross_corr_array[i] > TARGET_THRESHOLD){
			target_detected = 1;
			printf("Target detected! i = %d, cross-correlation %f\n", i,cross_corr_array[i]);
		}
	}
	return target_detected;
}

int hps_write_dsp(int hps_dsp_byte32,  int32_t fpga_threshold){//update the fpga via hps
	uint8_t dsp_byte = (uint8_t) hps_dsp_byte32;
	//printf("hps_dsp_byte_addr is %x ", *hps_dsp_byte_addr);
	*hps_dsp_byte_addr = dsp_byte;
	//set the threshold - signed, recast the uint8_t address to signed 32
	*(int32_t *)hps_dsp_threshold_addr = (int32_t) fpga_threshold;
	return 0;
}

int main(int argc, char** argv){
	int continuously = 0,i=0;
	int fpga_dsp_threshold = FPGA_DSP_THRESHOLD;
	for(i = 0; i < argc; i++)
	{
		if(strcmp( argv[i], "-h")==0){
			printf("options:\n\
			- no options, runs until a target is detected\n\
			-h usage\n\
			-c run continuously\n\
			-t <threshold>\n\
			- SW[3] = 0 runs fpga dsp\n");
			return 0;
		}else if(strcmp(argv[i] , "-c")==0){
			continuously = 1;
			printf("running continuously.. press PB button to quit\n");
		}else if(strcmp(argv[i] , "")==0){
			printf("running until target detected..\n");
		}else if(strcmp(argv[i] , "-t")==0){
			printf("setting threshold to %s\n",argv[2]);
			fpga_dsp_threshold = atoi(argv[2]);
		}
	}
	//timing variables
	struct timespec start, request, stop;
	double accum,accumrq;
	int fd;//hps address handle to be closed on exit
	int loop_count;
	int fifo_readout_count = 0; //set when adc_measured_count==0 //general counter
	int target_detections = 0;
	uint32_t fifo[FIFO_READOUT_COUNT]={0};//our fifo readout
	int good_frames=0;
	int tdetected=0;
	int dsp_byte_countdown = 0;
	//load the target filter data
	double target[TARGET_ARRAY_SIZE];
	if(get_target_data(target)==0)
		printf("TARGET_ARRAY_SIZE %d, target[0] %f, target[TARGET_ARRAY_SIZE-1] %f\n",TARGET_ARRAY_SIZE, target[0],target[TARGET_ARRAY_SIZE-1]);

	fd = get_hps_addresses();//required before updating the global variables
	update_read_hps();//updates our global variables with what hps currently has over the avalon bus
	print_hps_variables();//requires previous update
	//check if display sw 3 is set high for hps dsp detection
	if(dipsw_mask<8){ //fpga dsp detection
		printf("hps_dsp_threshold in fpga is %d\n",*(int32_t*) hps_dsp_threshold_addr);
		hps_write_dsp(123,fpga_dsp_threshold);
		printf("Most significant SW[3] = 0\nFPGA detection threshold %d. Set SW[3] on for HPS detection.\n",*(int32_t*) hps_dsp_threshold_addr);
		while(key_byte==3 && dipsw_mask< 8){
			if(fpga_dsp_byte == 255 ){
				if(!tdetected){ //only do this once per change
					tdetected = 1;
					target_detections++;
					printf("target_detections %d, fpga_dsp_byte %d, fpga_cc_out %d\n",target_detections, fpga_dsp_byte, fpga_cc_out);
				}
			}else if(fpga_dsp_byte == 123){
				if(tdetected) { //only do this once per change
					tdetected = 0;//reset since fpga_dsp_byte is low
					printf("No target - fpga_dsp_byte %d\n", fpga_dsp_byte);
				}
			}
			update_read_hps();
			usleep(FPGA_ADC_FRAME_TIME);				
		}
	}else{ //hps dsp detection
		fpga_dsp_threshold = 0; 
		hps_write_dsp(123,fpga_dsp_threshold);
	}
	//CONTINUOUSLY READ FIFO BLOCKS, AND ASSESS FOR CORRELATION
	while(key_byte==3 && dipsw_mask>=8){//for(i=0;i<3;i++){
		//wait for fifo to fill again
		//printf("waiting to fill fifo..\n");
		while(!hps_fifo_wrfull){
			usleep( DTUS );
			hps_fifo_wrfull = *(uint8_t*) hps_fifo_wrfull_addr;
		}
		clock_gettime( CLOCK_REALTIME, &start);//start timer			 
		fifo_readout_count=0;//reset this fifo_readout_count=0;//reset this
		request_read_fifo(); //sets the hps_read_rq_byte, sets hps_read_clk_byte high
		//fpga latches the request while reading channels in sequence.
		//readout begins when the last adc measured is channel 7, with fifo aligned and full
		loop_count = 0;//exit if we never get a full fifo
		while( loop_count < LOOP_COUNT_MAX ) {//upto 8 adc measurement delay to get fifo aligned.
			loop_count++;
			update_read_hps();
			if(get_read_status()){ //here comes fifo 
				previous_fifo_used = fifo_usedw32;//we will look for changes to know we have the next value
				clk(DTUS);//cycle the hps_read_clk
				//fifo_used should change to FIFO_WORDS-1
				reset_read_fifo();//*hps_read_rq_byte_addr = 0x00 -set read request low since acknowleged
				clock_gettime( CLOCK_REALTIME, &request);
				while(fifo_readout_count < FIFO_READOUT_COUNT && hps_read_status==1 ){ //fast polling
					update_read_hps();
					//printf("i:%d, dipsw=%d,  hps_read_status=%d, hps_word=%d, hps_read_rq_byte=%d, hps_read_clk_byte=%d, fifo_used %d\n", 
						//fifo_readout_count,dipsw_mask, hps_read_status,
						//hps_word, hps_read_rq_byte, hps_read_clk_byte,fifo_usedw32);
					if(fifo_usedw32 != previous_fifo_used){
						fifo[fifo_readout_count] = hps_word;
						previous_fifo_used = fifo_usedw32;
						fifo_readout_count++;
					}
					previous_fifo_used = fifo_usedw32;;
					clk(DTUS);//cycle the hps_read_clk
				}
				//when we have fifo
				clock_gettime( CLOCK_REALTIME, &stop);
				accum = ( stop.tv_sec - start.tv_sec )+ ( stop.tv_nsec - start.tv_nsec )/ 1000000000.0;
				accumrq = ( request.tv_sec - start.tv_sec )+ ( request.tv_nsec - start.tv_nsec )/ 1000000000.0;
				good_frames++;//keep track of how many frames
				tdetected = cross_correlate(target,fifo);
				update_read_hps();
				if(tdetected){
					target_detections++;
					hps_write_dsp(255, 0);//TBD
					update_read_hps();
					printf("tdetected %d, target_detections %d, fpga_dsp_byte %d\n",tdetected,target_detections, fpga_dsp_byte);
					dsp_byte_countdown = 30;//about 61 loops /sec for fifo = 256, frames = 32
					//save_fifo_data(fifo);
					//break;
				}else{
					update_read_hps();
					if(dsp_byte_countdown <= 0){
						hps_write_dsp(123, 0);
					}else{
						hps_write_dsp(255, 0);
						dsp_byte_countdown--;
						//printf("dsp_byte_countdown %d\n",dsp_byte_countdown);
					}
				}
				break;
			}else{//waiting for hps_read_status
				usleep( DTUS );//
			}
		}
		//break comes here
		if((target_detections > 0) || (key_byte < 3) ){
			if(!continuously)
				break;//get out of the while() loop
		}
		if(loop_count > LOOP_COUNT_MAX){
			printf("loop_count %d exceeded LOOP_COUNT_MAX!.  Check fpga.\n", loop_count);
			break;
		}
		//hps_write_dsp(good_frames, 0);//KEEP CHANGING hps_dsp_byte in fpga, which changes fpga_dsp_byte
	}
	//summarize 
	print_hps_variables();
	printf( "Good frames read %d, target detected %d, dtus %d, request %lfs, readout %lfs,\n", good_frames,
		target_detections,DTUS,accumrq,accum-accumrq );

	save_fifo_data(fifo);
	
	// clean up our memory mapping and exit
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	close( fd );
	return( 0 );
}
