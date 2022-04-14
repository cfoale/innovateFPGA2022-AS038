/*
adc_dac.cpp adapted by C.M. Foale 2022 from DC934A.cpp  and DC2025a.ino
Byte order was changed in SpiWrapper.cpp to conform with the LTC2668 datasheet
Adaption copyright (c) 2022 C.M.Foale
Copyright (c) 2021, Mihai Ursu
*/


#include <string>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <Arduino.h>
#include <SPI.h>
#include <SpiWrapper.h>
#include <Wire.h>
#include <WireWrapper.h>
#include <De10Nano.h>


#include <LT_I2C.h>
#include <QuikEval_EEPROM.h> 
#include <LT_SPI.h>
#include <LTC2668.h>

#define FIFO_READOUT_COUNT	8192
#define FIFO_FRAME_COUNT   (FIFO_READOUT_COUNT*9/8)
#define MEASUREMENT_TIME 	800 //units are us. goal is a readout time of 1900us
#define SPI_TRANSACTION_TIME 6  //us - 6us at 5Mhz sending 32 bits (logic analyzer)

//Macros
#define REF_EXTERNAL    LTC2668_REF_DISABLE     //!< External mode 
#define REF_INTERNAL    0                       //!< Internal mode

// Function Declaration
int get_data(double frame[]);//read 9 float values per line
int set_dac(double d[]); //set the 8 float values to the dac
int get_next_data_filename(char* name, int ln); //reads the selected_data_files.txt to get the name of the next line ln
// Global variable
static DemoBoardType detectedBoard; //cmf - this is defined in LinduinoWrapper/QuickEval_EEPROM.h

//! Used to manipulate EEPROM data.
union eeprom_data_union
{
  struct data_struct_type               //! EEPROM data structure
  {
    int16_t cal_key;                    //!< The key that keeps track of the calibration
    uint8_t soft_span_range[16];        //!< SoftSpan range
    uint16_t dac_code_a[16];            //!< DAC Register A
    uint16_t dac_code_b[16];            //!< DAC Register B
    uint16_t toggle_word;               //!< Toggle control word
    uint8_t global_toggle_bit;          //!< Global toggle bit
    uint8_t mux_state;                  //!< Multiplexer address AND enable bit
    uint8_t reference_mode;             //!< Int. reference may be disabled in external reference mode (not required)
  } data_struct;                        //!< Name of structure

  char byte_array[sizeof(data_struct_type)]; //!< Array used to store the structure
};

eeprom_data_union eeprom; // Create union

// Constants

//! Used to keep track to print voltage or print code
enum
{
  PROMPT_VOLTAGE = 0, /**< 0 */
  PROMPT_CODE = 1     /**< 1 */
};

int verbose = false;//controls printf's

int main(int argc, char** argv){
	bool loop_data_files = false;
	for(int i = 0; i < argc; i++)
	{
		if(strcmp( argv[i], "-h")==0){{
			printf("options:\n - no options, runs once using files in 'selected_data_files.txt'\n -h usage\n -l loop data files\n -lv loop data verbose");
			return 0;
		}
		}else if(strcmp(argv[i] , "-l")==0){
			loop_data_files = true;
			printf("looping data files..hit Control C to quit\n");
		}else if(strcmp(argv[i] , "-lv")==0){
			printf("looping data files verbosely..hit Control C to quit\n");
			verbose = true;
		}
	}
		
    De10Nano* de10 = De10Nano::getInstance();
    if( de10 ){

        de10->setLtcGpioDirection( De10Nano::DIO_DIRECTION_OUTPUT );
        de10->setLtcGpioStatus( De10Nano::DIO_STATUS_LOW );
        quikeval_I2C_init();          // Configure the EEPROM I2C port for 100kHz
        WireWrapper wire = quikeval_I2C_get_wire();

        SPISettings spiSettings = SPISettings( 5000000, MSBFIRST, SPI_MODE1 );//cmf - SpiWrapper has SPI_MODE1 by default
        quikeval_SPI_apply_settings( spiSettings ); //1Mhz results in a 30us 32 bit DAC transaction
        
        if(!quikeval_SPI_init())
			printf("Unable to configure SPI!\n");          // Configure the spi port (1MHz SCK set by default in De10Nano.cpp)
		quikeval_SPI_connect();       // Connect SPI to main data port	
        char expectedBoardName[] = "DC2025A";  // Demo Board Name stored in QuikEval EEPROM
        if(!quikEvalEepromDiscoverDemoBoard( wire, expectedBoardName, &detectedBoard )){
            printf( "\nCould not find a %s board connected", expectedBoardName );
			return 0;
		}
        printf( "\nConnected to %s\n", detectedBoard.demoCircuitNumber );
            
		//static  uint8_t selected_dac = 0;     // The selected DAC to be updated (0=CH0, 1=CH1 ... 16=All).  Initialized to CH0.
		//Testing.....
		//uint16_t dac_code= 0x0;
		//LTC2668_write(LTC2668_CS,LTC2668_CMD_WRITE_N_UPDATE_N, selected_dac, dac_code); // Send dac_code
		
		double frame[FIFO_FRAME_COUNT];
		if(verbose) printf("FIFO_FRAME_COUNT = %d\n", FIFO_FRAME_COUNT);
		if(loop_data_files)
			while(true)
				get_data(frame); 
		else
			get_data(frame);
    }
    return 0;
}

int get_next_data_filename(char* name, int ln){ //reads the selected_data_files.txt to get the name of the next line ln
	FILE * fp;
	fp = fopen("selected_data_files.txt", "r");
    if (fp == NULL){
		printf("Unable to open selected_data_files.txt\n");
        return 1;
	}
	int ret=0;
	int i=0;
	while(ret != EOF){
		ret= fscanf(fp,"%s^\n",name); //get the first line upto \n
		//printf("i= %d, ret = %d get_next_data_filename found\n%s\n",i,ret,name);
		if(ret == -1)
			break;
		if(i==ln){
			//printf("i= %d=ln= %d,get_next_data_filename found\n%s\n",i,ln,name);
			break;
		}
		i++;	
	}
	fclose(fp);
	if(i==ln && ret != -1)
		return (ln +1);
	else
		return -1;
}

int get_data(double frame[]){//read 9 float values per line
	//timing variables
	struct timespec start, request, stop;
	double accum,accumrq;
	FILE * fp;
    double value;
	double d[8];//8 channel measurement from a line in the file to be sent to dac
	int i=0;
	char name[256]; //big string
	int next = 0;
	next = get_next_data_filename(name,next);
	while(next != -1){
		clock_gettime( CLOCK_REALTIME, &start);//start timer
		i=0;
		printf("file %d %s\n",next-1, name);
		
		fp = fopen(name, "r");
		if (fp == NULL){
			printf("Unable to open %s\n",name);
			return 1;
		}
		while (fscanf(fp,"%lf",&value)!=EOF) {
			//printf("%d:%f\n",i, value);
			frame[i] = value;//returns double precision
			i++;
		}
		fclose(fp);   
		//printf("loaded %s..\n",name);
	
		clock_gettime( CLOCK_REALTIME, &request);
		accumrq = ( request.tv_sec - start.tv_sec )+ ( request.tv_nsec - start.tv_nsec )/ 1000000000.0;
		clock_gettime( CLOCK_REALTIME, &start);//start timer
		for( i=0;i<FIFO_FRAME_COUNT/9;i++){ //print out what we got, 9 channels at a time
		//printf("%f %f %f %f %f %f %f %f %f\n",frame[9*i+0],frame[9*i+1],frame[9*i+2],
			//frame[9*i+3],frame[9*i+4],frame[9*i+5],frame[9*i+6],frame[9*i+7],frame[9*i+8]);
			d[0] = frame[9*i+1];
			d[1] = frame[9*i+2];
			d[2] = frame[9*i+3];
			d[3] = frame[9*i+4];
			d[4] = frame[9*i+5];
			d[5] = frame[9*i+6];
			d[6] = frame[9*i+7];
			d[7] = frame[9*i+8];
			set_dac(d);
			usleep(MEASUREMENT_TIME );
		}
		clock_gettime( CLOCK_REALTIME, &stop);
		accum = ( stop.tv_sec - start.tv_sec )+ ( stop.tv_nsec - start.tv_nsec )/ 1000000000.0;
		i--;
		if(verbose) printf("last line of data\n%f %f %f %f %f %f %f %f %f\n",frame[9*i+0],frame[9*i+1],frame[9*i+2],
			frame[9*i+3],frame[9*i+4],frame[9*i+5],frame[9*i+6],frame[9*i+7],frame[9*i+8]);
		if(verbose)  printf( "Frames read %d, SPI transaction time %dus, adjusted measurement time %dus, request %lfs, readout %lfs,\n", i
			,SPI_TRANSACTION_TIME,MEASUREMENT_TIME, accumrq,accum/(float)i);
			
		next = get_next_data_filename(name,next);//get the next filename
	}
	return 0;
}

int set_dac(double ch[]){ //set the 8 float values to the dac
	uint16_t dac_code;
	double V[8];//volts
	double cal = 0.8175;
	for(int i=0;i<8;i++){
		V[i] = cal * 5.0*(ch[i]/4096.0);
		dac_code = LTC2668_voltage_to_code(V[i],
                                   LTC2668_MIN_OUTPUT[eeprom.data_struct.soft_span_range[i]],
                                   LTC2668_MAX_OUTPUT[eeprom.data_struct.soft_span_range[i]]);
		// Store dac_code to register variable a
		eeprom.data_struct.dac_code_a[i] = dac_code;
		LTC2668_write(LTC2668_CS, LTC2668_CMD_WRITE_N, i, dac_code);
		usleep(SPI_TRANSACTION_TIME);//at 5Mhz, the transaction is 6us.  So wait that long between transactions
	}
	LTC2668_write(LTC2668_CS,LTC2668_CMD_UPDATE_ALL, 0, 0);
	//printf("%f %f %f %f %f %f %f %f\n",V[0],V[1],V[2],V[3],V[4],V[5],V[6],V[7]); //remember printf slows down timing
	return 0;
}


