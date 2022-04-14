/*
DC934A.cpp

This file contains the sources for the LTC Demo Circuit board DC934A.
It demonstrates the usage in Linux of the unmodified Arduino drivers for
the LTC2607 DAC and LTC2422 ADC, with the help of I2C and SPI wrappers.


Copyright (c) 2021, Mihai Ursu

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <string>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <Arduino.h>
#include <De10Nano.h>
#include <LT_I2C.h>
#include <LT_SPI.h>
#include <QuikEval_EEPROM.h>
#include <LTC2607.h>
#include <LTC2422.h>


enum : uint8_t
{
    DAC_CH_A,
    DAC_CH_B,
    DAC_CH_BOTH,

    DAC_COUNT
};

enum : uint8_t
{
    ADC_CH_0,
    ADC_CH_1,

    ADC_COUNT
};


//! ADC conversion duration specified in datasheet (<= 136.20 ms)
static const uint8_t LTC2422_CONV_DURATION_MS = 137;

//! Look-up table for DAC_A, DAC_B, or both.
//! Builds the command byte for the DAC address.
static const uint8_t ADDRESS_MAP[DAC_COUNT] =
{
    LTC2607_DAC_A,
    LTC2607_DAC_B,
    LTC2607_ALL_DACS
};


char getche();
void printTitle();
void printPrompt( uint8_t selectedDac );

uint16_t getVoltage( double lsb, int32_t offset );

void menu1SelectDac( uint8_t* selectedDac );
uint8_t menu2WriteToInputRegister( uint8_t selectedDac );
uint8_t menu3WriteAndUpdateDac( uint8_t selectedDac );
uint8_t menu4UpdatePowerUpDac( uint8_t selectedDac );
uint8_t menu5PowerDownDac( uint8_t selectedDac );
void menu6ReadAdc();
uint8_t menu7Sweep();
void menu8CalibrateAll();


//! Common reference voltage for the DAC and ADC
static double vRef = 2.5;

//! Voltage equivalent to the LTC2607 least significant bit
static double ltc2607Lsb = vRef / ( 1 << 16 );
static double ltc2607LsbArray[DAC_COUNT] = { ltc2607Lsb, ltc2607Lsb, ltc2607Lsb };

//! Array with LTC2607 offset variables. Can be calibrated through the option menu.
static int32_t ltc2607OffsetArray[DAC_COUNT] = { 0 };

//! Voltage equivalent to the LTC2422 least significant bit
static double ltc2422Lsb = vRef / ( 1 << 20 );


int main()
{
    De10Nano* de10 = De10Nano::getInstance();

    if( de10 )
    {
        //**********************
        // GPIO
        //**********************
        de10->setLtcGpioDirection( De10Nano::DIO_DIRECTION_OUTPUT );
        de10->setLtcGpioStatus( De10Nano::DIO_STATUS_HIGH );

        //**********************
        // I2C
        //**********************
        quikeval_I2C_init();
        WireWrapper wire = quikeval_I2C_get_wire();

        //**********************
        // SPI
        //**********************
        // settings for LTC2422
        // datasheet: fESCK < 2000 kHz
        SPISettings spiSettings = SPISettings( 1000000, MSBFIRST, SPI_MODE0 );
        quikeval_SPI_apply_settings( spiSettings );
        bool spiInit = quikeval_SPI_init();

        char expectedBoardName[] = "DC934";
        DemoBoardType detectedBoard;
        bool dc934Connected = quikEvalEepromDiscoverDemoBoard( wire, expectedBoardName, &detectedBoard );

        if( !dc934Connected )
        {
            printf( "\nCould not find a %s board connected", expectedBoardName );
        }
		dc934Connected = True;
        if( spiInit && dc934Connected )
        {
            printTitle();

            printf( "\nConnected to %s", detectedBoard.demoCircuitNumber );

            if( detectedBoard.demoCircuitOption )
            {
                printf( "-%c", detectedBoard.demoCircuitOption );
            }

            printf( ", Product Name is %s", detectedBoard.productName );

            uint8_t selectedDac = DAC_CH_BOTH;

            printf( "\n\nV_REF is currently set to %.3lf V and should match the real voltage applied"
                      "\nto the board. Do you want to change it? [y/n]", vRef );

            if( 'y' == tolower( getche() ) )
            {
                char str[10] = "";
                int fields = 0;

                do
                {
                    printf( "\n\nEnter new V_REF [0-5]: ");
                    fields = scanf( "%s", str );
                    vRef = atof( str );
                }while( vRef < 0 || vRef > 5 || ( 1 != fields ) );
            }

            ltc2607Lsb = vRef / ( 1 << 16 );
            ltc2422Lsb = vRef / ( 1 << 20 );

            for( uint8_t i = 0; i < DAC_COUNT; i++ )
            {
                ltc2607LsbArray[i] = ltc2607Lsb;
            }

            printf( "\nContinuing with V_REF = %.3lf V", vRef );
            printf( "\nLTC2607 lsb =  %.3lf uV", 1.e6 * ltc2607Lsb );
            printf( "\nLTC2422 lsb =  %.3lf uV", 1.e6 * ltc2422Lsb );

            for(;;)
            {
                printPrompt( selectedDac );               
                char cmd = getche();

                switch( cmd )
                {
                    case '1':
                        // Select a DAC to update
                        menu1SelectDac( &selectedDac );
                        break;

                    case '2':
                        // Write to input register only
                        menu2WriteToInputRegister( selectedDac );
                        break;

                    case '3':
                        menu3WriteAndUpdateDac( selectedDac );
                        break;

                    case '4':
                        // Update/Power up DAC
                        menu4UpdatePowerUpDac( selectedDac );
                        break;

                    case '5':
                        // Power down DAC
                        menu5PowerDownDac( selectedDac );
                        break;

                    case '6':
                        menu6ReadAdc();
                        break;

                    case '7':
                        menu7Sweep();
                        break;

                    case '8':
                        menu8CalibrateAll();
                        break;

                    case '9':
                        printf( "\n\nNormal exit.\n" );
                        exit( 0 );
                        break;

                    default:
                        printf( "\n%c - Incorrect Option\n", cmd );
                        break;
                }
            }
        }
    }

    printf( "\n\n" );
    return 0;
}


//!************************************************************************
//! Gets a character from keyboard
//!
//! @returns the character
//!************************************************************************
char getche()
{
    char buf = 0;
    struct termios old;
    fflush( stdout );

    if( tcgetattr( 0, &old ) >= 0 )
    {
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;

        if( tcsetattr( 0, TCSANOW, &old ) >= 0 )
        {
            if( read( 0, &buf, 1 ) >= 0 )
            {
                old.c_lflag |= ICANON;
                old.c_lflag |= ECHO;
                tcsetattr( 0, TCSADRAIN, &old );

                printf( "%c", buf );
            }
        }
    }

    return buf;
}


//!************************************************************************
//! Print the title
//!
//! @returns nothing
//!************************************************************************
void printTitle()
{
    printf( "\n" );
    printf( "*****************************************************************\n" );
    printf( "* DC934A Demonstration Program                                  *\n" );
    printf( "*                                                               *\n" );
    printf( "* This program demonstrates communication with the LTC2607      *\n" );
    printf( "* 16-Bit Dual Rail-to-Rail DAC with I2C Interface. This board   *\n" );
    printf( "* also features an LTC2422 2-Channel 20-Bit uPower No Latency   *\n" );
    printf( "* Delta Sigma ADC for readback.                                 *\n" );
    printf( "*****************************************************************\n" );
}


//!************************************************************************
//! Print the main menu and prompt for an input command
//!
//! @returns nothing
//!************************************************************************
void printPrompt
    (
    uint8_t selectedDac     //!< the selected DAC
    )
{
    printf( "\n\nCommand Summary:\n" );
    printf( "  1 - Select DAC\n" );
    printf( "  2 - Write to input register (no update)\n" );
    printf( "  3 - Write and update DAC\n" );
    printf( "  4 - Update/Power up DAC\n" );
    printf( "  5 - Power Down DAC\n" );
    printf( "  6 - Read ADCs\n" );
    printf( "  7 - Sweep\n" );
    printf( "  8 - Calibrate ALL\n" );
    printf( "  9 - Exit program\n" );
    printf( "\n  Selected DAC: " );

    if( DAC_CH_BOTH == selectedDac )
    {
        printf( "Both\n" );
    }
    else
    {
        printf( "%c\n", selectedDac + 'A' );
    }

    printf( "\nEnter a command: " );
}


//!************************************************************************
//! Read desired DAC output voltage from user input.
//! Check min & max ranges before calling LTC2607_voltage_to_code().
//!
//! @returns the code that can be written to the LTC2607 DAC
//!************************************************************************
uint16_t getVoltage
    (
    double  lsb,        //!< the DAC lsb
    int32_t offset      //!< the DAC offset
    )
{
    uint16_t code = 0;
    double dacVoltage = 0;
    char str[10] = "";
    int fields = 0;

    do
    {
        printf( "\n\nEnter the desired DAC output voltage [0-%.3lf]: ", vRef);
        fields = scanf( "%s", str );
        dacVoltage = atof( str );
    }while( dacVoltage < 0 || dacVoltage > vRef || ( 1 != fields ) );

    // Function LTC2607_voltage_to_code() contains in its body a type cast like
    // dac_code = (uint16_t) (float_code);
    // , which means that overflow occurs if float_code is > 65535, or underflow
    // if negative.
    //
    // Variable dac_code will not be larger than 65535 or negative after this line.
    //
    // The flow will never go through the last two if() statements in function
    // LTC2607_voltage_to_code(), and range conditions are required before calling it.

    double maxAlowedDacVoltage = lsb * ( offset + 65535 );
    double minAlowedDacVoltage = lsb * offset;

    if( dacVoltage > maxAlowedDacVoltage )
    {
        dacVoltage = maxAlowedDacVoltage;
    }

    if( dacVoltage < minAlowedDacVoltage )
    {
        dacVoltage = minAlowedDacVoltage;
    }

    code = LTC2607_voltage_to_code( dacVoltage, lsb, offset );
    printf( "For %.6lf V the code is 0x%04X", dacVoltage, code );
    return code;
}


//!************************************************************************
//! Select DAC to update. Prompts user for DAC A, DAC B, or both.
//! This function does not communicate with the LTC2607.
//!
//! @returns nothing
//!************************************************************************
void menu1SelectDac
    (
    uint8_t* selectedDac    //!< the selected DAC
    )
{
    printf( "\n\nSelect DAC to be updated (0=A, 1=B, 2=Both) " );
    uint8_t selection = getche() - '0';

    // if user enters and invalid option, default to both
    if( selection > DAC_CH_BOTH )
    {
        selection = DAC_CH_BOTH;
    }

    *selectedDac = selection;
    printf( "\nSelection -> " );

    if( DAC_CH_BOTH == *selectedDac )
    {
        printf( "Both DACs" );
    }
    else
    {
        printf( "DAC %c", *selectedDac + 'A' );
    }
}


//!************************************************************************
//! Write to input register only. Does not update the output voltage.
//!
//! @returns 0 on success, 1 otherwise
//!************************************************************************
uint8_t menu2WriteToInputRegister
    (
    uint8_t selectedDac     //!< the selected DAC
    )
{
    // Connects I2C port to the QuikEval connector
    quikeval_I2C_connect();

    uint16_t dacCode = getVoltage( ltc2607LsbArray[selectedDac],
                                   ltc2607OffsetArray[selectedDac] );

    // Write a code to the LTC2607 internal register. Output voltage is not updated by the "write" command.
    uint8_t ack = LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS,
                                 LTC2607_WRITE_COMMAND,
                                 ADDRESS_MAP[selectedDac],
                                 dacCode );
    return ack;
}


//!************************************************************************
//! Write to input register and update output voltage
//!
//! @returns 0 on success, 1 otherwise
//!************************************************************************
uint8_t menu3WriteAndUpdateDac
    (
    uint8_t selectedDac     //!< the selected DAC
    )
{
    // Connects I2C port to the QuikEval connector
    quikeval_I2C_connect();

    uint16_t dacCode = getVoltage( ltc2607LsbArray[selectedDac],
                                   ltc2607OffsetArray[selectedDac] );

    // Write a code to the LTC2607 internal register. Output voltage is not updated by the "write" command.
    uint8_t ack = LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS,
                                 LTC2607_WRITE_UPDATE_COMMAND,
                                 ADDRESS_MAP[selectedDac],
                                 dacCode );
    return ack;
}


//!************************************************************************
//! Update/Power Up DAC
//!
//! @returns 0 on success, 1 otherwise
//!************************************************************************
uint8_t menu4UpdatePowerUpDac
    (
    uint8_t selectedDac     //!< the selected DAC
    )
{
    // Connects I2C port to the QuikEval connector
    quikeval_I2C_connect();

    // Update output voltage from internal register. Data is ignored.
    uint8_t ack = LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS,
                                 LTC2607_UPDATE_COMMAND,
                                 ADDRESS_MAP[selectedDac],
                                 0 );
    return ack;
}


//!************************************************************************
//! Power Down DAC
//!
//! @returns 0 on success, 1 otherwise
//!************************************************************************
uint8_t menu5PowerDownDac
    (
    uint8_t selectedDac     //!< the selected DAC
    )
{
    // Connects I2C port to the QuikEval connector
    quikeval_I2C_connect();

    // Power down selected DAC. Data is ignored.
    uint8_t ack = LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS,
                                 LTC2607_POWER_DOWN_COMMAND,
                                 ADDRESS_MAP[selectedDac],
                                 0);
    return ack;
}


//!************************************************************************
//! Read voltage from the ADC
//!
//! @returns nothing
//!************************************************************************
void menu6ReadAdc()
{
    uint8_t adcChannel = 0;
    int32_t adcCode = 0;
    int32_t adcCodes[ADC_COUNT] = { 0 };
    double adcValues[ADC_COUNT] = { 0 };

    // Connect SPI to QuikEval connector
    quikeval_SPI_connect();

    // Throw out the stale data.
    LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode );
    // Read values are not used, just trigger the most recent conversion.
    delay( LTC2422_CONV_DURATION_MS );

    // Read first available conversion from the ADC. The ADC channel will toggle with each reading.
    LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode );
    adcCodes[adcChannel] = adcCode;
    delay( LTC2422_CONV_DURATION_MS );

    // Read second available conversion from the ADC.
    LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode );
    adcCodes[adcChannel] = adcCode;

    adcValues[ADC_CH_0] = LTC2422_code_to_voltage( adcCodes[ADC_CH_0], ltc2422Lsb );
    adcValues[ADC_CH_1] = LTC2422_code_to_voltage( adcCodes[ADC_CH_1], ltc2422Lsb );

    // On the DC934 board:
    // - CH-A of the DAC is connected to CH-1 of the ADC
    // - CH-B of the DAC is connected to CH-0 of the ADC
    // => Take into account the crossed channel assignments
    printf( "\n" );
    printf( "\nCH-A = %.6lf V", adcValues[ADC_CH_1] );
    printf( "\nCH-B = %.6lf V", adcValues[ADC_CH_0] );   
}


//!************************************************************************
//! Voltage Sweep
//!
//! @returns 0 on success, 1 otherwise
//!************************************************************************
uint8_t menu7Sweep()
{
    uint8_t retVal = 0;
    uint16_t sample = 1;
    char str[10] = "";
    int fields = 0;

    // Reads number of sample points from user
    do
    {
        printf( "\n\nEnter the desired number of sample points [1-255]: " );
        fields = scanf( "%s", str );
        sample = atoi( str );
    }while( sample < 1 || sample > 255 || ( 1 != fields ) );

    // Calculates size of each step
    uint16_t sampleCode = 65535;

    if( 1 != sample )
    {
        sampleCode = 65535 / ( sample - 1 );
    }

    int32_t adcCode = 0;
    uint8_t adcChannel = 0;

    //************************************
    // Connect SPI to QuikEval connector
    //************************************
    quikeval_SPI_connect();

    delay( LTC2422_CONV_DURATION_MS );
    // Take one ADC reading. Throw away the data, but check the channel.
    LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode );

    // If the data was from channel 1, take another reading so that the next reading will be channel 0.
    if( ADC_CH_1 == adcChannel )
    {
        delay( LTC2422_CONV_DURATION_MS );

        LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode );
        // IF we just read channel 1 (DAC A),
        // a conversion on channel B has just started. We want the FIRST reading in the table to be DAC A, so flush
        // this conversion, starting another conversion on DAC A. The Autozero phase takes 4/60 of a second (66.7ms) so there
        // is some time to print the header and set the DAC outputs.
    }

    printf( " Code,   DAC_A,   DAC_B\n" );

    // one voltage step at a time
    for( uint16_t i = 0; i < sample; i++ )
    {
        //************************************
        // Connect I2C to QuikEval connector
        //************************************
        quikeval_I2C_connect();

        uint16_t dacCode = sampleCode * i;
        // Write DAC code to both channels.
        LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS, LTC2607_WRITE_UPDATE_COMMAND, LTC2607_ALL_DACS, dacCode );


        //************************************
        // Connect SPI to QuikEval connector
        //************************************
        quikeval_SPI_connect();

        delay( LTC2422_CONV_DURATION_MS );
        // Read ADC channel 0 (DAC A) voltage and print it
        LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode );

        double adcVoltage = LTC2422_code_to_voltage( adcCode, ltc2422Lsb );
        printf( "%5u,%6lf,", dacCode, adcVoltage );

        delay( LTC2422_CONV_DURATION_MS );
        // Read ADC channel 1 (DAC B) voltage and print it
        LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode );

        adcVoltage = LTC2422_code_to_voltage( adcCode, ltc2422Lsb );
        printf( "%6lf\n", adcVoltage );
        fflush( stdout );

        // If they get out of sync, print "Out of sync!". This only happens if something bad occurs.
        if( ADC_CH_1 == adcChannel )
        {
            printf( "Out of sync!!\n" );

            //************************************
            // Connect I2C to QuikEval connector
            //************************************
            quikeval_I2C_connect();

            //Set output to zero
            LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS, LTC2607_WRITE_UPDATE_COMMAND, LTC2607_ALL_DACS, 0x0000 );
            return 1;
        }
    }

    //************************************
    // Connect I2C to QuikEval connector
    //************************************
    quikeval_I2C_connect();

    // Set output to zero
    LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS, LTC2607_WRITE_UPDATE_COMMAND, LTC2607_ALL_DACS, 0x0000 );

    printf( "\nCopy and save data points to a .csv file" );
    printf( "\nand open in Excel to plot points." );

    return retVal;
}


//!************************************************************************
//! Calibrate All
//!
//! @returns nothing
//!************************************************************************
void menu8CalibrateAll()
{
    //************************************
    // Connect I2C to QuikEval connector
    //************************************
    quikeval_I2C_connect();

    const uint16_t CAL_LOW_DAC_CODE  = 0x00FF;  // LTC2607 calibration code used for low-side force/measure
    LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS, LTC2607_WRITE_UPDATE_COMMAND, LTC2607_DAC_A, CAL_LOW_DAC_CODE );
    LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS, LTC2607_WRITE_UPDATE_COMMAND, LTC2607_DAC_B, CAL_LOW_DAC_CODE );

    printf( "\n\nCalibrating DACs ... " );
    fflush( stdout );
    delay( 2000 );

    //************************************
    // Connect SPI to QuikEval connector
    //************************************
    quikeval_SPI_connect();
    delay( 50 );

    int32_t adcCode = 0;
    uint8_t adcChannel = 0;

    delay( LTC2422_CONV_DURATION_MS );
    LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode ); // Throw away last reading

    delay( LTC2422_CONV_DURATION_MS );
    LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode );

    double voltage1[2] = { 0 }; // Calibration voltage 1

    if( ADC_CH_0 == adcChannel )
    {
        voltage1[DAC_CH_B] = LTC2422_code_to_voltage( adcCode, ltc2422Lsb );
    }
    else if( ADC_CH_1 == adcChannel )
    {
        voltage1[DAC_CH_A] = LTC2422_code_to_voltage( adcCode, ltc2422Lsb );
    }

    delay( LTC2422_CONV_DURATION_MS );
    LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode );

    if( ADC_CH_0 == adcChannel )
    {
        voltage1[DAC_CH_B] = LTC2422_code_to_voltage( adcCode, ltc2422Lsb );
    }
    else if( ADC_CH_1 == adcChannel )
    {
        voltage1[DAC_CH_A] = LTC2422_code_to_voltage( adcCode, ltc2422Lsb );
    }

    //************************************
    // Connect I2C to QuikEval connector
    //************************************
    quikeval_I2C_connect();

    const uint16_t CAL_HIGH_DAC_CODE = 0xFF00;  // LTC2607 calibration code used for high-side force/measure
    LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS, LTC2607_WRITE_UPDATE_COMMAND, LTC2607_DAC_A, CAL_HIGH_DAC_CODE );
    LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS, LTC2607_WRITE_UPDATE_COMMAND, LTC2607_DAC_B, CAL_HIGH_DAC_CODE );

    delay( 2000 );

    //************************************
    // Connect SPI to QuikEval connector
    //************************************
    quikeval_SPI_connect();
    delay( 50 );

    delay( LTC2422_CONV_DURATION_MS );
    LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode ); // Throw away last reading

    delay( LTC2422_CONV_DURATION_MS );
    LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode );

    double voltage2[2] = { 0 }; // Calibration voltage 2

    if( ADC_CH_0 == adcChannel )
    {
        voltage2[DAC_CH_B] = LTC2422_code_to_voltage( adcCode, ltc2422Lsb );
    }
    else if( ADC_CH_1 == adcChannel )
    {
        voltage2[DAC_CH_A] = LTC2422_code_to_voltage( adcCode, ltc2422Lsb );
    }

    delay( LTC2422_CONV_DURATION_MS );
    LTC2422_adc_read( LTC2422_CS, &adcChannel, &adcCode );

    if( ADC_CH_0 == adcChannel )
    {
        voltage2[DAC_CH_B] = LTC2422_code_to_voltage( adcCode, ltc2422Lsb );
    }
    else if( ADC_CH_1 == adcChannel )
    {
        voltage2[DAC_CH_A] = LTC2422_code_to_voltage( adcCode, ltc2422Lsb );
    }

    LTC2607_calibrate( CAL_LOW_DAC_CODE, CAL_HIGH_DAC_CODE,
                       voltage1[DAC_CH_A], voltage2[DAC_CH_A],
                       reinterpret_cast<float*>( &ltc2607LsbArray[DAC_CH_A] ),
                       &ltc2607OffsetArray[DAC_CH_A] );

    LTC2607_calibrate( CAL_LOW_DAC_CODE, CAL_HIGH_DAC_CODE,
                       voltage1[DAC_CH_B], voltage2[DAC_CH_B],
                       reinterpret_cast<float*>( &ltc2607LsbArray[DAC_CH_B] ),
                       &ltc2607OffsetArray[DAC_CH_B] );

    // Store All DACs lsb and offset from DAC A
    ltc2607LsbArray[DAC_CH_BOTH] = ltc2607LsbArray[DAC_CH_A];
    ltc2607OffsetArray[DAC_CH_BOTH] = ltc2607OffsetArray[DAC_CH_A];

    printf( "Calibration Complete\n" );
    printf( "lsb DAC A: %.3lf uV   offset: %d\n", 1.e6 * ltc2607LsbArray[DAC_CH_A],    ltc2607OffsetArray[DAC_CH_A] );
    printf( "lsb DAC B: %.3lf uV   offset: %d\n", 1.e6 * ltc2607LsbArray[DAC_CH_B],    ltc2607OffsetArray[DAC_CH_B] );
    printf( "All DACs : %.3lf uV   offset: %d\n", 1.e6 * ltc2607LsbArray[DAC_CH_BOTH], ltc2607OffsetArray[DAC_CH_BOTH] );
    fflush( stdout );

    //************************************
    // Connect I2C to QuikEval connector
    //************************************
    quikeval_I2C_connect();

    // Set output to zero
    LTC2607_write( LTC2607_I2C_GLOBAL_ADDRESS, LTC2607_WRITE_UPDATE_COMMAND, LTC2607_ALL_DACS, 0x0000 );
}
