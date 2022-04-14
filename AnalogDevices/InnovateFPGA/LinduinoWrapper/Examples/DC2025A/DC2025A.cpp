/*
DC2025A.cpp adapted by C.M. Foale 2022 from DC934A.cpp  and DC2025a.ino
Byte order was changed in SpiWrapper.cpp to conform with the LTC2668 datasheet
Adaption copyright (c) 2022 C.M.Foale
Copyright (c) 2021, Mihai Ursu
*/
/*!
Linear Technology DC2025A Demonstration Board.
LTC2668: 16 Channel SPI 16-/12-Bit Rail-to-Rail DACs with 10ppm/C Max Reference.

  Explanation of Commands:
   1- Select DAC: Select one of sixteen DACs to test : 0 to 15

   2- Write to DAC input register: Value is stored in the DAC for updating
      later, allowing multiple channels to be updated at once, either
      through a software "Update All" command or by asserting the LDAC# pin.
      User will be prompted to enter either a code in hex or decimal, or a
      voltage. If a voltage is entered, a code will be calculated based on
      the active scaling and reference parameters - ideal values if no
      calibration was ever stored.

   3- Write and Update: Similar to item 1, but DAC is updated immediately.

   4- Update DAC: Copies the value from the input register into the DAC
      Register. Note that a "write and update" command writes the code to
      BOTH the input register and DAC register, so subsequent "update"
      commands will simply re-copy the same data (no change in output.)

   5- Power Down DAC: Disable DAC output. Power supply current is reduced.
      DAC code present in DAC registers at time of shutdown are preserved.

   6- Set reference mode, either internal or external: Selecting external
      mode powers down the internal reference voltage. It is the users
      responsibility to apply a 2.5v reference.

   7- Set SoftSpan: There are four options. In external mode, it is the users
      responsibility to compensate for the desired voltage.

   8- Toggle Selected Word: Switch between register A or B for DAC code.

   9- Set MUX: Enables / disables the MUX and sets the channel.

   10- Global Toggle Bit Settings - Enabling this feature sets the DAC to
       toggle from register A(when TGL pin is LOW) and register b(when TGL pin
       is HIGH). TGL pin is set HIGH with an internal pull up when the global toggle bit
       is set, and TGL pin is set LOW with an internal pull down when the global toggle bit
       is not set.

   11- Enable, disable, or store to EEPROM: To store to EEROM ensure all
       registers are set to their desired settings. Next, go to th store
       setting EEPROM menu and select it. Upon Linduinos power up, the
       previously stored settings will be restored.

   12- Voltage Ramp: Sets a voltage ramp to all the channels. CH0 = 0V, CH1 = 0.1V,
       CH2 = 0.2V, CH3 = 0.4V, ect.

USER INPUT DATA FORMAT:
 decimal : 1024
 hex     : 0x400
 octal   : 02000  (leading 0 "zero\n")
 binary  : B10000000000
 float   : 1024.0

@endverbatim

http://www.linear.com/product/LTC2668

http://www.linear.com/product/LTC2668#demoboards


Copyright 2018(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
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
#include <SPI.h>
#include <SpiWrapper.h>
#include <Wire.h>
#include <WireWrapper.h>
#include <De10Nano.h>


#include <LT_I2C.h>
#include <QuikEval_EEPROM.h> 
#include <LT_SPI.h>
#include <LTC2668.h>


//Macros
#define REF_EXTERNAL    LTC2668_REF_DISABLE     //!< External mode 
#define REF_INTERNAL    0                       //!< Internal mode
#define EEPROM_CAL_STATUS_ADDRESS 0x40    //! QuikEval EEPROM calibration start address
#define EEPROM_CAL_KEY            0x1234  //! Value to indicate calibration has occurred
#define EEPROM_I2C_ADDRESS        0xA0    //! EEPROM I2C address
#define EEPROM_PAGE_SIZE          16      //! EEPROM Page size in bytes
#define EEPROM_DATA_SIZE          256     //! EEPROM size in bytes
#define EEPROM_TIMEOUT            10      //! EEPROM timeout in ms

// Function Declaration

void print_title();                                     // Print the title block
void print_prompt(int8_t selected_dac);                 // Prompt the user for an input command
int16_t prompt_voltage_or_code(uint8_t selected_dac);   // Prompt the user for voltage or code
uint16_t get_voltage(uint8_t selected_dac);             // Obtains voltage from user
uint16_t get_code();                                    // Obtains code from user

void menu_1_select_dac(uint8_t *selected_dac);              // Menu  1: prompts user for DAC selection
void menu_2_write_to_input_register(uint8_t selected_dac);  // Menu  2: sends data to selected DAC register with no update
void menu_3_write_and_update_dac(uint8_t selected_dac);     // Menu  3: sends data to selected DAC register with update
void menu_4_update_power_up_dac(uint8_t selected_dac);      // Menu  4: updates and powers up selected  DAC
void menu_5_power_down_dac(uint8_t selected_dac);           // Menu  5: powers down selected DAC
void menu_6_set_reference_mode();                           // Menu  6: prompts user to enable/disable internal reference
void menu_7_set_softspan_range(uint8_t selected_dac);       // Menu  7: prompts user to sets the SoftSpan range
void menu_8_toggle_select_word();                           // Menu  8: prompts user to enter the toggle select word
void menu_9_set_mux();                                      // Menu  9: sets MEX
void menu_10_global_toggle_settings();                      // Menu 10: sets the global toggle bit
//void menu_11_enable_disable_eeprom_restore();               // Menu 11: EEPOM restore settings - not available in LinduinoWrapper/QuickEval_EEPROM.h
void menu_12_voltage_ramp();                                // Menu 12: sets a voltage ramp for the LTC2668
void menu_13_settling_test();                               // Menu 13: Settling time test
void menu_15_loopback_test();
void menu_16_exit();

char getche();//gets a single character input
int read_int(); //gets an integer
float read_float();//gets float
// Global variable
static uint8_t demo_board_connected = 0;   //!< Set to 1 if the board is connected
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

int main()
{
    De10Nano* de10 = De10Nano::getInstance();

    if( de10 )
    {
        //**********************
        // GPIO
        //**********************
        de10->setLtcGpioDirection( De10Nano::DIO_DIRECTION_OUTPUT );
        de10->setLtcGpioStatus( De10Nano::DIO_STATUS_LOW );

        //**********************
        // I2C
        //**********************
        quikeval_I2C_init();          // Configure the EEPROM I2C port for 100kHz
        WireWrapper wire = quikeval_I2C_get_wire();

        //**********************
        // SPI
        //**********************
        //SPISettings spiSettings = SPISettings( 500000, MSBFIRST, SPI_MODE1 );//cmf - for dc934A SpiWrapper has SPI_MODE1 by default
        //quikeval_SPI_apply_settings( spiSettings ); //for dc934A
        
        bool spiInit = quikeval_SPI_init();          // Configure the spi port for 4MHz SCK set by default in De10Nano.cpp
		quikeval_SPI_connect();       // Connect SPI to main data port
		
		de10->setLtcGpioDirection( De10Nano::DIO_DIRECTION_OUTPUT ); //cmf -  essential for SPI
        de10->setLtcGpioStatus( De10Nano::DIO_STATUS_LOW ); //cmf -essential for SPI
		
        char expectedBoardName[] = "DC2025A";  // Demo Board Name stored in QuikEval EEPROM

        bool dc2025Connected = quikEvalEepromDiscoverDemoBoard( wire, expectedBoardName, &detectedBoard );
        printf("DemoBoardType productName %s demoCircuitNumber %s demoCircuitOption %c connected %d\n",
			detectedBoard.productName,detectedBoard.demoCircuitNumber,detectedBoard.demoCircuitOption,dc2025Connected);
		
        if( !dc2025Connected )
        {
            printf( "\nCould not find a %s board connected", expectedBoardName );
        }

        if( spiInit && dc2025Connected )
        {
            demo_board_connected = 1;
            print_title();
            printf( "\nConnected to %s\n", detectedBoard.demoCircuitNumber );
            if( detectedBoard.demoCircuitOption )
            {
                printf( "-%c\n", detectedBoard.demoCircuitOption );
            }
            printf( ", Product Name is %s\n", detectedBoard.productName );
            
			static  uint8_t selected_dac = 3;     // The selected DAC to be updated (0=CH0, 1=CH1 ... 16=All).  Initialized to CH0.
			//Testing.....
			/*
			uint16_t dac_code= 0x2648;
			LTC2668_write(LTC2668_CS,LTC2668_CMD_WRITE_N_UPDATE_N, selected_dac, dac_code); // Send dac_code
			return 0; */
			
            printf( "\n\nEnter the DAC number integer:\n");
            for(;;)
            {
                print_prompt( selected_dac );               
                //char cmd = getche();
				int cmd = read_int();
                switch( cmd )
                {
                    case 1:
                        // Select a DAC to update
                        menu_1_select_dac(&selected_dac);
                        break;

                    case 2:
                        // Write to input register only
                        menu_2_write_to_input_register( selected_dac );
                        break;

                    case 3:
                        menu_3_write_and_update_dac( selected_dac );
                        break;

                    case 4:
                        // Update/Power up DAC
                        menu_4_update_power_up_dac( selected_dac );
                        break;

                    case 5:
                        // Power down DAC
                        menu_5_power_down_dac( selected_dac );
                        break;

                    case 6:
						menu_6_set_reference_mode();
						break;
					case 7:
						menu_7_set_softspan_range(selected_dac);
						break;
					case 8:
						menu_8_toggle_select_word();
						break;
					case 9:
						menu_9_set_mux();
						break;
					case 10:
						menu_10_global_toggle_settings();
						break;
					case 11:
						printf("not implemented\n\n");
						//menu_11_enable_disable_eeprom_restore();
						break;
					case 12:
						menu_12_voltage_ramp();
						break;
					case 13:
						menu_13_settling_test();
						break;
					case 14:
						printf("not implemented\n\n");
						break;
					case 15:
						menu_15_loopback_test();
						break;
					case 16:
						menu_16_exit();
						break;
					default:
						printf("Incorrect Option\n");
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

//! Select which DAC to operate on
void menu_1_select_dac(uint8_t *selected_dac)
{
  // Select a DAC to operate on
  printf("Select DAC to operate on (0 to 15, 16 for ALL DACs)\n");
  *selected_dac = read_int();
  if (*selected_dac == 16)
    printf("All/n\n");
  else
    printf("%d/n",*selected_dac);
}

//! Write data to input register, but do not update DAC output
void menu_2_write_to_input_register(uint8_t selected_dac)
{
  uint16_t dac_code;

  if (prompt_voltage_or_code(selected_dac) == PROMPT_VOLTAGE)
    dac_code = get_voltage(selected_dac);
  else
    dac_code = get_code();

  if (selected_dac <= 15)
  {
    // Store dac_code to register variable a or b
    if (eeprom.data_struct.toggle_word & (0x01 << selected_dac))
    {
      eeprom.data_struct.dac_code_b[selected_dac] = dac_code;
    }
    else
      eeprom.data_struct.dac_code_a[selected_dac] = dac_code;
    LTC2668_write(LTC2668_CS, LTC2668_CMD_WRITE_N, selected_dac, dac_code);
  }
  else
  {
    // Store dac_code to register variable a or b
    for (uint8_t i = 0; i <= 15 ; i++)
    {
      if (eeprom.data_struct.toggle_word & (0x01 << i))
      {
        eeprom.data_struct.dac_code_b[i] = dac_code;
      }
      else
        eeprom.data_struct.dac_code_a[i] = dac_code;
    }
    LTC2668_write(LTC2668_CS, LTC2668_CMD_WRITE_ALL, selected_dac, dac_code);
  }
}

//! Write data to DAC register (which updates output immediately)
void menu_3_write_and_update_dac(uint8_t selected_dac)
{
  uint16_t dac_code;

  if (prompt_voltage_or_code(selected_dac) == PROMPT_VOLTAGE)
    dac_code = get_voltage(selected_dac);
  else
    dac_code = get_code();

  if (selected_dac <= 15)
  {
    // Store dac_code to register variable a or b
    if (eeprom.data_struct.toggle_word & (0x01 << selected_dac))
    {
      eeprom.data_struct.dac_code_b[selected_dac] = dac_code;
    }
    else
      eeprom.data_struct.dac_code_a[selected_dac] = dac_code;

    LTC2668_write(LTC2668_CS,LTC2668_CMD_WRITE_N_UPDATE_N, selected_dac, dac_code); // Send dac_code
  }
  else  // All DACs
  {
    // Store dac_code to register variable a or b
    for (uint8_t i = 0; i <= 15 ; i++)
    {
      if (eeprom.data_struct.toggle_word & (0x01 << i))
      {
        eeprom.data_struct.dac_code_b[i] = dac_code;
      }
      else
        eeprom.data_struct.dac_code_a[i] = dac_code;
    }
    LTC2668_write(LTC2668_CS,LTC2668_CMD_WRITE_ALL_UPDATE_ALL, 0, dac_code);    // Send dac_code
  }
}

//! Update DAC with data that is stored in input register, power up if sleeping
void menu_4_update_power_up_dac(uint8_t selected_dac)
{
  // Update DAC
  if (selected_dac <= 15)
    LTC2668_write(LTC2668_CS,LTC2668_CMD_UPDATE_N, selected_dac, 0);
  else
    LTC2668_write(LTC2668_CS,LTC2668_CMD_UPDATE_ALL, 0, 0);
}

//! Power down DAC
void menu_5_power_down_dac(uint8_t selected_dac)
{
  // Power down DAC
  if (selected_dac <= 15)
    LTC2668_write(LTC2668_CS, LTC2668_CMD_POWER_DOWN_N, selected_dac, 0);
  else
    LTC2668_write(LTC2668_CS, LTC2668_CMD_POWER_DOWN_ALL, 0, 0);
}

//! Set reference mode
void menu_6_set_reference_mode(void)
{
  int16_t user_command;
  printf("0 - Power down the internal reference\n");
  printf("1 - Power up the internal reference\n");
  user_command = read_int();
  if (user_command > 1)
    user_command = 1;
  if (user_command < 0)
    user_command = 0;

  if (user_command == 1)
  {
    eeprom.data_struct.reference_mode = REF_INTERNAL;
    LTC2668_write(LTC2668_CS, LTC2668_CMD_CONFIG, 0, eeprom.data_struct.reference_mode);
    printf("Internal reference has been powered up\n");
  }
  else
  {
    eeprom.data_struct.reference_mode = REF_EXTERNAL;
    LTC2668_write(LTC2668_CS, LTC2668_CMD_CONFIG, 0, eeprom.data_struct.reference_mode);
    printf("Internal reference has been powered down\n");
  }
}

//! Set SoftSpan Range
void menu_7_set_softspan_range(uint8_t selected_dac)
{
  int16_t user_command;

  printf("0- 0V to 5V\n");
  printf("1- 0V to 10V\n");
  printf("2- -5V to 5V\n");
  printf("3- -10V to 10V\n");
  printf("4- -2.5V to 2.5V\n");
  printf("Select a SoftSpan Range: \n");
  user_command = read_int();

  if (user_command > 4)
    user_command = 4;
  if (user_command < 0)
    user_command = 0;
  printf("%d/n",user_command);

  if (selected_dac <= 15)
  {
    eeprom.data_struct.soft_span_range[selected_dac] = user_command;
    LTC2668_write(LTC2668_CS, LTC2668_CMD_SPAN, selected_dac, eeprom.data_struct.soft_span_range[selected_dac]);
  }
  else
  {
    for (uint8_t i = 0; 0 <= 15; i++)
      eeprom.data_struct.soft_span_range[i] = user_command;

    LTC2668_write(LTC2668_CS, LTC2668_CMD_SPAN_ALL, 0, eeprom.data_struct.soft_span_range[0]);
  }
}

//! Enter toggle select word, which also sets the register that will be written
//! if bit is 0, register A is written, if 1, register B is written.
void menu_8_toggle_select_word()
{
  // Select a DAC to operate on
  printf("Toggle Select bit sets the register to be written for the corresponding DAC.\n\n");
  printf("0 for Register A or 1 for Register B.\n\n");
  printf("Note: DAC Update from Register B requires TGB = 1\n\n");
  printf("      DAC Update from Register A requires TCB = 0\n\n");
  printf("Enter Toggle Select Byte as hex (0xMM) or binary (B10101010): \n\n");
  eeprom.data_struct.toggle_word = read_int();

  LTC2668_write(LTC2668_CS, LTC2668_CMD_TOGGLE_SEL, 0, eeprom.data_struct.toggle_word);
  printf("eeprom.data_struct.toggle_word %d\n\n",eeprom.data_struct.toggle_word);
}

//! Enable / Disable and sets the channel for the MUX
void menu_9_set_mux()
{
  int16_t user_command;
  printf("0- Disable Mux\n\n");
  printf("1- Enable Mux\n\n");
  printf("Enter a command: \n\n");
  user_command = read_int();

  if (user_command == 1)
  {
    printf("Select MUX channel(0-CH0, 1-CH1,...15-CH15 : \n\n");
    user_command = read_int();

    if (user_command > 15)
      user_command = 15;
    if (user_command < 0)
		user_command = 0;

    printf("%d\n",user_command);
    eeprom.data_struct.mux_state = LTC2668_MUX_ENABLE | user_command;
    LTC2668_write(LTC2668_CS,LTC2668_CMD_MUX, 0, eeprom.data_struct.mux_state);
  }
  else
  {
    eeprom.data_struct.mux_state = LTC2668_MUX_ENABLE | user_command;
    LTC2668_write(LTC2668_CS,LTC2668_CMD_MUX, 0, eeprom.data_struct.mux_state);
    printf("The MUX has been disabled\n");
  }
}
//! Enable / Disable the global toggle bit
void menu_10_global_toggle_settings()
{
  int16_t user_command;
  printf("0- Disable Global Toggle\n");
  printf("1- Enable Global Toggle\n");
  printf("Enter a command: \n");
  user_command = read_int();

  if (user_command > 1)
    user_command = 1;
  if (user_command < 0)
    user_command = 0;
  printf("%d\n",user_command);

  eeprom.data_struct.global_toggle_bit = user_command;
  LTC2668_write(LTC2668_CS, LTC2668_CMD_GLOBAL_TOGGLE, 0, eeprom.data_struct.global_toggle_bit);
}
/*NOT AVAILABLE IN LinduinoWrapper/QuickEval_EEPROM
//! Enable / Disable restoration of DAC values from EEPROM Use with caution - behaviour is undefined if you
//! enable restoration and data has NOT been previously stored from a known state.

void menu_11_enable_disable_eeprom_restore()
{
  printf("\n0- Enable Restoration\n");
  printf("1- Disable Restoration\n");
  printf("2- Store DAC Settings\n");
  int user_input = read_int();
  switch (user_input)
  {
    case 0:
      printf("Enabling EEPROM restoration\n");
      eeprom.data_struct.cal_key = EEPROM_CAL_KEY;
      eeprom_write_int16(EEPROM_I2C_ADDRESS, eeprom.data_struct.cal_key, EEPROM_CAL_STATUS_ADDRESS);
      break;
    case 1:
      printf("Disabling EEPROM restoration\n");
      eeprom.data_struct.cal_key = 0;
      eeprom_write_int16(EEPROM_I2C_ADDRESS, eeprom.data_struct.cal_key, EEPROM_CAL_STATUS_ADDRESS);
      break;
    case 2:
      eeprom.data_struct.cal_key = EEPROM_CAL_KEY;
      eeprom_write_byte_array(EEPROM_I2C_ADDRESS, eeprom.byte_array, EEPROM_CAL_STATUS_ADDRESS, sizeof(eeprom_data_union));
      printf("Stored Settings to EEPROM\n");
  }
}*/

//! Sets a voltage ramp to all the channels
void menu_12_voltage_ramp()
{
  uint8_t i;
  for (i=0; i <= 15; ++i)
  {
    LTC2668_write(LTC2668_CS,LTC2668_CMD_WRITE_N_UPDATE_N, i, i * 1312);
  }
  printf("A voltage ramp was set to the LTC2668\n");
}

///////////////////////////////////////////////////////////////////////////
// The following functions are used internally for testing, and included //
// here for reference.                                                   //
///////////////////////////////////////////////////////////////////////////

// Settling time routine, used for datasheet pictures. Set scope to ARM trigger on SETTLE_TRIGGER
// Rising edge, trigger on CS pin for the actual edge that causes the update.
// You can test any other channel / starting / final values by setting up the DAC's
// register A to the initial (starting) voltage and register B to the final (settled) voltage.
// The channel's toggle bit must be set as well.

#define INTEGRATOR_CONTROL  2               // Pin to control integrator on settling time circuit
// Set low for 100ms after full settling has occurred,
// high to run test.
#define SETTLE_TRIGGER      3               // High to low edge close to DAC active edge. Can also be
// used to drive DAC simulator.
#define SETTLE_CHANNEL 0                   // Which DAC to test for settling (only one at a time to prevent
// crosstalk artefacts.)
void menu_13_settling_test()
{
/*  int16_t user_input, initial, final;
  int8_t range;
  printf("Select test to run:\n");
  printf("0: presently programmed DAC values\n");
  printf("1: 0-5 rising\n");
  printf("2: 0-5 falling\n");
  printf("3: 0-10 rising\n");
  printf("4: 0-10 falling\n");
  printf("5: bipolar 10 rising\n");
  printf("6: bipolar 10 falling\n");
  printf("7: 0-5 1/4 to 3/4 scale, rising\n");
  printf("8: 0-5 3/4 to 1/4 scale, falling\n");
  printf("9: bipolar 5 rising\n");
  printf("10: bipolar 5 falling\n");
  printf("11: bipolar 2.5 rising\n");
  printf("12: bipolar 2.5 falling\n");

  user_input = read_int();
  switch (user_input)
  {
    case 1:
      initial = 0x0000;
      final = 0xFFFF;
      range = LTC2668_SPAN_0_TO_5V;
      break;
    case 2:
      initial = 0xFFFF;
      final = 0x0000;
      range = LTC2668_SPAN_0_TO_5V;
      break;
    case 3:
      initial = 0x0000;
      final = 0xFFFF;
      range = LTC2668_SPAN_0_TO_10V;
      break;
    case 4:
      initial = 0xFFFF;
      final = 0x0000;
      range = LTC2668_SPAN_0_TO_10V;
      break;
    case 5:
      initial = 0x0000;
      final = 0xFFFF;
      range = LTC2668_SPAN_PLUS_MINUS_10V;
      break;
    case 6:
      initial = 0xFFFF;
      final = 0x0000;
      range = LTC2668_SPAN_PLUS_MINUS_10V;
      break;
    case 7:
      initial = 0x4000;
      final = 0xC000;
      range = LTC2668_SPAN_0_TO_5V;
      break;
    case 8:
      initial = 0xC000;
      final = 0x4000;
      range = LTC2668_SPAN_0_TO_5V;
      break;
    case 9:
      initial = 0x0000;
      final = 0xFFFF;
      range = LTC2668_SPAN_PLUS_MINUS_5V;
      break;
    case 10:
      initial = 0xFFFF;
      final = 0x0000;
      range = LTC2668_SPAN_PLUS_MINUS_5V;
      break;
    case 11:
      initial = 0x0000;
      final = 0xFFFF;
      range = LTC2668_SPAN_PLUS_MINUS_2V5;
      break;
    case 12:
      initial = 0xFFFF;
      final = 0x0000;
      range = LTC2668_SPAN_PLUS_MINUS_2V5;
      break;
    default:
      printf("Using presently programmed DAC values\n");
  }
  if (user_input != 0) // Load DAC to be tested with range, initial, and final values.
  {
    LTC2668_write(LTC2668_CS, LTC2668_CMD_SPAN, SETTLE_CHANNEL, range); // Set Span
    LTC2668_write(LTC2668_CS, LTC2668_CMD_TOGGLE_SEL, 0, 0x0000); // Point to register A
    LTC2668_write(LTC2668_CS, LTC2668_CMD_WRITE_N_UPDATE_N, SETTLE_CHANNEL, initial); // Send dac_code
    LTC2668_write(LTC2668_CS, LTC2668_CMD_TOGGLE_SEL, 0, 0x0001 << SETTLE_CHANNEL); // Point to register B
    LTC2668_write(LTC2668_CS, LTC2668_CMD_WRITE_N_UPDATE_N, SETTLE_CHANNEL, final); // Send dac_code
  }
  printf("Settling time test running, send any character to terminate.\n");
  pinMode(INTEGRATOR_CONTROL, OUTPUT);
  pinMode(SETTLE_TRIGGER, OUTPUT);
  while (!Serial.available())
  {
    PORTD |= 1 << INTEGRATOR_CONTROL; // Disable integrator
    PORTD |= 1 << SETTLE_TRIGGER;
    LTC2668_write(LTC2668_CS, LTC2668_CMD_GLOBAL_TOGGLE, 0, 0); // Set global toggle bit LOW, INITIAL value
    PORTD &= ~(1 << SETTLE_TRIGGER); // Reset settle trigger LOW
    delay(10); // Wait to get to initial value
    LTC2668_write(LTC2668_CS, LTC2668_CMD_GLOBAL_TOGGLE, 0, 1); // Set global toggle HIGH, FINAL value
    delay(10); // Wait to get to final value, here is where you look at the settling
    PORTD &= ~(1 << INTEGRATOR_CONTROL); // Enable integrator
    delay(100);
  }
  Serial.read();
  printf("Exiting Settling time test.\n");
  */
}

// Loopback test to verify data integrity.
void menu_15_loopback_test()
{
  uint16_t i;
  int8_t ack = 0;
  printf("Testing readback by writing 0-10,000, NOP commands\n\n");
  LTC2668_write(LTC2668_CS, LTC2668_CMD_NO_OPERATION, 0, 0); // If this is the first time the read function is called, need to initialize
  for (i=0; i<=10000; ++i)
  {
    ack |= LTC2668_write(LTC2668_CS, LTC2668_CMD_NO_OPERATION, 0, i);
  }
  if (ack)
  {
    printf("Oops, got an error somewhere!\n");
  }
  else
  {
    printf("No errors!!\n");
  }
}


//! Prompt user to enter a voltage or digital code to send to DAC
//! @returns user input
int16_t prompt_voltage_or_code(uint8_t selected_dac )
{
  int16_t user_input;

  printf("Type 1 to enter voltage, 2 to enter code: \n");
  user_input = read_int();
  printf("%d\n", user_input);

  if (user_input != 2)
  {
    if (selected_dac >= 16)
    {
      printf("\nCaution! Voltage SoftSpan could be different for different DACs\n");
      printf("Ensure All DACs can be set to desired voltages\n");
      printf("DAC 0 is used for SoftSpan values for all DACs.\n");
      printf("Toggle Register Select Bits 0=A, 1 = B: 0b\n");
      printf("eeprom.data_struct.toggle_word %d\n",eeprom.data_struct.toggle_word);
    }
    return(PROMPT_VOLTAGE);
  }
  else
  {
    if (selected_dac >= 16)
    {
      printf("DAC 0 is used for SoftSpan values for all DACs.\n");
    }
  }
  return(PROMPT_CODE);
}

//! Get voltage from user input, calculate DAC code based on lsb, offset
//! @returns the DAC code
uint16_t get_voltage(uint8_t selected_dac)
{
  float dac_voltage;

  printf("Enter Desired DAC output voltage: \n");
  dac_voltage = read_float();
  printf("%f",dac_voltage);
  printf(" V\n");

  if (selected_dac <= 15)
    return(LTC2668_voltage_to_code(dac_voltage,
                                   LTC2668_MIN_OUTPUT[eeprom.data_struct.soft_span_range[selected_dac]],
                                   LTC2668_MAX_OUTPUT[eeprom.data_struct.soft_span_range[selected_dac]]));
  else
    return(LTC2668_voltage_to_code(dac_voltage,
                                   LTC2668_MIN_OUTPUT[eeprom.data_struct.soft_span_range[0]],
                                   LTC2668_MAX_OUTPUT[eeprom.data_struct.soft_span_range[0]]));
}

//! Get code to send to DAC directly, in decimal, hex, or binary
//! @return code from user
uint16_t get_code()
{
  uint16_t returncode;
  printf("Enter Desired DAC Code\n");
  printf("(Format 32768, 0x8000, 0100000, or B1000000000000000): \n");
  returncode = (uint16_t) read_int();
  printf("0x%x\n",returncode);
  return(returncode);
}

//! Prints the title block when program first starts.
void print_title()
{
  printf("*****************************************************************\n");
  printf("* DC2025 Demonstration Program                                  *\n");
  printf("*                                                               *\n");
  printf("* This program demonstrates how to send data to the LTC2668     *\n");
  printf("* Sixteen Channel 16/12-bit DAC found on the DC2025 demo board. *\n");
  printf("*                                                               *\n");
  printf("*****************************************************************\n");
}

//! Prints main menu.
void print_prompt(int8_t selected_dac)
{
  // Displays menu
  printf("\nCommand Summary:\n");
  printf("  1-Select DAC\n");
  printf("  2-Write to input register (no update)\n");
  printf("  3-Write and update DAC\n");
  printf("  4-Update / power up DAC\n");
  printf("  5-Power down DAC\n");
  printf("  6-Set reference mode\n");
  printf("  7-Set SoftSpan\n");
  printf("  8-Toggle selected register\n");
  printf("  9-Set Mux\n");
  printf(" 10-Global toggle bit settings\n");
  printf(" 11-Enable, disable, or store to EEPROM\n");
  printf(" 12-Voltage Ramp\n");
  printf(" 13-Settling Test\n");
  printf(" 15-Loopback Test\n");
  printf(" 16-Exit\n");
  printf("\nPresent Values:\n");
  printf("  Selected DAC: \n");

  // Display current settings
  if (selected_dac >= 15 )
  {
    printf("All\n");
    printf("  SoftSpan range: %f", LTC2668_MIN_OUTPUT[eeprom.data_struct.soft_span_range[0]]);
    printf(" V to %f V\n",LTC2668_MAX_OUTPUT[eeprom.data_struct.soft_span_range[0]]);
  }
  else
  {
    printf("%d  SoftSpan range: %f  V to %f V\n",selected_dac, LTC2668_MIN_OUTPUT[eeprom.data_struct.soft_span_range[selected_dac]], 
		LTC2668_MAX_OUTPUT[eeprom.data_struct.soft_span_range[selected_dac]]);
  }
  printf("  Toggle Register: \n");
  for (uint8_t i = 0; i <= 15; i++)
  {
    if ((eeprom.data_struct.toggle_word &  (0x8000 >> i)))
      printf("B");
    else
      printf("A");
  }
  printf("\n  Global Toggle Bit: \n");
  printf("%d \n",eeprom.data_struct.global_toggle_bit);

  printf("  Reference: \n");
  if (eeprom.data_struct.reference_mode == REF_INTERNAL)
    printf("Internal\n");
  else
    printf("External\n");

  printf("\nEnter a command: \n");
}

int read_int() {
	char str[16]="";
	int ret = scanf("%[^\n]%*c",str);
	int intval = atoi(str);
	//printf("read_int got %d\n",intval);
	return intval;
}

// Read a float value from the serial interface
float read_float()
{
  float data;
  char str[16]="";
  int ret = scanf("%[^\n]%*c",str);
  data = atof(str);
  //printf("read_float got %f\n",data);
  return(data);
}

void menu_16_exit(){
	exit( 0 );
}
