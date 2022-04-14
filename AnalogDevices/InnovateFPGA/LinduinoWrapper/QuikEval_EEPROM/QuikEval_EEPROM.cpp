/*
QuikEval_EEPROM.cpp

This file contains the sources for accessing the LTC QuikEval EEPROM.


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

#include "QuikEval_EEPROM.h"

#include <string.h>
#include <string>
#include <sstream>
#include <vector>


//! Field indexes from the ID string
//!
//! ID string example: LTC2654-L16,Cls,D2636,01,01,DC,DC1678A-A,-------
//!                    ^^^^^^^^^^^     ^^^^^          ^^^^^^^^^
//!                         |            |                |
//!                         |            |                L---- 6   DEMO_BOARD_NAME
//!                         |            L--------------------- 2   GUI_CLASS_NUMBER
//!                         L---------------------------------- 0   MAIN_PART_NUMBER
enum EepromFields : uint8_t
{
    EEPROM_FIELDS_MAIN_PART_NUMBER = 0, //! index of the main part number in the parsed ID string
    EEPROM_FIELDS_GUI_CLASS_NUMBER = 2, //! index of the GUI class number in the parsed ID string
    EEPROM_FIELDS_DEMO_BOARD_NAME  = 6, //! index of the demo board name in the parsed ID string

    EEPROM_FIELDS_COUNT            = 7  //! expected number of fields in the parsed ID string
};


//! length of the ID string
static const uint8_t EEPROM_ID_SIZE = 50;


//!************************************************************************
//! Read the ID string from the EEPROM and determine if the correct board
//! is connected.
//!
//! @returns: true if the detected board number matches the expected one
//!************************************************************************
bool quikEvalEepromDiscoverDemoBoard
    (
    WireWrapper     wire,                   //!< I2C bus where the EEPROM is
    char*           expectedBoardNumber,    //!< given DC number to look for
    DemoBoardType*  detectedBoard           //!< structure with the detected board
    )
{
    bool isConnected = false;

    if( expectedBoardNumber && detectedBoard )
    {
        memset( detectedBoard, 0, sizeof( DemoBoardType ) );

        wire.beginTransmission( QUIKEVAL_EEPROM_I2C_ADDRESS );
        wire.write( static_cast<uint8_t>( 0x00 ) );
        wire.endTransmission();

        const uint8_t BUFFER_SIZE = EEPROM_ID_SIZE + 2;
        wire.requestFrom( static_cast<int>( QUIKEVAL_EEPROM_I2C_ADDRESS ), BUFFER_SIZE );

        char buf[BUFFER_SIZE] = { 0 };
        uint8_t i = 0;

        while( wire.available() )
        {
            buf[i++] = wire.read();

            if( BUFFER_SIZE == i )
            {
                break;
            }
        }

        std::stringstream rawString( buf );
        std::vector<std::string> vec;
        std::string crtSubstring;

        while( getline( rawString, crtSubstring, ',' ) )
        {
            vec.push_back( crtSubstring );
        }

        if( vec.size() >= EEPROM_FIELDS_COUNT )
        {
            strcpy( detectedBoard->productName, vec.at( EEPROM_FIELDS_MAIN_PART_NUMBER ).c_str() ); // e.g. LTC2607

            rawString.clear();
            rawString.str( vec.at( EEPROM_FIELDS_DEMO_BOARD_NAME ) ); // e.g. DC934 or DC934-A
            vec.clear();

            while( getline( rawString, crtSubstring, '-' ) )
            {
                vec.push_back( crtSubstring );
            }

            if( ( 2 == vec.size() ) && ( 1 == vec.at( 1 ).size() ) )
            {
                detectedBoard->demoCircuitOption = vec.at( 1 ).c_str()[0];
            }

            strcpy( detectedBoard->demoCircuitNumber, vec.at( 0 ).c_str() );

            if( 0 == strcmp( expectedBoardNumber, detectedBoard->demoCircuitNumber ) )
            {
                isConnected = true;
            }
        }
    }

    return isConnected;
}
