/*
QuikEval_EEPROM.h

This file contains the definitions for accessing the LTC QuikEval EEPROM.


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

#ifndef QuikEval_EEPROM_h
#define QuikEval_EEPROM_h

#include <stdint.h>
#include <WireWrapper.h>


//************************************************************************
// data types and constants
//************************************************************************
//! I2C slave address of the EEPROM
const uint8_t QUIKEVAL_EEPROM_I2C_ADDRESS = 0x50;


//! Structure to hold parsed information from the ID string.
//! ID string example: LTC2654-L16,Cls,D2636,01,01,DC,DC1678A-A,-------
typedef struct
{
  char productName[15];        //!< LTC Product  (LTC2654-L16)
  char demoCircuitNumber[15];  //!< Demo Circuit number (DC1678)
  char demoCircuitOption;      //!< Demo Circuit option (A)
}DemoBoardType;


//************************************************************************
// functions
//************************************************************************
bool quikEvalEepromDiscoverDemoBoard
    (
    WireWrapper     wire,                       //!< I2C bus where the EEPROM is
    char*           expectedDemoCircuitNumber,  //!< given DC number to look for
    DemoBoardType*  detectedBoard               //!< structure with the detected board
    );

#endif // QuikEval_EEPROM_h
