/*
Arduino.h

This file contains the definitions for the Arduino wrapper.


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

#ifndef Arduino_h
#define Arduino_h

#include <stdint.h>
#include <string.h>
#include <math.h>

#include <SpiWrapper.h>
#include <WireWrapper.h>


//************************************************************************
// data types and constants
//************************************************************************
typedef uint8_t byte;


// see  https://www.arduino.cc/en/reference/SPI
#ifndef SS
#define SS 10   //! Slave Select pin for Arduino Uno (active low)
#endif

#ifndef MISO
#define MISO 12 //! Master Input Slave Output pin for Arduino Uno
#endif


//! Statuses enum for digital pins
typedef enum
{
    LOW,                //! status of digital pin is low
    HIGH                //! status of digital pin is high
}InoDigitalPinStatus;


//! Types enum for digital pins
typedef enum
{
    INPUT,              //! type of digital pin is input
    OUTPUT,             //! type of digital pin is output
    INPUT_PULLUP        //! type of digital pin is pull-up input
}InoDigitalPinType;


//************************************************************************
// functions
//************************************************************************
void pinMode
    (
    uint8_t pin,        //!< digital pin
    uint8_t mode        //!< mode for configuring the pin
    );

void digitalWrite
    (
    uint8_t pin,        //!< digital pin
    uint8_t val         //!< value to write to the pin
    );

int digitalRead
    (
    uint8_t pin         //!< digital pin to read from
    );


void delay
    (
    uint32_t ms         //!< duration in milliseconds
    );

void delayMicroseconds
    (
    uint32_t us         //!< duration in microseconds
    );


int constrain
    (
    int     x,          //!< argument to compare with limits
    int     a,          //!< lower limit
    int     b           //!< higher limit
    );

#endif // Arduino_h
