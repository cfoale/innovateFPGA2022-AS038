/*
Arduino.cpp

This file contains the sources for the Arduino wrapper.


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

#include <Arduino.h>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <De10Nano.h>


//!************************************************************************
//! Set the mode of a digital pin.
//! The only digital pin used individually on the LTC connector is the GPIO,
//! which has dedicated functions in the De10Nano class.
//!
//! @returns: nothing
//!************************************************************************
void pinMode
    (
    uint8_t pin,    //!< digital pin
    uint8_t mode    //!< mode for configuring the pin
    )
{
    (void)pin;
    (void)mode;
}


//!************************************************************************
//! Write a state to a digital pin.
//! The only tracked writings are to the /SS pin for providing compatibility
//! in Linux for native Arduino code.
//!
//! @returns: nothing
//************************************************************************
void digitalWrite
    (
    uint8_t pin,    //!< digital pin
    uint8_t val     //!< value to write to the pin
    )
{   
    #ifdef DE10NANO_SUPPORT
        if( SS == pin )
        {
            De10Nano* de10 = De10Nano::getInstance();

            if( de10 )
            {
                // The next call is for SPI transfers between explicit /SS pin transitions,
                // which are typical for Arduino/Linduino but not for Linux.
                //
                // Example:
                //
                // digitalWrite(SS, LOW);
                // ..
                // transfer(..);
                // ..
                // digitalWrite(SS, HIGH);
                //
                de10->propagateLtcSpiSs( LOW == val );
            }
        }
    #endif
}


//!***********************************************************************
//! Read the state of a digital pin
//! The only digital pin used individually on the LTC connector is the GPIO,
//! which has dedicated functions in the De10Nano class.
//!
//! @returns: The state of the pin
//!************************************************************************
int digitalRead
    (
    uint8_t pin     //!< digital pin to read from
    )
{
    (void)pin;
    return 0;
}


//!************************************************************************
//! Delay [ms]
//!
//! @returns: nothing
//************************************************************************
void delay
    (
    uint32_t ms     //!< duration in milliseconds
    )
{
    usleep( 1000 * ms );
}


//!************************************************************************
//! Delay [us]
//!
//! @returns: nothing
//!************************************************************************
void delayMicroseconds
    (
    uint32_t us     //!< duration in microseconds
    )
{
    usleep( us );
}


//!************************************************************************
//! Check if a parameter is within two limits
//! NOT checking if first limit is below second limit
//!
//! @returns: Constrain value
//!************************************************************************
int constrain
    (
    int     x,          //!< argument to compare with limits
    int     a,          //!< lower limit
    int     b           //!< higher limit
    )
{
    int retVal = x;

    if( x < a )
    {
        retVal = a;
    }
    else if( x > b )
    {
        retVal = b;
    }

    return retVal;
}
