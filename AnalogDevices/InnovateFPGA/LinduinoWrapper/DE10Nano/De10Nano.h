/*
De10Nano.h

This file contains the definitions for the DE10-Nano board.


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

#ifndef De10Nano_h
#define De10Nano_h

#include <string>
#include <cstdint>

#include <hwlib.h>
#include <socal/socal.h>
#include <socal/hps.h>
#include <socal/alt_gpio.h>

//
// ARM Cortex-A9, architecture: ARMv7-A
//
// possible choices for flags:  __arm__   __ARM_ARCH_7__   __ARM_ARCH_7A__
//
#if( defined __arm__ )
    #define DE10NANO_SUPPORT
#endif


class De10Nano
{
    //************************************************************************
    // data types and constants
    //************************************************************************
    public:
        typedef enum
        {
            I2C_0,                          //! 1st I2C bus
            I2C_1,                          //! 2nd I2C bus
            I2C_2                           //! 3rd I2C bus
        }I2c;

        typedef enum
        {
            I2C_ONBOARD_GSENSOR = I2C_0,    //! alias for the on-board I2C bus
            I2C_LTC_QUIKEVAL    = I2C_1,    //! alias for the LTC connector I2C bus
            I2C_ARDUINO_SHIELD  = I2C_2     //! alias for the Arduino shield I2C bus
        }I2cAlias;


        typedef enum
        {
            SPI_0                           //! 1st SPI bus
        }Spi;


        typedef enum
        {
            DIO_DIRECTION_UNKNOWN,          //! direction of the digital IO is unknown
            DIO_DIRECTION_INPUT,            //! direction of the digital IO is input
            DIO_DIRECTION_OUTPUT            //! direction of the digital IO is output
        }DioDirection;

        typedef enum
        {
            DIO_STATUS_UNKNOWN,             //! status of the digital IO is unknown
            DIO_STATUS_LOW,                 //! status of the digital IO is low
            DIO_STATUS_HIGH                 //! status of the digital IO is high
        }DioStatus;


        static const uint32_t HW_REGS_BASE = ALT_STM_OFST;      //! offset for the Altera System Trace Macrocell
        static const uint32_t HW_REGS_SPAN = 0x04000000;        //! registers span
        static const uint32_t HW_REGS_MASK = HW_REGS_SPAN - 1;  //! registers mask

    private:
        static const uint32_t LTC_GPIO_MASK = 1 << 11; //! bit mask for HPS GPIO pin on the LTC connector

    //************************************************************************
    // functions
    //************************************************************************
    public:
        De10Nano();

        static De10Nano* getInstance();


        std::string getI2cString
            (
            I2c     aI2c            //!< the I2C bus to get its name
            );

        std::string getI2cString
            (
            I2cAlias aI2cAlias      //!< the I2C bus to get its name
            );

        std::string getSpiString
            (
            Spi     aSpi            //!< the SPI bus to get its name
            );


        void propagateLtcSpiSs
            (
            const bool aStatus      //!< true if LTC SPI /SS status is active (low)
            );

        bool getLtcSpiSs();

        bool getLtcSpiLastSs();


        void setLtcGpioDirection
            (
            const DioDirection  aDirection  //!< pin direction (I/O)
            );

        void setLtcGpioStatus
            (
            const DioStatus     aStatus     //!< pin status (L/H)
            );


        DioStatus getLtcGpioStatus();

        void toggleLtcGpio();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        static De10Nano*    sInstance;          //!< singleton

        bool                mLtcSsActive;       //!< LTC SPI /SS current status
        bool                mLtcSsWasRead;      //!< true if last LTC /SS status was read
};

#endif // De10Nano_h
