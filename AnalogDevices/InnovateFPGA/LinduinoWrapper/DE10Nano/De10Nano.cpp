/*
De10Nano.cpp
This file contains the sources for DE10-Nano board.


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

#include "De10Nano.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>


De10Nano* De10Nano::sInstance = nullptr;

//!************************************************************************
//! Constructor
//!************************************************************************
De10Nano::De10Nano()
    : mLtcSsActive( false )
    , mLtcSsWasRead( false )
{
}


//!************************************************************************
//! Singleton
//!
//! @returns the instance of the object
//!************************************************************************
De10Nano* De10Nano::getInstance()
{
    if( !sInstance )
    {
        sInstance = new De10Nano;
    }

    return sInstance;
}


//!************************************************************************
//! Get the full name for an I2C bus
//!
//! @returns: A string with the full name of the I2C
//!************************************************************************
std::string De10Nano::getI2cString
    (
    I2c      aI2c    //!< the I2C bus to get its name
    )
{
    std::string i2cString;

    switch( aI2c )
    {
        case I2C_0:
            i2cString = "/dev/i2c-0";
            break;

        case I2C_1:
            i2cString = "/dev/i2c-1";
            break;

        case I2C_2:
            i2cString = "/dev/i2c-2";
            break;

        default:
            break;
    }

    return i2cString;
}


//!************************************************************************
//! Get the full name for an I2C bus
//!
//! @returns: A string with the full name of the I2C
//!************************************************************************
std::string De10Nano::getI2cString
    (
    I2cAlias   aI2cAlias    //!< the I2C bus alias to get its name
    )
{
    return getI2cString( static_cast<I2c>( aI2cAlias ) );
}


//!************************************************************************
//! Get the full name for a SPI bus
//!
//! @returns: A string with the full name of the SPI
//!************************************************************************
std::string De10Nano::getSpiString
    (
    Spi         aSpi    //!< the SPI bus to get its name
    )
{
    std::string spiString;

    switch( aSpi )
    {
        case SPI_0:
            spiString = "/dev/spidev0.0";
            printf("De10Nano.getSpiString -  /dev/spidev0.0\n");
            break;

        default:
			printf("De10Nano.getSpiString - no spiString\n");
            break;
    }

    return spiString;
}


//!************************************************************************
//! Propagate the status of the LTC SPI /SS pin.
//!
//! @returns: nothing
//!************************************************************************
void De10Nano::propagateLtcSpiSs
    (
    const bool   aStatus         //!< true if LTC SPI /SS status is active (low)
    )
{
    mLtcSsActive = aStatus;
    mLtcSsWasRead = false; // becomes true after next call to getLtcSpiSs()
}


//!************************************************************************
//! Get the last status of the LTC SPI /SS pin.
//! Needed for multi-byte transactions.
//!
//! @returns: true if LTC SPI /SS last status was low
//!************************************************************************
bool De10Nano::getLtcSpiLastSs()
{
    return mLtcSsWasRead;
}


//!************************************************************************
//! Get the status of the LTC SPI /SS pin.
//!
//! @returns: true if LTC SPI /SS pin is low
//!************************************************************************
bool De10Nano::getLtcSpiSs()
{
    mLtcSsWasRead = true;
    return mLtcSsActive;
}


//!************************************************************************
//! Set the LTC GPIO pin direction
//!
//! @returns: nothing
//!************************************************************************
void De10Nano::setLtcGpioDirection
    (
    const DioDirection  aDirection  //!< pin direction (I/O)
    )
{
    void* virtualBase;
    int   fd = 0;
	//printf("De10Nano.setLtcGpioDirection 0-unknown, 1-in, 2-out = %d\n",aDirection);
    if( -1 != ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) )
    {
        virtualBase = mmap( nullptr, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

        if( MAP_FAILED != virtualBase )
        {
            switch( aDirection )
            {
                case DIO_DIRECTION_INPUT:
                    alt_clrbits_word( reinterpret_cast<uint32_t>( virtualBase ) + ( reinterpret_cast<uint32_t>( ALT_GPIO1_SWPORTA_DDR_ADDR ) & HW_REGS_MASK ), LTC_GPIO_MASK );
                    break;

                case DIO_DIRECTION_OUTPUT:
                    alt_setbits_word( reinterpret_cast<uint32_t>( virtualBase ) + ( reinterpret_cast<uint32_t>( ALT_GPIO1_SWPORTA_DDR_ADDR ) & HW_REGS_MASK ), LTC_GPIO_MASK );
                    break;

                default:
                    break;
            }

            munmap( virtualBase, HW_REGS_SPAN );
        }

        close( fd );
    }
}


//!************************************************************************
//! Set the LTC GPIO pin status
//!
//! @returns: nothing
//!************************************************************************
void De10Nano::setLtcGpioStatus
    (
    const DioStatus     aStatus //!< pin status (L/H)
    )
{
    void* virtualBase;
    int   fd = 0;
	//printf("De10Nano.setLtcGpioStatus 0-unknown, 1-low, 2-high = %d\n",aStatus);
    if( -1 != ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) )
    {
        virtualBase = mmap( nullptr, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

        if( MAP_FAILED != virtualBase )
        {
            switch( aStatus )
            {
                case DIO_STATUS_LOW:
                    alt_clrbits_word( reinterpret_cast<uint32_t>( virtualBase ) + ( reinterpret_cast<uint32_t>( ALT_GPIO1_SWPORTA_DR_ADDR ) & HW_REGS_MASK ), LTC_GPIO_MASK );
                    break;

                case DIO_STATUS_HIGH:
                    alt_setbits_word( reinterpret_cast<uint32_t>( virtualBase ) + ( reinterpret_cast<uint32_t>( ALT_GPIO1_SWPORTA_DR_ADDR ) & HW_REGS_MASK ), LTC_GPIO_MASK );
                    break;

                default:
                    break;
            }

            munmap( virtualBase, HW_REGS_SPAN );
        }

        close( fd );
    }
}


//!************************************************************************
//! Get the LTC GPIO pin status
//!
//! @returns: The status of the digital pin
//!************************************************************************
De10Nano::DioStatus De10Nano::getLtcGpioStatus()
{
    DioStatus retVal = DIO_STATUS_UNKNOWN;

    void* virtualBase;
    int fd = 0;

    if( -1 != ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) )
    {
        virtualBase = mmap( nullptr, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

        if( MAP_FAILED != virtualBase )
        {
            uint32_t scanInput = alt_read_word( reinterpret_cast<uint32_t>( virtualBase ) + ( reinterpret_cast<uint32_t>( ALT_GPIO1_EXT_PORTA_ADDR ) & HW_REGS_MASK ) );

            if( ~scanInput & LTC_GPIO_MASK )
            {
                retVal = DIO_STATUS_LOW;
            }
            else
            {
                retVal = DIO_STATUS_HIGH;
            }

            munmap( virtualBase, HW_REGS_SPAN );
        }

        close( fd );
    }

    return retVal;
}


//!************************************************************************
//! Toggle the output of LTC GPIO pin.
//! Does NOT verify if it was previously set as output pin.
//!
//! @returns: nothing
//!************************************************************************
void De10Nano::toggleLtcGpio()
{
    void* virtualBase;
    int fd = 0;
	//printf("De10Nano.toggleLtcGpio\n");
    if( -1 != ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) )
    {
        virtualBase = mmap( nullptr, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

        if( MAP_FAILED != virtualBase )
        {
            uint32_t scanInput = alt_read_word( reinterpret_cast<uint32_t>( virtualBase ) + ( reinterpret_cast<uint32_t>( ALT_GPIO1_EXT_PORTA_ADDR ) & HW_REGS_MASK ) );

            if( ~scanInput & LTC_GPIO_MASK )
            {
                alt_setbits_word( reinterpret_cast<uint32_t>( virtualBase ) + ( reinterpret_cast<uint32_t>( ALT_GPIO1_SWPORTA_DR_ADDR ) & HW_REGS_MASK ), LTC_GPIO_MASK );
            }
            else
            {
                alt_clrbits_word( reinterpret_cast<uint32_t>( virtualBase ) + ( reinterpret_cast<uint32_t>( ALT_GPIO1_SWPORTA_DR_ADDR ) & HW_REGS_MASK ), LTC_GPIO_MASK );
            }

            munmap( virtualBase, HW_REGS_SPAN );
        }

        close( fd );
    }
}
