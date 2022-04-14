/*
WireWrapper.cpp

This file contains the sources for the Arduino-to-Linux Wire wrapper.


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

#include "WireWrapper.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <De10Nano.h>

#ifdef DE10NANO_SUPPORT
    #include <sys/ioctl.h>
    #include <linux/i2c.h>
    #include <linux/i2c-dev.h>
#endif


//!************************************************************************
//! Constructor
//!************************************************************************
WireWrapper::WireWrapper()
    : mI2cSlaveAddress( 0 )
    , mI2cFile( -1 )
    , mI2cBusConnected( false )
    , mI2cSlaveConnected( false )
    , mTxBytesCount( 0 )
    , mRxBytesCount( 0 )
{
    memset( mTxBuffer, 0, sizeof( mTxBuffer ) );
    memset( mRxBuffer, 0, sizeof( mRxBuffer ) );
}


//!************************************************************************
//! Destructor
//!************************************************************************
WireWrapper::~WireWrapper()
{
    #ifdef DE10NANO_SUPPORT
        ::close( mI2cFile );
    #endif
}


//!************************************************************************
//! Set the I2C bus to communicate over
//! Must be called before begin()
//!
//! @returns: nothing
//!************************************************************************
void WireWrapper::setBus
    (
    std::string     busString   //!< I2C bus string
    )
{
    mI2cBusString = busString;
}


//!************************************************************************
//! Begin the I2C communication.
//! setBus() and setSlaveAddress() must be called before this function.
//!
//! @returns: nothing
//!************************************************************************
void WireWrapper::begin()
{
    #ifdef DE10NANO_SUPPORT
        if( -1 != mI2cFile )
        {
            ::close( mI2cFile );
        }
    #endif

    mI2cBusConnected = false;
    mI2cSlaveConnected = false;
    mI2cFile = -1;

    mTxBytesCount = 0;
    mRxBytesCount = 0;

    memset( mTxBuffer, 0, sizeof( mTxBuffer ) );
    memset( mRxBuffer, 0, sizeof( mRxBuffer ) );

    #ifdef DE10NANO_SUPPORT
        if( ( mI2cFile = ::open( mI2cBusString.c_str(), O_RDWR ) ) >= 0 )
        {
            mI2cBusConnected = true;
        }
    #endif
}


//!************************************************************************
//! Initialize Tx buffer content
//!
//! @returns: nothing
//!************************************************************************
void WireWrapper::beginTransmission
    (
    const uint8_t   address     //!< I2C slave address
    )
{
    mI2cSlaveAddress = address;
    mI2cSlaveConnected = false;

    mTxBytesCount = 0;
    mRxBytesCount = 0;

    memset( mTxBuffer, 0, sizeof( mTxBuffer ) );
    memset( mRxBuffer, 0, sizeof( mRxBuffer ) );

    if( mI2cBusConnected )
    {
        ::close( mI2cFile );

        if( ( mI2cFile = ::open( mI2cBusString.c_str(), O_RDWR ) ) >= 0 )
        {
            mI2cBusConnected = true;

            if( ioctl( mI2cFile, I2C_SLAVE, mI2cSlaveAddress ) >= 0 )
            {
                mI2cSlaveConnected = true;
            }
        }
    }
}


//!************************************************************************
//! Writes all bytes from the Tx buffer to the I2C bus
//!
//! @returns: Status after sending data over I2C
//!************************************************************************
int WireWrapper::endTransmission()
{
    int status = END_TRANSMISSION_STATUS_OTHER_ERROR;

    #ifdef DE10NANO_SUPPORT
        if( mI2cBusConnected && mI2cSlaveConnected )
        {
            if( mTxBytesCount == ::write( mI2cFile, mTxBuffer, mTxBytesCount ) )
            {
                status = END_TRANSMISSION_STATUS_OK;
            }
        }
    #endif

    return status;
}


//!************************************************************************
//! Requests data from the Rx buffer
//!
//! @returns: Number of bytes read
//!************************************************************************
int WireWrapper::requestFrom
    (
    const uint8_t   address,    //!< I2C register
    const uint16_t  quantity    //!< number of bytes to request
    )
{
    mRxBytesCount = 0;
    (void)address;

    #ifdef DE10NANO_SUPPORT
        if( mI2cBusConnected && mI2cSlaveConnected )
        {
            if( quantity <= I2C_RX_BUFFER_LEN )
            {
                memset( mRxBuffer, 0, sizeof( mRxBuffer ) );
                mRxBytesCount = ::read( mI2cFile, mRxBuffer, quantity );
            }
        }
    #endif

    return mRxBytesCount;
}


//!************************************************************************
//! Write a single byte to the Tx buffer after existing content
//!
//! @returns: Total number of bytes from the Tx buffer
//!************************************************************************
int WireWrapper::write
    (
    const uint8_t   value   //!< byte to be written
    )
{
    if( mI2cBusConnected && mI2cSlaveConnected )
    {
        // prevent END_TRANSMISSION_STATUS_DATA_TOO_LONG
        if( mTxBytesCount + 1 <= I2C_TX_BUFFER_LEN )
        {
            mTxBuffer[mTxBytesCount] = value;
            mTxBytesCount++;
        }
    }

    return mTxBytesCount;
}


//!************************************************************************
//! Write a byte array to the Tx buffer after existing content
//!
//! @returns: Total number of bytes from the Tx buffer
//!************************************************************************
int WireWrapper::write
    (
    uint8_t*        data,   //!< byte array to be written
    const uint16_t  length  //!< size of the array
    )
{
    if( mI2cBusConnected && mI2cSlaveConnected )
    {
        // prevent END_TRANSMISSION_STATUS_DATA_TOO_LONG
        if( mTxBytesCount + length <= I2C_TX_BUFFER_LEN )
        {
            memcpy( &mTxBuffer + mTxBytesCount, data, length );
            mTxBytesCount += length;
        }
    }

    return mTxBytesCount;
}


//!************************************************************************
//! Write a string to the Tx buffer after existing content
//!
//! @returns: Total number of bytes from the Tx buffer
//!************************************************************************
int WireWrapper::write
    (
    char*           string  //!< string to be written
    )
{
    if( mI2cBusConnected && mI2cSlaveConnected )
    {
        if( string )
        {
            size_t strLength = strlen( string );

            // prevent END_TRANSMISSION_STATUS_DATA_TOO_LONG
            if( mTxBytesCount + strLength <= I2C_TX_BUFFER_LEN )
            {
                memcpy( &mTxBuffer + mTxBytesCount, string, strLength );
                mTxBytesCount += strLength;
            }
        }
    }

    return mTxBytesCount;
}


//!************************************************************************
//! Get the first byte from the Rx buffer and shifts left the buffer
//!
//! @returns: First byte from the Rx buffer
//!************************************************************************
uint8_t WireWrapper::read()
{
    uint8_t retVal = 0;

    if( mI2cBusConnected && mI2cSlaveConnected )
    {
        if( mRxBytesCount > 0 )
        {
            retVal = mRxBuffer[0];

            for( uint8_t i = 0; i < mRxBytesCount - 1; i++ )
            {
                mRxBuffer[i] = mRxBuffer[i + 1];
            }

            mRxBytesCount--;

            if( 0 == mRxBytesCount )
            {
                memset( mRxBuffer, 0, sizeof( mRxBuffer ) );
            }
        }
    }

    return retVal;
}


//!************************************************************************
//! Get the number of available bytes in Rx buffer
//!
//! @returns: nothing
//!************************************************************************
int WireWrapper::available()
{
    return mRxBytesCount;
}
