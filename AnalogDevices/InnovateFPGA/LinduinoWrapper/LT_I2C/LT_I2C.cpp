/*
LT_I2C.cpp

This file contains the Wire wrapper sources for Linduino.


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

#include "LT_I2C.h"

#include <string>
#include <De10Nano.h>
#include <Arduino.h>


WireWrapper Wire;

//!************************************************************************
//! Get the high byte from a word
//!
//! @returns: the high byte
//!************************************************************************
static uint8_t getHighByte
    (
    const uint16_t word     //!< data word
    )
{
    return static_cast<uint8_t>( ( word >> 8 ) & 0x00FF );
}


//!************************************************************************
//! Get the low byte from a word
//!
//! @returns: the low byte
//!************************************************************************
static uint8_t getLowByte
    (
    const uint16_t word     //!< data word
    )
{
    return static_cast<uint8_t>( word & 0x00FF );
}


//!************************************************************************
//! Read one byte
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_read_byte
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t*    value       //!< byte to be read
    )
{
    int couldRead = 1;

    if( value )
    {
        Wire.requestFrom( static_cast<int>( address ), 1 );

        if( 1 <= Wire.available() )
        {
            *value = Wire.read();
            couldRead = 0;
        }
    }

    return couldRead;
}


//!************************************************************************
//! Write one byte to the device at specified address
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_write_byte
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     value       //!< byte to be written
    )
{
    Wire.beginTransmission( address );
    Wire.write( value );
    int status = Wire.endTransmission();

    return ( WireWrapper::END_TRANSMISSION_STATUS_OK == status ? 0 : 1 );
}


//!************************************************************************
//! Read one byte from specified register
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_read_byte_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     command,    //!< register address
    uint8_t*    value       //!< byte to be read
    )
{
    int couldRead = 1;

    if( value )
    {
        Wire.beginTransmission( address );
        Wire.write( command );

        if( WireWrapper::END_TRANSMISSION_STATUS_OK == Wire.endTransmission() )
        {
            Wire.requestFrom( static_cast<int>( address ), 1 );

            if( 1 <= Wire.available() )
            {
                *value = Wire.read();
                couldRead = 0;
            }
        }
    }

    return couldRead;
}


//!************************************************************************
//! Write one byte to the specified register
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_write_byte_data
    (
    uint8_t address,        //!< 7-bit I2C address
    uint8_t command,        //!< register address
    uint8_t value           //!< byte to be written
    )
{
    Wire.beginTransmission( address );
    Wire.write( command );
    Wire.write( value );
    int status = Wire.endTransmission();

    return ( WireWrapper::END_TRANSMISSION_STATUS_OK == status ? 0 : 1 );
}


//!************************************************************************
//! Read a word from a register
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_read_word_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     command,    //!< register address
    uint16_t*   value       //!< word to be read
    )
{
    int couldRead = 1;

    if( value )
    {
        Wire.beginTransmission( address );
        Wire.write( command );

        if( WireWrapper::END_TRANSMISSION_STATUS_OK == Wire.endTransmission() )
        {
            Wire.requestFrom( static_cast<int>( address ), 2 );

            if( 2 <= Wire.available() )
            {
                uint8_t hiByte = Wire.read();       // MSB first
                uint8_t loByte = Wire.read();       // LSB second
                *value = ( hiByte << 8  ) | loByte;
                couldRead = 0;
            }
        }
    }

    return couldRead;
}


//!************************************************************************
//! Write a word to a register
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_write_word_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     command,    //!< register address
    uint16_t    value       //!< word to be written
    )
{
    Wire.beginTransmission( address );
    Wire.write( command );
    Wire.write( getHighByte( value ) );     // MSB first
    Wire.write( getLowByte( value ) );      // LSB second
    int status = Wire.endTransmission();

    return ( WireWrapper::END_TRANSMISSION_STATUS_OK == status ? 0 : 1 );
}


//!************************************************************************
//! Read a byte array, starting at specified register address
//! and ending at (command + length - 1)
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_read_block_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     command,    //!< register address
    uint8_t     length,     //!< length of array
    uint8_t*    values      //!< byte array to be read
    )
{
    int couldRead = 1;

    if( values )
    {
        Wire.beginTransmission( address );
        Wire.write( command );

        if( WireWrapper::END_TRANSMISSION_STATUS_OK == Wire.endTransmission() )
        {
            Wire.requestFrom( static_cast<int>( address ), length );

            if( length <= Wire.available() )
            {
                for( uint8_t i = 0; i < length; i++ )
                {
                    values[i] = Wire.read();
                }

                couldRead = 0;
            }
        }
    }

    return couldRead;
}


//!************************************************************************
//! Read a block of data, no command byte, reads length number of bytes
//! and stores them in the byte array
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_read_block_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     length,     //!< length of array
    uint8_t*    values      //!< byte array to be read
    )
{
    int couldRead = 1;

    if( values )
    {
        Wire.requestFrom( static_cast<int>( address ), length );

        if( length <= Wire.available() )
        {
            for( uint8_t i = 0; i < length; i++ )
            {
                values[i] = Wire.read();
            }

            couldRead = 0;
        }
    }

    return couldRead;
}


//!************************************************************************
//! Write a block of data, starting at specified register address
//! and ending at (command + length - 1)
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_write_block_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     command,    //!< register address
    uint8_t     length,     //!< length of array
    uint8_t*    values      //!< byte array to be written
    )
{
    int couldWrite = 1;

    if( values )
    {
        Wire.beginTransmission( address );
        Wire.write( command );

        for( uint8_t i = 0; i < length; i++ )
        {
            Wire.write( values[i] );
        }

        if( WireWrapper::END_TRANSMISSION_STATUS_OK == Wire.endTransmission() )
        {
            couldWrite = 0;
        }
    }

    return couldWrite;
}


//!************************************************************************
//! Write a two bytes command and receive a block of bytes
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_two_byte_command_read_block
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint16_t    command,    //!< command word
    uint8_t     length,     //!< length of array
    uint8_t*    values      //!< byte array to be read
    )
{
    int couldRead = 1;

    if( values )
    {
        Wire.beginTransmission( address );
        Wire.write( getHighByte( command ) );   // MSB first
        Wire.write( getLowByte( command ) );    // LSB second

        if( WireWrapper::END_TRANSMISSION_STATUS_OK == Wire.endTransmission() )
        {
            Wire.requestFrom( static_cast<int>( address ), length );

            if( length <= Wire.available() )
            {
                for( uint8_t i = 0; i < length; i++ )
                {
                    values[i] = Wire.read();
                }

                couldRead = 0;
            }
        }
    }

    return couldRead;
}


//!************************************************************************
//! Write one byte
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_write
    (
    uint8_t     data        //!< byte to be written
    )
{
    int bytesWritten = Wire.write( data );
    return ( bytesWritten > 0 ) ? 0 : 1;
}


//!************************************************************************
//! Read one byte
//!
//! @returns: the byte read
//!************************************************************************
uint8_t i2c_read
    (
    int8_t      ack         //!< acknowledge flag
    )
{
    (void)ack;
    uint8_t readByte = 0;

    if( 1 <= Wire.available() )
    {
        readByte = Wire.read();
    }

    return readByte;
}


//!************************************************************************
//! Poll the I2C port
//!
//! @returns: 0 on success, 1 otherwise
//!************************************************************************
int8_t i2c_poll
    (
    uint8_t     i2c_address //!< I2C address of the slave being polled
    )
{
    Wire.beginTransmission( i2c_address );
    int status = Wire.endTransmission();

    return ( WireWrapper::END_TRANSMISSION_STATUS_OK == status ? 0 : 1 );
}


//!************************************************************************
//! Switch the MUX to connect I2C pins to QuikEval connector.
//! This will disconnect the SPI pins.
//!
//! @returns: nothing
//!************************************************************************
void quikeval_I2C_connect(void)
{
    De10Nano* de10 = De10Nano::getInstance();

    if( de10 )
    {
        if( De10Nano::DIO_STATUS_LOW == de10->getLtcGpioStatus() )
        {
            de10->setLtcGpioStatus( De10Nano::DIO_STATUS_HIGH );
        }
    }
}


//!************************************************************************
//! Get the I2C bus
//!
//! @returns: the WireWrapper object containing the I2C bus
//!************************************************************************
WireWrapper quikeval_I2C_get_wire()
{
    return Wire;
}


//!************************************************************************
//! Initialize the Linduino I2C port.
//!
//! @returns: nothing
//!************************************************************************
void quikeval_I2C_init()
{
    De10Nano* de10 = De10Nano::getInstance();

    if( de10 )
    {
        std::string i2cBusString = de10->getI2cString( De10Nano::I2C_LTC_QUIKEVAL );
        Wire.setBus( i2cBusString );
        Wire.begin();
    }
}
