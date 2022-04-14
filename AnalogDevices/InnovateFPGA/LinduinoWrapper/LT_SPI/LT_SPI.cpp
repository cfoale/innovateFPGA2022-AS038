/*
LT_SPI.cpp

This file contains the SPI wrapper sources for Linduino.


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

#include "LT_SPI.h"

#include <De10Nano.h>
#include <Arduino.h>


static SPISettings spiSettings;
SpiWrapper SPI;


//!************************************************************************
//! Apply or update SPI settings
//!
//! @returns: nothing
//!************************************************************************
void quikeval_SPI_apply_settings
    (
    SPISettings settings        //!< SPI settings to be applied
    )
{
    spiSettings = settings;
}


//!************************************************************************
//! Switch the MUX to connect SPI pins to QuikEval connector.
//! This will disconnect the I2C pins.
//!
//! @returns: nothing
//!************************************************************************
void quikeval_SPI_connect()
{
    De10Nano* de10 = De10Nano::getInstance();

    if( de10 )
    {
        if( De10Nano::DIO_STATUS_HIGH == de10->getLtcGpioStatus() )
        {
            de10->setLtcGpioStatus( De10Nano::DIO_STATUS_LOW );
        }
    }
}


//!************************************************************************
//! Initialize SPI. Must be called before using the other SPI routines.
//!
//! @returns: true if SPI is initialized
//!************************************************************************
bool quikeval_SPI_init()
{
    bool isConnected( false );
    De10Nano* de10 = De10Nano::getInstance();
	//printf("LT_SPI.cpp quikeval_SPI_init\n");
    if( de10 )
    {
        std::string spiBusString = de10->getSpiString( De10Nano::SPI_0 );

        // the following call must precede begin()
        SPI.setBus( spiBusString );

        isConnected = SPI.begin();

        if( isConnected )
        {
            SPI.beginTransaction( spiSettings );
        }
    }

    return isConnected;
}


//!************************************************************************
//! Setup for hardware SPI communication by dividing the nominal clock.
//! SPI.setClockDivider() is an obsolete Arduino call and should be avoided.
//!
//! @returns: nothing
//!************************************************************************
void spi_enable
    (
    uint8_t spi_clock_divider   //!< SPI clock divider
    )
{
    SPI.begin();
    SPI.setClockDivider( spi_clock_divider );
}


//!************************************************************************
//! Disable SPI
//!
//! @returns: nothing
//!************************************************************************
void spi_disable()
{
    SPI.end();
}


//!************************************************************************
//! Read and write a data byte
//!
//! @returns: the data byte read
//!************************************************************************
int8_t spi_read
    (
    int8_t  data        //!< byte to be written
    )
{
    int8_t readByte = 0;
	printf("LT_SPI.cpp spi_read\n");
    SPI.beginTransaction( spiSettings );
    readByte = static_cast<int8_t>( SPI.writeByte( static_cast<uint8_t>( data ) ) );
    SPI.endTransaction();
	printf("LT_SPI.cpp spi_read %x\n",readByte);
    return readByte;
}


//!************************************************************************
//! Send and read a byte array
//!
//! @returns: nothing
//!************************************************************************

void spi_transfer_block
    (
    uint8_t     cs_pin,         //!< Chip select pin
    uint8_t*    tx,             //!< Byte array to be transmitted
    uint8_t*    rx,             //!< Byte array to be received
    uint8_t     length,         //!< Length of array
    uint8_t     bits_per_word,  //!< Bits per word ..16
    uint8_t     bits_to_shift,  //!< Number of bits to shift ..8
    uint8_t     shift_direction //!< Direction to shift (0=L,1=R) ..L
    )
{
	//printf("LT_SPI.cpp spi_transfer_block bits_per_word %d\n",bits_per_word);
    switch( bits_per_word )
    {
        case 8:
            for( int8_t i = length - 1; i >= 0; i-- )
            {
                spi_transfer_byte( cs_pin, tx[i], &rx[i] );
            }
            break;

        case 16: //cmf - this is the case LTC2668_write calls into
            {
                uint8_t rxBArray[4] = { 0 };
                SPI.transferWords( tx, rxBArray, 2 );

                uint32_t rxDWord = 0;
                rxDWord  = rxBArray[0];
                rxDWord |= rxBArray[1] << 8;
                rxDWord |= rxBArray[2] << 16;
                rxDWord |= rxBArray[3] << 24;

                if( bits_to_shift )
                {
                    if( SPI_DIRECTION_TO_SHIFT_LEFT == shift_direction )
                    {
                        rxDWord <<= bits_to_shift;
                    }
                    else
                    {
                        rxDWord >>= bits_to_shift;
                    }
                }

                rx[3] = static_cast<uint8_t>( ( rxDWord >> 24 ) & 0x000000FF );
                rx[2] = static_cast<uint8_t>( ( rxDWord >> 16 ) & 0x000000FF );
                rx[1] = static_cast<uint8_t>( ( rxDWord >>  8 ) & 0x000000FF );
                rx[0] = static_cast<uint8_t>( ( rxDWord       ) & 0x000000FF );
            }
            break;

        default:
            printf( "\n spi_transfer_block() - unsupported BPW" );
            break;
    }
}


//!************************************************************************
//! Byte transfer over SPI
//!
//! @returns: nothing
//!************************************************************************
void spi_transfer_byte
    (
    uint8_t     cs_pin,     //!< Chip Select pin
    uint8_t     tx,         //!< transmitted byte
    uint8_t*    rx          //!< received byte
    )
{
    (void)cs_pin;

    SPI.beginTransaction( spiSettings );
    *rx = SPI.writeByte( tx );
    SPI.endTransaction();
    printf("LT_SPI.cpp spi_transfer_byte tx %x rx %x\n",tx,*rx);
}


//!************************************************************************
//! Word transfer over SPI
//!
//! @returns: nothing
//!************************************************************************
void spi_transfer_word
    (
    uint8_t     cs_pin,     //!< Chip Select pin
    uint16_t    tx,         //!< transmitted word
    uint16_t*   rx          //!< received word
    )
{
    (void)cs_pin;

    union
    {
        uint8_t  b[2];
        uint16_t w;
    }data_tx;

    union
    {
        uint8_t  b[2];
        uint16_t w;
    }data_rx;

    data_tx.w = tx;

    SPI.beginTransaction( spiSettings );
    data_rx.b[1] = SPI.writeByte( data_tx.b[1] );  // Read MSB and send MSB
    data_rx.b[0] = SPI.writeByte( data_tx.b[0] );  // Read LSB and send LSB
    *rx = data_rx.w;
    SPI.endTransaction();
}


//!************************************************************************
//! Write one byte to SPI
//!
//! @returns: nothing
//!************************************************************************
void spi_write
    (
    int8_t  data        //!< byte to be written
    )
{
    SPI.beginTransaction( spiSettings );
    SPI.writeByte( static_cast<uint8_t>( data ) );
    SPI.endTransaction();
    printf("LT_SPI.cpp spi_write %x\n", data);
}
