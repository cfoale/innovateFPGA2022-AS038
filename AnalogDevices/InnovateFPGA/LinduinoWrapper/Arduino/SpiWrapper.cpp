/*
SpiWrapper.cpp
**************************************************************
Colin Michael Foale 2022 changed method 
bool SpiWrapper::transferWords 
It was modified to change endian assumption of the Linux device driver.
*********************************************************************
This file contains the sources for the Arduino-to-Linux SPI wrapper.


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

#include "SpiWrapper.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <De10Nano.h>

#ifdef DE10NANO_SUPPORT
    #include <sys/ioctl.h>
    #include <linux/spi/spidev.h>
#endif


//!************************************************************************
//! Constructor
//!************************************************************************
SpiWrapper::SpiWrapper()
    : mSpiFile( -1 )
    , mSpiConnected( false )

    , mSsActive( false )
    , mLastSsActive( false )

    , mClkSpeedHz( 1000000 )
    , mDataOrder( 0 )
    , mSpiMode( 1 )
    , mBitsPerWord( 8 )
{
}


//!************************************************************************
//! Destructor
//!************************************************************************
SpiWrapper::~SpiWrapper()
{
    end();
}


//!************************************************************************
//! Set the SPI bus to communicate over
//! Must be called before begin()
//!
//! @returns: nothing
//!************************************************************************
void SpiWrapper::setBus
    (
    std::string     busString      //!< SPI bus string
    )
{
    mSpiBusString = busString;
}


//!************************************************************************
//! Start the communication over the SPI bus and apply settings
//!
//! @returns: true if could connect to SPI
//!************************************************************************
bool SpiWrapper::begin()
{
    mSpiConnected = false;
    mSpiFile = -1;

    #ifdef DE10NANO_SUPPORT
        if( ( mSpiFile = ::open( mSpiBusString.c_str(), O_RDWR ) ) >= 0 )
        {
            mSpiConnected = true;
            setBitsPerWord( mBitsPerWord );
        }
    #endif

    if( !mSpiConnected )
    {
        printf( "\n Could not start a connection with SPI." );
    }

    return mSpiConnected;
}


//!************************************************************************
//! Disable the existing SPI communication
//!
//! @returns: nothing
//!************************************************************************
void SpiWrapper::end()
{
    #ifdef DE10NANO_SUPPORT
        ::close( mSpiFile );
    #endif

    mSpiFile = -1;
    mSpiConnected = false;
}


//!************************************************************************
//! Begin SPI transaction compatibility with Arduino.
//! Updated parametere are: DataMode, ClockHz, and BitOrder.
//!
//! @returns: nothing
//!************************************************************************
void SpiWrapper::beginTransaction
    (
    SPISettings settings    //!< Arduino SPI settings structure
    )
{
    setDataMode( settings.getDataMode() );
    setSpeedHz( settings.getSpeedHz() );
    setBitOrder( settings.getDataOrder() );
}


//!************************************************************************
//! End transaction compatibility.
//!
//! @returns: nothing
//!************************************************************************
void SpiWrapper::endTransaction()
{
    // Arduino calls to SPI.endTransaction() do not change parameters, only
    // next calls to SPI.beginTransaction() are used for changing them.
}


//!************************************************************************
//! Full-duplex I/O transfer
//!
//! @returns: true if transfer does not fail
//!************************************************************************
bool SpiWrapper::fullDuplexTxRx
    (
    uint8_t         send[],     //!< byte array to send
    uint8_t         receive[],  //!< byte array to receive
    const uint8_t   length      //!< byte array length
    )
{
    bool status( false );

    #ifdef DE10NANO_SUPPORT
        spi_ioc_transfer tr;
        memset( &tr, 0, sizeof( tr ) );

        tr.tx_buf = (unsigned long long)send;
        tr.rx_buf = (unsigned long long)receive;
        tr.len = length;
        tr.speed_hz = mClkSpeedHz;
        tr.bits_per_word = mBitsPerWord;

        if( ioctl( mSpiFile, SPI_IOC_MESSAGE( 1 ), &tr ) >= 0 )
        {
            status = true;
        }
    #endif

    return status;
}


//!************************************************************************
//! Full-duplex I/O transfer for words (16 bits per word)
//!
//! @returns: true if transfer does not fail
//!************************************************************************
// SPI.transferWords( tx, rxBArray, 2 ); //cmf - how this was called by LT_SPI.cpp

bool SpiWrapper::transferWords
    (
    uint8_t          send[],    //!< byte array to send, MSB first
    uint8_t          receive[], //!< byte array to receive, MSB first
    const uint8_t    length     //!< word array length
    )
{
    bool status( false );

    #ifdef DE10NANO_SUPPORT
		//printf("SpiWrapper::transferWords DE10NANO_SUPPORT\n");
		//cmf - SWITCH the order of words in send, to conform with LTC2668 data sheet
		uint8_t switched[4]; //we know length is 2
		switched[0] = send[2];
		switched[1] = send[3];
		switched[2] = send[0];
		switched[3] = send[1];
		
        spi_ioc_transfer tr; //cmf - this could be an array, e.g. [0] is a command, [1] is the received message
        memset( &tr, 0, sizeof( tr ) );
        
		/*printf("sizeof (send) is %d\n",sizeof( send));
		int len = sizeof(send);
		uint8_t *bp; //buffer pointer
	    for(bp = send;len;len--)
			printf("%02x ", *bp++);
		printf("\n");
		*/
        tr.tx_buf = (unsigned long long) switched; //cmf - send; //tr.tx_buf is 64 bits
        //printf("\nsizeof (tr.tx_buf) is %d\n",sizeof( tr.tx_buf ));
        
        tr.rx_buf = (unsigned long long)receive;
        tr.len = 2*length;//cmf - len should be the number of bytes, not words.
        tr.speed_hz = mClkSpeedHz;
        tr.bits_per_word = 16;

        if( ioctl( mSpiFile, SPI_IOC_MESSAGE( 1 ), &tr ) >= 0 ) //only one transaction, as tr is not an array - cmf
        {
            status = true;
            //printf("\nSpiWrapper::transferWords status = %d\n", status);
        }
    #endif

    return status;
}


//!************************************************************************
//! Set the SPI bit order
//!
//! @returns: nothing
//!************************************************************************
void SpiWrapper::setBitOrder
    (
    const uint8_t order     //!< Arduino SPI bit order
    )
{
    bool status = false;

    #ifdef DE10NANO_SUPPORT
        if( mSpiConnected )
        {
            switch( order )
            {
                case LSBFIRST:
                    mDataOrder = 1;
                    break;

                case MSBFIRST:
                    mDataOrder = 0;
                    break;

                default:
                    break;
            }

            if( -1 != ioctl( mSpiFile, SPI_IOC_WR_LSB_FIRST, &mDataOrder ) )
            {
                uint8_t readOrder = 0;

                if( -1 != ioctl( mSpiFile, SPI_IOC_RD_LSB_FIRST, &readOrder ) )
                {
                    status = ( readOrder == mDataOrder );
                }
            }
        }
    #endif

    //printf( "\n mDataOrder = %u", mDataOrder );

    if( !status )
    {
        printf( "\n SpiWrapper::setBitOrder() failed" );
    }
}


//!************************************************************************
//! Set the number of bits per word
//!
//! @returns: nothing
//!************************************************************************
void SpiWrapper::setBitsPerWord
    (
    const uint8_t bits  //!< number of bits
    )
{
    bool status = false;

    #ifdef DE10NANO_SUPPORT
        if( mSpiConnected )
        {
            mBitsPerWord = bits;

            if( -1 != ioctl( mSpiFile, SPI_IOC_WR_BITS_PER_WORD, &mBitsPerWord ) )
            {
                uint8_t bits = 0;

                if( -1 != ioctl( mSpiFile, SPI_IOC_RD_BITS_PER_WORD, &bits ) )
                {
                    status = ( mBitsPerWord == bits );
                }
            }
        }
    #endif

    //printf( "\n mBitsPerWord = %u", mBitsPerWord );

    if( !status )
    {
        printf( "\n SpiWrapper::setBitsPerWord() failed" );
    }
}


//!************************************************************************
//! Set the divider for SPI clock relative to the system clock
//! Function is obsolete and not recommended for new designs.
//!
//! @returns: nothing
//!************************************************************************
void SpiWrapper::setClockDivider
    (
    const uint8_t divider   //!< Arduino SPI CLK divider
    )
{
    bool status = false;

    #ifdef DE10NANO_SUPPORT
        if( mSpiConnected )
        {
            mClkSpeedHz = ARDUINO_UNO_R3_CLK_HZ;

            switch( divider )
            {
                case SPI_CLOCK_DIV2:
                    mClkSpeedHz /= 2;
                    break;

                case SPI_CLOCK_DIV4:
                    mClkSpeedHz /= 4;
                    break;

                case SPI_CLOCK_DIV8:
                    mClkSpeedHz /= 8;
                    break;

                case SPI_CLOCK_DIV16:
                    mClkSpeedHz /= 16;
                    break;

                case SPI_CLOCK_DIV32:
                    mClkSpeedHz /= 32;
                    break;

                case SPI_CLOCK_DIV64:
                    mClkSpeedHz /= 64;
                    break;

                case SPI_CLOCK_DIV128:
                    mClkSpeedHz /= 128;
                    break;

                default:
                    break;
            }

            if( -1 != ioctl( mSpiFile, SPI_IOC_WR_MAX_SPEED_HZ, &mClkSpeedHz ) )
            {
                uint32_t readSpeedHz = 0;

                if( -1 != ioctl( mSpiFile, SPI_IOC_RD_MAX_SPEED_HZ, &readSpeedHz ) )
                {
                    status = ( readSpeedHz == mClkSpeedHz );
                }
            }
        }
    #endif

    printf( "\n\n SPI.setClockDivider() is an obsolete function. Please replace all calls to it.\n\n" );

    if( !status )
    {
        printf( "\n SpiWrapper::setClockDivider() failed" );
    }
}


//!************************************************************************
//! Set the SPI data mode
//!
//! @returns: nothing
//!************************************************************************
void SpiWrapper::setDataMode
    (
    const uint8_t mode      //!< Arduino SPI data mode
    )
{
    bool status = false;

    #ifdef DE10NANO_SUPPORT
        if( mSpiConnected )
        {
            switch( mode )
            {
                case SPI_MODE0:
                    mSpiMode = SPI_MODE_0;
                    break;

                case SPI_MODE1:
                    mSpiMode = SPI_MODE_1;
                    break;

                case SPI_MODE2:
                    mSpiMode = SPI_MODE_2;
                    break;

                case SPI_MODE3:
                    mSpiMode = SPI_MODE_3;
                    break;

                default:
                    break;
            }

            if( -1 != ioctl( mSpiFile, SPI_IOC_WR_MODE, &mSpiMode ) )
            {
                uint8_t readMode = 0;

                if( -1 != ioctl( mSpiFile, SPI_IOC_RD_MODE, &readMode ) )
                {
                    status = ( readMode % 4 == mSpiMode );
                }
            }
        }
    #endif

    //printf( "\n mSpiMode = %u", mSpiMode );

    if( !status )
    {
        printf( "\n SpiWrapper::setDataMode() failed" );
    }
}


//!************************************************************************
//! Set the SPI clock speed
//!
//! @returns: nothing
//!************************************************************************
void SpiWrapper::setSpeedHz
    (
    const uint32_t speedHz  //!< transfer speed in Hz
    )
{
    bool status = false;

    #ifdef DE10NANO_SUPPORT
        if( mSpiConnected )
        {
            mClkSpeedHz = speedHz;

            if( mClkSpeedHz > ARDUINO_UNO_R3_CLK_HZ )
            {
                mClkSpeedHz = ARDUINO_UNO_R3_CLK_HZ;
            }

            if( -1 != ioctl( mSpiFile, SPI_IOC_WR_MAX_SPEED_HZ, &mClkSpeedHz ) )
            {
                uint32_t readSpeedHz = 0;

                if( -1 != ioctl( mSpiFile, SPI_IOC_RD_MAX_SPEED_HZ, &readSpeedHz ) )
                {
                    status = ( readSpeedHz == mClkSpeedHz );
                }
            }
        }
    #endif

    //printf( "\n mClkSpeedHz = %u", mClkSpeedHz );

    if( !status )
    {
        printf( "\n SpiWrapper::setSpeedHz() failed" );
    }
}


//!************************************************************************
//! Transfer a single byte via SPI
//!
//! @returns: read byte for 8-bit data transfers
//!************************************************************************
uint8_t SpiWrapper::transfer
    (
    uint8_t val       //!< one-byte value
    )
{
    uint8_t retVal = 0;

    static uint8_t txBuffer[2] = { 0 };
    static uint8_t rxBuffer[2] = { 0 };

    bool multiByteTransfer = false;

    static bool readByteAvailable = false;
    static uint8_t lastReadByte = 0;
    static uint8_t writeAddress = 0;

    #ifdef DE10NANO_SUPPORT
        if( mSpiConnected )
        {
            De10Nano* de10 = De10Nano::getInstance();

            if( de10 )
            {
                mLastSsActive = de10->getLtcSpiLastSs();

                // expected to be always true inside a transfer() call
                mSsActive = de10->getLtcSpiSs();
            }

            multiByteTransfer = mLastSsActive && mSsActive;

            //***********************************************************
            // 1. Reading one byte from an address
            // 1.1 Set register address and call IOCTL
            //***********************************************************
            if( ( val & 0x80 )          // val is the address to read from
               && !multiByteTransfer    // single byte transfer, or first one from a series
              )
            {
                readByteAvailable = false;
                writeAddress = 0x00;

                memset( txBuffer, 0, sizeof( txBuffer ) );
                memset( rxBuffer, 0, sizeof( rxBuffer ) );
                txBuffer[0] = val; // address to read from

                spi_ioc_transfer tr;
                memset( &tr, 0, sizeof( tr ) );
                tr.len = 2;
                tr.tx_buf = (unsigned long)txBuffer;
                tr.rx_buf = (unsigned long)rxBuffer;
                tr.speed_hz = mClkSpeedHz;
                tr.bits_per_word = mBitsPerWord;

                if( ioctl( mSpiFile, SPI_IOC_MESSAGE( 1 ), &tr ) >= 0 )
                {
                    // The read byte is known and returned at this point.
                    //
                    // If the next call is transfer(0x00), this byte must be still available to read
                    lastReadByte = rxBuffer[1];
                    readByteAvailable = true;
                }
                else
                {
                    lastReadByte = 0;
                }

                retVal = lastReadByte;

                memset( txBuffer, 0, sizeof( txBuffer ) );
                memset( rxBuffer, 0, sizeof( rxBuffer ) );
            }
            //***********************************************************
            // 1. Reading one byte from an address
            // 1.2 Return the most recent read byte
            //***********************************************************
            else if( ( 0x00 == val )        // must-have if intention is to read most recent received byte
                  && readByteAvailable      // previous transfer was a read inquiry
                  && multiByteTransfer      // must not be first byte from a series
                   )
            {
                retVal = lastReadByte;
                readByteAvailable = false;

                memset( txBuffer, 0, sizeof( txBuffer ) );
                memset( rxBuffer, 0, sizeof( rxBuffer ) );
            }
            else
            {
                // needed when writing and reading are done using the same transfer call
                readByteAvailable = false;

                //***********************************************************
                // 2. Writing
                // 2.1 Store the start address to write to.
                //     If multiple bytes are written, it will be auto-incremented for each
                //     upcoming byte.
                //***********************************************************              
                if( ( 0x00 != val )         // if inquiry is to write, first byte is a non-zero address
                 && !multiByteTransfer      // single byte transfer, or first one from a series
                  )
                {
                    writeAddress = val;
                    txBuffer[0] = writeAddress;
                }
                //***********************************************************
                // 2. Writing
                // 2.1 Write val to the original address (single byte)
                //     or to the auto-incremented address (multiple bytes).
                //***********************************************************
                else if( multiByteTransfer )         // true beginning with two consecutive transfer() calls
                {
                    txBuffer[0] = writeAddress++;
                    txBuffer[1] = val;
                    memset( rxBuffer, 0, sizeof( rxBuffer ) );

                    spi_ioc_transfer tr;
                    memset( &tr, 0, sizeof( tr ) );
                    tr.len = 2;
                    tr.tx_buf = (unsigned long)txBuffer;
                    tr.rx_buf = (unsigned long)rxBuffer;
                    tr.speed_hz = mClkSpeedHz;
                    tr.bits_per_word = mBitsPerWord;

                    if( ioctl( mSpiFile, SPI_IOC_MESSAGE( 1 ), &tr ) >= 0 )
                    {
                        retVal = rxBuffer[1];
                    }

                    memset( txBuffer, 0, sizeof( txBuffer ) );
                    memset( rxBuffer, 0, sizeof( rxBuffer ) );
                }
            }
        }
    #endif

    return retVal;
}


//!************************************************************************
//! Full-duplex I/O transfer over SPI for byte arrays
//!
//! @returns: true if the byte array transfer does not fail
//!************************************************************************
bool SpiWrapper::transfer
    (
    uint8_t         buffer[],   //!< byte array to write
    const uint8_t   size        //!< length of byte array
    )
{
    uint8_t rx = 0x00;
    return fullDuplexTxRx( buffer, &rx, size );
}


//!************************************************************************
//! Read a byte from a register over SPI
//!
//! @returns: the read byte from specified address
//!************************************************************************
uint8_t SpiWrapper::readRegister
    (
    const uint8_t registerAddress       //!< register address
    )
{
    uint8_t txArray[2] = { 0 };
    uint8_t rxArray[2] = { 0 };
    txArray[0] = 0x80 | registerAddress;
    fullDuplexTxRx( txArray, rxArray, 2 );
    return rxArray[1];
}


//!************************************************************************
//! Write a byte to a register
//!
//! @returns: received byte
//!************************************************************************
uint8_t SpiWrapper::writeRegister
    (
    const uint8_t   registerAddress,    //!< register address
    const uint8_t   value               //!< byte to write
    )
{   
    uint8_t txArray[2] = { registerAddress, value };
    uint8_t rxArray[2] = { 0 };
    fullDuplexTxRx( txArray, rxArray, 2 );
    return rxArray[1];
}


//!************************************************************************
//! Write one byte
//!
//! @returns: received byte
//!************************************************************************
uint8_t SpiWrapper::writeByte
    (
    uint8_t   value             //!< byte to write
    )
{
    uint8_t rx = 0x00;
    fullDuplexTxRx( &value, &rx, 1 );
    return rx;
}
