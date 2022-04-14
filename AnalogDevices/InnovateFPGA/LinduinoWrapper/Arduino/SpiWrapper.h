/*
SpiWrapper.h

This file contains the definitions for the Arduino-to-Linux SPI wrapper.


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

#ifndef SpiWrapper_h
#define SpiWrapper_h

#include <cstdint>
#include <string>


//************************************************************************
// data types and constants
//************************************************************************
//!< SPI data order as defined in Arduino
typedef enum
{
    LSBFIRST,
    MSBFIRST
}DataOrder;


//!< SPI data mode as defined in Arduino
typedef enum
{
    SPI_MODE0 = 0x00,
    SPI_MODE1 = 0x04,
    SPI_MODE2 = 0x08,
    SPI_MODE3 = 0x0C
}SpiDataMode;


//!< SPI clock dividers as defined in Arduino
typedef enum
{
    SPI_CLOCK_DIV4   = 0x00,
    SPI_CLOCK_DIV16  = 0x01,
    SPI_CLOCK_DIV64  = 0x02,
    SPI_CLOCK_DIV128 = 0x03,
    SPI_CLOCK_DIV2   = 0x04,
    SPI_CLOCK_DIV8   = 0x05,
    SPI_CLOCK_DIV32  = 0x06
}SpiClockDiv;


//!< Class for SPISettings as defined in Arduino
class SPISettings
{ 
    public:
        //!************************************************************************
        //! Constructor
        //!************************************************************************
        SPISettings()
        {
            mClkSpeedHz = 1000000;
            mDataOrder  = MSBFIRST;
            mDataMode   = SPI_MODE1;
        }

        //!************************************************************************
        //! Constructor
        //!************************************************************************
        SPISettings
            (
            uint32_t    aClkSpeedHz,
            DataOrder   aDataOrder,
            SpiDataMode aDataMode
            )
        {
            mClkSpeedHz = aClkSpeedHz;
            mDataOrder = aDataOrder;
            mDataMode = aDataMode;
        }

        //!************************************************************************
        //! Get the SPI bus speed
        //!
        //! @returns: The speed in Hz
        //!************************************************************************
        uint32_t getSpeedHz()
        {
            return mClkSpeedHz;
        }

        //!************************************************************************
        //! Get the SPI data order
        //!
        //! @returns: The data order
        //!************************************************************************
        uint8_t getDataOrder()
        {
            return static_cast<uint8_t>( mDataOrder );
        }

        //!************************************************************************
        //! Get the SPI data mode
        //!
        //! @returns: The data mode
        //!************************************************************************
        uint8_t getDataMode()
        {
            return static_cast<uint8_t>( mDataMode );
        }

    private:
        uint32_t        mClkSpeedHz;    //!< Arduino SPI clock in Hz
        DataOrder       mDataOrder;     //!< Arduino SPI data order
        SpiDataMode     mDataMode;      //!< Arduino SPI data mode
};


//************************************************************************
// Class for wrapping Arduino SPI functionality in Embedded Linux
//************************************************************************
class SpiWrapper
{
    //************************************************************************
    // data types and constants
    //************************************************************************
    private:
        static const uint32_t ARDUINO_UNO_R3_CLK_HZ = 16000000; //!< reference system clock


    //************************************************************************
    // functions
    //************************************************************************
    public:
        SpiWrapper();

        ~SpiWrapper();


        void setBus
            (
            std::string     busString      //!< SPI bus string
            );


        bool begin();

        void end();


        void beginTransaction
            (
            SPISettings settings    //!< SPI settings
            );

        void endTransaction();


        void setBitOrder
            (
            const uint8_t order     //!< Arduino SPI bit order
            );


        void setClockDivider
            (
            const uint8_t divider   //!< Arduino SPI CLK divider
            );


        void setDataMode
            (
            const uint8_t mode      //!< Arduino SPI data mode
            );


        uint8_t transfer
            (
            uint8_t         val         //!< one-byte value
            );

        bool transfer
            (
            uint8_t         buffer[],   //!< byte array to write
            const uint8_t   size        //!< length of byte array
            );

        bool transferWords
            (
            uint8_t          send[],    //!< byte array to send, MSB first
            uint8_t          receive[], //!< byte array to receive, MSB first
            const uint8_t    length     //!< word array length
            );


        uint8_t readRegister
            (
            const uint8_t   registerAddress     //!< register address
            );

        uint8_t writeRegister
            (
            const uint8_t   registerAddress,    //!< register address
            const uint8_t   value               //!< byte to write
            );

        uint8_t writeByte
            (
            uint8_t   value             //!< byte to write
            );

    private:
        void setBitsPerWord
            (
            const uint8_t   bits        //!< number of bits (not Arduino related)
            );

        void setSpeedHz
            (
            const uint32_t  speedHz     //!< clock speed in Hz
            );

        bool fullDuplexTxRx
            (
            uint8_t         send[],     //!< byte array to send
            uint8_t         receive[],  //!< byte array to receive
            const uint8_t   length      //!< byte array length
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        std::string mSpiBusString;          //!< SPI bus string
        int         mSpiFile;               //!< file used for I/O
        bool        mSpiConnected;          //!< if the SPI connection was established

        bool        mSsActive;              //!< true if /SS is active (low)
        bool        mLastSsActive;          //!< true if last /SS was active (low)

        uint32_t    mClkSpeedHz;            //!< SPI CLK frequency in Hz
        uint8_t     mDataOrder;             //!< SPI data order (Linux)
        uint8_t     mSpiMode;               //!< SPI mode 0/1/2/3 (Linux)
        uint8_t     mBitsPerWord;           //!< SPI bits per word (Linux)
};

#endif // SpiWrapper_h

