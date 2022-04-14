/*
WireWrapper.h

This file contains the definitions for the Arduino-to-Linux Wire wrapper.


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

#ifndef WireWrapper_h
#define WireWrapper_h

#include <cstdint>
#include <string>


//************************************************************************
// Class for wrapping Arduino I2C functionality in Embedded Linux
//************************************************************************
class WireWrapper
{
    //************************************************************************
    // data types and constants
    //************************************************************************
    public:
        //!< End of transmission statuses
        enum EndTransmissionStatus
        {
            END_TRANSMISSION_STATUS_OK              = 0,
            END_TRANSMISSION_STATUS_DATA_TOO_LONG   = 1,
            END_TRANSMISSION_STATUS_NACK_ADDRESS    = 2,
            END_TRANSMISSION_STATUS_NACK_DATA       = 3,
            END_TRANSMISSION_STATUS_OTHER_ERROR     = 4
        };


    private:
        static const uint16_t I2C_TX_BUFFER_LEN = 2048;     //!< Tx buffer length
        static const uint16_t I2C_RX_BUFFER_LEN = 2048;     //!< Rx buffer length


    //************************************************************************
    // functions
    //************************************************************************
    public:
        WireWrapper();

        ~WireWrapper();


        void setBus
            (
            std::string     busString   //!< I2C bus string
            );

        void begin();

        void beginTransmission
            (
            const uint8_t   address     //!< I2C slave address
            );

        int endTransmission();


        int requestFrom
            (
            const uint8_t   address,    //!< I2C register
            const uint16_t  quantity    //!< number of bytes to request
            );


        int write
            (
            const uint8_t   value       //!< byte to be written
            );


        int write
            (
            uint8_t*        data,       //!< byte array to be written
            const uint16_t  length      //!< size of the array
            );

        int write
            (
            char*           string      //!< string to be written
            );


        uint8_t read();


        int available();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        std::string mI2cBusString;                  //!< I2C bus string
        uint8_t     mI2cSlaveAddress;               //!< I2C slave address
        int         mI2cFile;                       //!< file used for I/O
        bool        mI2cBusConnected;               //!< true if the I2C bus is open
        bool        mI2cSlaveConnected;             //!< true if the I2C slave is connected

        uint8_t     mTxBuffer[I2C_TX_BUFFER_LEN];   //!< Tx buffer
        int         mTxBytesCount;                  //!< number of bytes in the Tx bufffer

        uint8_t     mRxBuffer[I2C_RX_BUFFER_LEN];   //!< Rx buffer
        int         mRxBytesCount;                  //!< number of bytes in the Rx buffer
};

#endif // WireWrapper_h
