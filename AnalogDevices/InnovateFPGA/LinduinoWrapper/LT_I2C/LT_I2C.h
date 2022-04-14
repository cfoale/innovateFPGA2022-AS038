/*
LT_I2C.h

This file contains the Wire wrapper definitions for Linduino.


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

#ifndef LT_I2C_h
#define LT_I2C_h

#include <stdint.h>
#include <WireWrapper.h>

#ifndef QUIKEVAL_CS
#define QUIKEVAL_CS SS
#endif


//************************************************************************
//
// Defines and macros taken from the original LT_I2C.h from
// https://github.com/analogdevicesinc/Linduino/tree/master/LTSketchbook/libraries/LT_I2C
//
// Kept here so that compiling the original Linduino drivers does not fail.
//
// Most of these constants should not be used in Linux, because they point to Atmel/AVR hardware.
// Ported functions within this module are not using them.
//
//************************************************************************
//! @name HARDWARE I2C PRESCALER VALUES
//! @{
#define HARDWARE_I2C_PRESCALER_1  0
#define HARDWARE_I2C_PRESCALER_4  1
#define HARDWARE_I2C_PRESCALER_16 2
#define HARDWARE_I2C_PRESCALER_64 3
//! @}

//! @name I2C READ and WRITE BITS
//! @{
//! Eighth bit (LSB) of I2C address indicates a "read" or "write".
//! (The first seven bits are the 7-bit I2C address.)
#define I2C_READ_BIT    0x01
#define I2C_WRITE_BIT   0x00
//! @}

//! @name STATUS BITS
//! @{
#define STATUS_START               0x08
#define STATUS_REPEATED_START      0x10
#define STATUS_ADDRESS_WRITE_ACK   0x18
#define STATUS_ADDRESS_WRITE_NACK  0x20
#define STATUS_WRITE_ACK           0x28
#define STATUS_WRITE_NACK          0x30
#define STATUS_ARBITRATION_LOST    0x38
#define STATUS_ADDRESS_READ_ACK    0x40
#define STATUS_ADDRESS_READ_NACK   0x48
#define STATUS_READ_ACK            0x50
#define STATUS_READ_NACK           0x58
//! @}

//! @name TIMEOUT AND DELAY IN US
//! @{
#define HW_I2C_DELAY  1
#define HW_I2C_TIMEOUT  20000
//! @}

//! @name ACK OR NACK PARAMETER PASSED TO I2C_READ
//! @{
#define WITH_ACK  0  //!<  Use with i2c_read(WITH_ACK) to read with an acknowledge
#define WITH_NACK 1  //!<  Use with i2c_read(WITH_NACK) to read without an acknowledge.  Normally used after the last byte of a multi-byte read.
//! @}

//! @name OPTIONAL I2C Address MACRO
//! @{
#define I2C_8ADDR(address) (address >> 1)  //!< Use to convert an 8-bit I2C address to 7 bits.
//! @}


//************************************************************************
// functions
//************************************************************************
int8_t i2c_read_byte
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t*    value       //!< byte to be read
    );


int8_t i2c_write_byte
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     value       //!< byte to be written
    );


int8_t i2c_read_byte_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     command,    //!< register address
    uint8_t*    value       //!< byte to be read
    );


int8_t i2c_write_byte_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     command,    //!< register address
    uint8_t     value       //!< byte to be written
    );


int8_t i2c_read_word_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     command,    //!< register address
    uint16_t*   value       //!< word to be read
    );


int8_t i2c_write_word_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     command,    //!< register address
    uint16_t    value       //!< word to be written
    );


int8_t i2c_read_block_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     command,    //!< register address
    uint8_t     length,     //!< length of array
    uint8_t*    values      //!< byte array to be read
    );


int8_t i2c_read_block_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     length,     //!< length of array
    uint8_t*    values      //!< byte array to be read
    );


int8_t i2c_write_block_data
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint8_t     command,    //!< register address
    uint8_t     length,     //!< length of array
    uint8_t*    values      //!< byte array to be written
    );


int8_t i2c_two_byte_command_read_block
    (
    uint8_t     address,    //!< 7-bit I2C address
    uint16_t    command,    //!< command word
    uint8_t     length,     //!< length of array
    uint8_t*    values      //!< byte array to be read
    );


int8_t i2c_write
    (
    uint8_t     data        //!< byte to be written
    );


uint8_t i2c_read
    (
    int8_t      ack         //!< acknowledge flag
    );


int8_t i2c_poll
    (
    uint8_t     i2c_address //!< i2c_address is the address of the slave being polled.
    );


void quikeval_I2C_connect();

WireWrapper quikeval_I2C_get_wire();

void quikeval_I2C_init();


#endif  // LT_I2C_h
