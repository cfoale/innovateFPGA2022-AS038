/*
LT_SPI.h

This file contains the SPI wrapper definitions for Linduino.


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

#ifndef LT_SPI_h
#define LT_SPI_h

#include <stdint.h>
#include <SpiWrapper.h>


//************************************************************************
// data types and constants
//************************************************************************
typedef enum : uint8_t
{
    SPI_DIRECTION_TO_SHIFT_LEFT,
    SPI_DIRECTION_TO_SHIFT_RIGHT
}SpiDirectionToSfift;


//************************************************************************
// functions
//************************************************************************
void quikeval_SPI_apply_settings
    (
    SPISettings settings        //!< SPI settings to be applied
    );

void quikeval_SPI_connect();

bool quikeval_SPI_init();

void spi_enable
    (
    uint8_t spi_clock_divider   //!< SPI clock divider
    );

void spi_disable();

int8_t spi_read
    (
    int8_t data     //!< byte to be written
    );

void spi_transfer_block
    (
    uint8_t     cs_pin,             //!< Chip select pin
    uint8_t*    tx,                 //!< Byte array to be transmitted
    uint8_t*    rx,                 //!< Byte array to be received
    uint8_t     length,             //!< Length of array
    uint8_t     bits_per_word = 16, //!< Bits per word
    uint8_t     bits_to_shift = 8,  //!< Number of bits to shift
    uint8_t     shift_direction = 0 //!< Direction to shift (0=L,1=R)
    );

void spi_transfer_byte
    (
    uint8_t     cs_pin,     //!< Chip Select pin
    uint8_t     tx,         //!< transmitted byte
    uint8_t*    rx          //!< received byte
    );

void spi_transfer_word
    (
    uint8_t     cs_pin,     //!< Chip Select pin
    uint16_t    tx,         //!< transmitted word
    uint16_t*   rx          //!< received word
    );

void spi_write
    (
    int8_t      data        //!< byte to be written
    );

#endif  // LT_SPI_h
