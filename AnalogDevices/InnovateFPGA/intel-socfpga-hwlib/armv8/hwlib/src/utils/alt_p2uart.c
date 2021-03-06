/******************************************************************************
*
* Copyright 2013-2017 Altera Corporation. All Rights Reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#include <stdint.h>
#include "alt_16550_uart.h"
#include "alt_printf.h"
#include "socal/hps.h"
#include "socal/socal.h"
#include "socal/alt_uart.h"
#include "alt_p2uart.h"

typedef struct UART_INFO_s
{
    void (*putc_function)(char pchar, FILE * info);
    int (*getc_function)(FILE * info);
    int    init_done;
    ALT_16550_DEVICE_t uart_ID;
    ALT_16550_HANDLE_t mUart;
} UART_INFO_t;

ALT_STATUS_CODE alt_init_uart_for_printf(UART_INFO_t *uartInfo);

UART_INFO_t term0_info = { uart_putc, uart_getc, 0, ALT_16550_DEVICE_SOCFPGA_UART0, {0, 0, 0, 0, 0} };
FILE * term0 = (FILE *) &term0_info;
UART_INFO_t term1_info = { uart_putc, uart_getc, 0, ALT_16550_DEVICE_SOCFPGA_UART1, {0, 0, 0, 0, 0} };
FILE * term1 = (FILE *) &term1_info;


int uart_getc(FILE * info)
{
    ALT_STATUS_CODE status;
    char data = 0;
    UART_INFO_t *port_info = (UART_INFO_t *) info;
    
    if (!port_info->init_done)
    {
        alt_init_uart_for_printf(port_info);
    }

    do
    {
        if(data == '\r')
            data = '\n';
        if(data != 0)
            return (int)data;
        status = alt_16550_fifo_read(&port_info->mUart, &data, 1);
    }while(status == ALT_E_SUCCESS);
    return 0;
}

void uart_putc(char pchar, FILE *info)
{
    UART_INFO_t *port_info = (UART_INFO_t *) info;
    
    if (!port_info->init_done)
    {
        alt_init_uart_for_printf(port_info);
    }

    if (pchar=='\n')
    { 
        uart_putc('\r', info);
    }
    alt_16550_fifo_write_safe(&port_info->mUart, &pchar, 1, 1); 
}

ALT_STATUS_CODE alt_init_uart_for_printf(UART_INFO_t *uartInfo)
{
    ALT_STATUS_CODE status;

    status = alt_16550_init( uartInfo->uart_ID, 0, 0, 
                             &uartInfo->mUart);

    if (status == ALT_E_SUCCESS)
    {   
        status = alt_16550_baudrate_set(&uartInfo->mUart, BAUD_RATE); 
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_16550_line_config_set(&uartInfo->mUart,  
                                             ALT_16550_DATABITS_8, 
                                             ALT_16550_PARITY_DISABLE, 
                                             ALT_16550_STOPBITS_1); 
    }
    
    if (status == ALT_E_SUCCESS)
    {
        status = alt_16550_fifo_enable(&uartInfo->mUart);
    }

    if (status == ALT_E_SUCCESS)
    {
        status = alt_16550_enable(&uartInfo->mUart);
    }
    uartInfo->init_done = 1;
    return status;
}

void alt_log_done(FILE *op)
{
    UART_INFO_t *p_uart = (UART_INFO_t *) op;
    uint32_t line_status;

    ALT_PRINTF("\n");

    /* Ensure that the TX FIFO is empty. */

    do
    {
        alt_16550_line_status_get(&p_uart->mUart, &line_status);
    } while ((line_status & ALT_16550_LINE_STATUS_TEMT) == 0);

    alt_clrbits_word(ALT_UART_IER_ADDR(p_uart->mUart.location), ALT_UART_IER_PTIME_SET_MSK);

    /* Leave the UARTs enabled.
     * Many subsequent stages expect the UART to be already setup by preloader. */
}

