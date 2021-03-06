/******************************************************************************
*
* Copyright 2013 Altera Corporation. All Rights Reserved.
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

/*
 * $Id: //acds/rel/20.1/embedded/ip/hps/altera_hps/hwlib/include/alt_mpu_registers.h#1 $
 */

#ifndef __ALT_MPUSCU_H__
#define __ALT_MPUSCU_H__


#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */


/************************************************************************************************************/
/*                                alt_mpuscu.h                                                                 */
/*                                                                                                            */
/*  Definitions for the ARM Snoop Control Unit, which contains the Snoop Control Unit, the Watchdog         */
/*  Timer, the Private Timer, the Global Timer, the Interrupt Controller, and the Interrupt Distributor.    */
/*                                                                                                            */
/************************************************************************************************************/

#ifndef ALT_HPS_ADDR
#define ALT_HPS_ADDR 0x00
#endif


/*     ALT_MPUSCU_OFST is defined as a offset from ALT_HPS_ADDR in the SoCAL file hps.h            */
/*    and is the address of the base of the Snoop Control Unit (SCU)                                */
#define ALT_GLOBALTMR_BASE                  (ALT_MPUSCU_OFST + ALT_GLOBALTMR_MODULE_BASE_OFFSET)
#define ALT_CPU_WDTGPT_TMR_BASE             (ALT_MPUSCU_OFST + ALT_WDOG_TIMER_MODULE_BASE_OFFSET)
#define ALT_CPU_PRIVATE_TMR_BASE            (ALT_MPUSCU_OFST + ALT_CPU_PRIV_TIMER_MODULE_BASE_OFFSET)
#define ALT_CPU_INT_CTRL_BASE               (ALT_MPUSCU_OFST + ALT_INT_CONTROLLER_MODULE_BASE_OFFSET)
#define ALT_CPU_INT_DIST_BASE               (ALT_MPUSCU_OFST + ALT_INT_DISTRIBUTOR_MODULE_BASE_OFFSET)


            /* offsets */
        /* Global Timer offsets */
#define ALT_GLOBALTMR_MODULE_BASE_OFFSET        0x00000200
#define ALT_GLOBALTMR_CNTR_LO_REG_OFFSET        0x00000000
#define ALT_GLOBALTMR_CNTR_HI_REG_OFFSET        0x00000004
#define ALT_GLOBALTMR_CTRL_REG_OFFSET           0x00000008
#define ALT_GLOBALTMR_INT_STAT_REG_OFFSET       0x0000000C
#define ALT_GLOBALTMR_COMP_LO_REG_OFFSET        0x00000010
#define ALT_GLOBALTMR_COMP_HI_REG_OFFSET        0x00000014
#define ALT_GLOBALTMR_AUTOINC_REG_OFFSET        0x00000018

    /* Global Timer bitmasks */
#define ALT_GLOBALTMR_ENABLE_BIT                0x00000001
#define ALT_GLOBALTMR_COMP_ENABLE_BIT           0x00000002
#define ALT_GLOBALTMR_INT_ENABLE_BIT            0x00000004
#define ALT_GLOBALTMR_AUTOINC_ENABLE_BIT        0x00000008
#define ALT_GLOBALTMR_PS_MASK                   0x0000FF00
#define ALT_GLOBALTMR_PS_SHIFT                  8
#define ALT_GLOBALTMR_INT_STATUS_BIT            0x00000001

    /* Global timer constants */
#define ALT_GLOBALTMR_MAX                       0xFFFFFFFF
#define ALT_GLOBALTMR_PS_MAX                    0x000000FF


    /* Private timer offsets */
#define ALT_CPU_PRIV_TIMER_MODULE_BASE_OFFSET   0x00000600
#define ALT_CPU_PRIV_TMR_LOAD_REG_OFFSET        0x00000000
#define ALT_CPU_PRIV_TMR_CNTR_REG_OFFSET        0x00000004
#define ALT_CPU_PRIV_TMR_CTRL_REG_OFFSET        0x00000008
#define ALT_CPU_PRIV_TMR_INT_STATUS_REG_OFFSET  0x0000000C

    /* Private timer bitmasks */
#define ALT_CPU_PRIV_TMR_ENABLE                 0x00000001
#define ALT_CPU_PRIV_TMR_AUTO_RELOAD            0x00000002
#define ALT_CPU_PRIV_TMR_INT_EN                 0x00000004
#define ALT_CPU_PRIV_TMR_PS_MASK                0x0000FF00
#define ALT_CPU_PRIV_TMR_PS_SHIFT               8
#define ALT_CPU_PRIV_TMR_INT_STATUS             0x00000001

    /* Private timer constants */
#define ALT_CPU_PRIV_TMR_MAX                    0xFFFFFFFF
#define ALT_CPU_PRIV_TMR_PS_MAX                 0x000000FF


    /* Watchdog timer offsets */
#define ALT_WDOG_TIMER_MODULE_BASE_OFFSET       0x00000620
#define ALT_WDOG_LOAD_REG_OFFSET                0x00000000
#define ALT_WDOG_CNTR_REG_OFFSET                0x00000004
#define ALT_WDOG_CTRL_REG_OFFSET                0x00000008
#define ALT_WDOG_INTSTAT_REG_OFFSET             0x0000000C
#define ALT_WDOG_RSTSTAT_REG_OFFSET             0x00000010
#define ALT_WDOG_DISABLE_REG_OFFSET             0x00000014

    /* Watchdog timer bitmasks : */
    /* Control Register bitmasks */
#define ALT_WDOG_TMR_ENABLE                     0x00000001
#define ALT_WDOG_AUTO_RELOAD                    0x00000002
#define ALT_WDOG_INT_EN                         0x00000004
#define ALT_WDOG_WDT_MODE                       0x00000008
#define ALT_WDOG_PS_MASK                        0x0000FF00
#define ALT_WDOG_PS_SHIFT                       8
    /* Interrupt Status Register bitmasks */
#define ALT_WDOG_INT_STAT_BIT                   0x00000001
    /* Reset Status Register bitmasks */
#define ALT_WDOG_RST_STAT_BIT                   0x00000001

    /* Watchdog timer constants */
#define ALT_WDOG_TMR_MAX                        UINT32_MAX
#define ALT_WDOG_PS_MAX                         UINT8_MAX
#define ALT_WDOG_DISABLE_VAL0                   0x12345678
#define ALT_WDOG_DISABLE_VAL1                   0x87654321



        /* Interrupt Manager offsets */
    /*   <Add definitions here> */
#define ALT_INT_CONTROLLER_MODULE_BASE_OFFSET   0x00000100
#define ALT_INT_DISTRIBUTOR_MODULE_BASE_OFFSET  0x00001000
#define ALT_INT_DIST_TYPE_REG                   0x00000004


    /*  Upper bound of the MPUSCU address space  */
#define ALT_MPUSCU_MAX                          0x00001FFF



#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __ALT_MPUSCU_H__ */
