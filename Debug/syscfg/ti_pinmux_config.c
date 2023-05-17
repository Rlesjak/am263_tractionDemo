/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Auto generated file
 */
#include "ti_drivers_config.h"
#include <drivers/pinmux.h>

static Pinmux_PerCfg_t gPinMuxMainDomainCfg[] = {
            /* EPWM7 pin config */
    /* EPWM7_A -> EPWM7_A (F4) */
    {
        PIN_EPWM7_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM0 pin config */
    /* EPWM0_A -> EPWM0_A (B2) */
    {
        PIN_EPWM0_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM0_B -> EPWM0_B (B1) */
    {
        PIN_EPWM0_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM1 pin config */
    /* EPWM1_A -> EPWM1_A (D3) */
    {
        PIN_EPWM1_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM1_B -> EPWM1_B (D2) */
    {
        PIN_EPWM1_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
            /* EPWM2 pin config */
    /* EPWM2_A -> EPWM2_A (C2) */
    {
        PIN_EPWM2_A,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* EPWM2_B -> EPWM2_B (C1) */
    {
        PIN_EPWM2_B,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

            /* EQEP2 pin config */
    /* EQEP2_A -> I2C0_SDA (B13) */
    {
        PIN_I2C0_SDA,
        ( PIN_MODE(8) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC )
    },
    /* EQEP2_B -> I2C0_SCL (A13) */
    {
        PIN_I2C0_SCL,
        ( PIN_MODE(8) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC )
    },
    /* EQEP2_STROBE -> MCAN2_TX (B12) */
    {
        PIN_MCAN2_TX,
        ( PIN_MODE(8) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC )
    },
    /* EQEP2_INDEX -> MCAN2_RX (A12) */
    {
        PIN_MCAN2_RX,
        ( PIN_MODE(8) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC )
    },

            /* GPIO0 pin config */
    /* GPIO129 -> SDFM0_D3 (C14) */
    {
        PIN_SDFM0_D3,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO128 -> SDFM0_CLK3 (A15) */
    {
        PIN_SDFM0_CLK3,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO81 -> MMC_DAT2 (A3) */
    {
        PIN_MMC_DAT2,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_ASYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO77 -> MMC_CLK (B6) */
    {
        PIN_MMC_CLK,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_ASYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO79 -> MMC_DAT0 (B5) */
    {
        PIN_MMC_DAT0,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_ASYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO75 -> UART1_RXD (L3) */
    {
        PIN_UART1_RXD,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_ASYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO69 -> EPWM13_A (K4) */
    {
        PIN_EPWM13_A,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO73 -> EPWM15_A (P15) */
    {
        PIN_EPWM15_A,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO71 -> EPWM14_A (V17) */
    {
        PIN_EPWM14_A,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO125 -> SDFM0_D1 (D13) */
    {
        PIN_SDFM0_D1,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO107 -> PR0_PRU1_GPIO6 (E16) */
    {
        PIN_PR0_PRU1_GPIO6,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO115 -> PR0_PRU1_GPIO11 (B18) */
    {
        PIN_PR0_PRU1_GPIO11,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO111 -> PR0_PRU1_GPIO2 (E17) */
    {
        PIN_PR0_PRU1_GPIO2,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },
            /* GPIO0 pin config */
    /* GPIO83 -> MMC_SDWP (C6) */
    {
        PIN_MMC_SDWP,
        ( PIN_MODE(7) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW | PIN_QUAL_SYNC | PIN_GPIO_R5SS0_0 )
    },

            /* SPI1 pin config */
    /* SPI1_CLK -> SPI1_CLK (A10) */
    {
        PIN_SPI1_CLK,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* SPI1_D0 -> SPI1_D0 (B10) */
    {
        PIN_SPI1_D0,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* SPI1_D1 -> SPI1_D1 (D9) */
    {
        PIN_SPI1_D1,
        ( PIN_MODE(0) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

            /* SPI1_CS0 pin config */
    /* SPI1_CS0 -> SPI1_CS0 (C9) */
    {
        PIN_SPI1_CS0,
        ( PIN_MODE(0) | PIN_PULL_DISABLE )
    },

            /* UART1 pin config */
    /* UART1_RXD -> LIN1_RXD (A9) */
    {
        PIN_LIN1_RXD,
        ( PIN_MODE(1) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },
    /* UART1_TXD -> LIN1_TXD (B9) */
    {
        PIN_LIN1_TXD,
        ( PIN_MODE(1) | PIN_PULL_DISABLE | PIN_SLEW_RATE_LOW )
    },

    {PINMUX_END, PINMUX_END}
};


/*
 * Pinmux
 */
void Pinmux_init(void)
{
    Pinmux_config(gPinMuxMainDomainCfg, PINMUX_DOMAIN_ID_MAIN);
}

