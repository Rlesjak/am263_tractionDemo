/*
 * Copyright (C) 2021-2022 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "device.h"

#include "Serial_Cmd_HAL.h"
#include "Serial_CLI.h"
#include "ucc5870.h"

#include "FOC_loop.h"

/* Global variables and objects */
Bool gFlag = TRUE;
uint16_t gLoopTicker = 0;

void trinv_init(void)
{
    //
    // Select resolver sin/cos swap mux
    //
    uint32_t    gpioBaseAddr, pinNum;
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_RES_INH_BASE_ADDR);
    pinNum       = CONFIG_GPIO_RES_INH_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, CONFIG_GPIO_RES_INH_DIR);
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);

    //
    // Init UCC5870
    //
    //Init_UCC5870_Regs();
    // Init_UCC5870();

}


void trinv_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    trinv_init();
    DebugP_log("trinv init done \r\n");
    FOC_init();
    DebugP_log("foc init done \r\n");
    FOC_cal();
    DebugP_log("foc cal done \r\n");
    FOC_run();
    DebugP_log("foc run done \r\n");

    SerialCmd_init();
    serial_cli_init();

    while (gFlag){

        SerialCmd_read();

        // Izvrsavanje Serial_CLI interpretera
        serial_cli_service();

        gLoopTicker+=1;
        if (gLoopTicker>=100)
        {
            gLoopTicker=0;
        }
    }
    Board_driversClose();
    Drivers_close();
}
