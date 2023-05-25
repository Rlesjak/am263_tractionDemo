#include "device.h"

#include "Serial_Cmd_HAL.h"
#include "Serial_CLI.h"
#include "ucc5870.h"

#include "FOC_loop.h"
#include "IPC_RPC_Comm.h"


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

    setup_IPC();

    SerialCmd_init();
    serial_cli_init();


    while (gFlag){

        SerialCmd_read();

        // Izvrsavanje Serial_CLI interpretera
        serial_cli_service();

        // Izvrsavanje Komunikacije sa jezgrom za tcp komunikaciju
        processIPC();


        gLoopTicker+=1;
        if (gLoopTicker>=100)
        {
            gLoopTicker=0;
        }
    }
    Board_driversClose();
    Drivers_close();
}
