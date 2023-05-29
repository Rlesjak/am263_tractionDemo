#include "device.h"
#include "Serial_Cmd_HAL.h"
#include "Serial_CLI.h"
#include "ucc5870.h"
#include "FOC_loop.h"
#include "IPC_RPC_Comm.h"



void trinv_main(void *args)
{
    /* Pokreni drivere konfigurirane SysConfig alatom */
    Drivers_open();
    Board_driversOpen();

    /* Inicijalizacija koda za vektorsko upravljanje */
    FOC_init();
    DebugP_log("foc init done \r\n");
    /* Kalibracija ADC-a */
    FOC_cal();
    DebugP_log("foc cal done \r\n");
    /**
     * Registracija interrupta za izvođenje
     * vektorskog upravljanja
    */
    FOC_run();
    DebugP_log("foc run done \r\n");

    /* Inicijalizacija međuprocesorske komunikacije */
    setup_IPC();

    while (TRUE){
        // Izvrsavanje komunikacije sa jezgrom za tcp komunikaciju
        processIPC();
    }


    Board_driversClose();
    Drivers_close();
}
