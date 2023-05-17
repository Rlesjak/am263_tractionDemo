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

/*
 * EDMA
 */
/* EDMA atrributes */
static EDMA_Attrs gEdmaAttrs[CONFIG_EDMA_NUM_INSTANCES] =
{
    {

        .baseAddr           = CSL_TPCC0_U_BASE,
        .compIntrNumber     = CSLR_R5FSS0_CORE0_INTR_TPCC0_INT_0,
        .intrAggEnableAddr  = CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_TPCC0_INTAGG_MASK,
        .intrAggEnableMask  = 0x1FF & (~(2U << 0)),
        .intrAggStatusAddr  = CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_TPCC0_INTAGG_STATUS,
        .intrAggClearMask   = (2U << 0),
        .initPrms           =
        {
            .regionId     = 0,
            .queNum       = 0,
            .initParamSet = FALSE,
            .ownResource    =
            {
                .qdmaCh      = 0x03U,
                .dmaCh[0]    = 0xFFFFFFFFU,
                .dmaCh[1]    = 0x000000FFU,
                .tcc[0]      = 0xFFFFFFFFU,
                .tcc[1]      = 0x000000FFU,
                .paramSet[0] = 0xFFFFFFFFU,
                .paramSet[1] = 0xFFFFFFFFU,
                .paramSet[2] = 0xFFFFFFFFU,
                .paramSet[3] = 0xFFFFFFFFU,
                .paramSet[4] = 0xFFFFFFFFU,
                .paramSet[5] = 0xFFFFFFFFU,
                .paramSet[6] = 0xFFFFFFFFU,
                .paramSet[7] = 0x000007FFU,
            },
            .reservedDmaCh[0]    = 0x00000000U,
            .reservedDmaCh[1]    = 0x00000000U,
        },
    },
};

/* EDMA objects - initialized by the driver */
static EDMA_Object gEdmaObjects[CONFIG_EDMA_NUM_INSTANCES];
/* EDMA driver configuration */
EDMA_Config gEdmaConfig[CONFIG_EDMA_NUM_INSTANCES] =
{
    {
        &gEdmaAttrs[CONFIG_EDMA0],
        &gEdmaObjects[CONFIG_EDMA0],
    },
};

uint32_t gEdmaConfigNum = CONFIG_EDMA_NUM_INSTANCES;

/*
 * GPIO
 */

/* ----------- GPIO Direction, Trigger, Interrupt initialization ----------- */

void GPIO_init()
{
    uint32_t    baseAddr;

    /* Instance 0 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_RES_INH_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_RES_INH_PIN, CONFIG_GPIO_RES_INH_DIR);

    /* Instance 1 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_RES_OTSD_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_RES_OTSD_PIN, CONFIG_GPIO_RES_OTSD_DIR);

    /* Instance 2 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_NFLT1_H_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_NFLT1_H_PIN, CONFIG_GPIO_NFLT1_H_DIR);

    /* Instance 3 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_NFLT1_L_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_NFLT1_L_PIN, CONFIG_GPIO_NFLT1_L_DIR);

    /* Instance 4 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_NFLT2_H_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_NFLT2_H_PIN, CONFIG_GPIO_NFLT2_H_DIR);

    /* Instance 5 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_NFLT2_L_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_NFLT2_L_PIN, CONFIG_GPIO_NFLT2_L_DIR);

    /* Instance 6 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_ASC_EN_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_ASC_EN_PIN, CONFIG_GPIO_ASC_EN_DIR);

    /* Instance 7 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_ASC_H_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_ASC_H_PIN, CONFIG_GPIO_ASC_H_DIR);

    /* Instance 8 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_ASC_L_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_ASC_L_PIN, CONFIG_GPIO_ASC_L_DIR);

    /* Instance 9 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_VINVC_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_VINVC_PIN, CONFIG_GPIO_VINVC_DIR);

    /* Instance 10 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_OVP_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_OVP_PIN, CONFIG_GPIO_OVP_DIR);

    /* Instance 11 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_OCP_LATCH_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_OCP_LATCH_PIN, CONFIG_GPIO_OCP_LATCH_DIR);

    /* Instance 12 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_OTP_LATCH_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_OTP_LATCH_PIN, CONFIG_GPIO_OTP_LATCH_DIR);

    /* Instance 13 */
    /* Get address after translation translate */
    baseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_TEST_TOGGLE_BASE_ADDR);
    GPIO_setDirMode(baseAddr, CONFIG_GPIO_TEST_TOGGLE_PIN, CONFIG_GPIO_TEST_TOGGLE_DIR);
}


/* ----------- GPIO Interrupt de-initialization ----------- */
void GPIO_deinit()
{

}

/*
 * IPC Notify
 */
#include <drivers/ipc_notify.h>
#include <drivers/ipc_notify/v1/ipc_notify_v1.h>

/* this function is called within IpcNotify_init, this function returns core specific IPC config */
void IpcNotify_getConfig(IpcNotify_InterruptConfig **interruptConfig, uint32_t *interruptConfigNum)
{
    /* extern globals that are specific to this core */
    extern IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss0_0[];
    extern uint32_t gIpcNotifyInterruptConfigNum_r5fss0_0;

    *interruptConfig = &gIpcNotifyInterruptConfig_r5fss0_0[0];
    *interruptConfigNum = gIpcNotifyInterruptConfigNum_r5fss0_0;
}

/*
 * IPC RP Message
 */
#include <drivers/ipc_rpmsg.h>

/* Number of CPUs that are enabled for IPC RPMessage */
#define IPC_RPMESSAGE_NUM_CORES           (2U)
/* Number of VRINGs for the numner of CPUs that are enabled for IPC */
#define IPC_RPMESSAGE_NUM_VRINGS          (IPC_RPMESSAGE_NUM_CORES*(IPC_RPMESSAGE_NUM_CORES-1))
/* Number of a buffers in a VRING, i.e depth of VRING queue */
#define IPC_RPMESSAGE_NUM_VRING_BUF       (1U)
/* Max size of a buffer in a VRING */
#define IPC_RPMESSAGE_MAX_VRING_BUF_SIZE  (1024U)
/* Size of each VRING is
 *     number of buffers x ( size of each buffer + space for data structures of one buffer (32B) )
 */
#define IPC_RPMESSAGE_VRING_SIZE          RPMESSAGE_VRING_SIZE(IPC_RPMESSAGE_NUM_VRING_BUF, IPC_RPMESSAGE_MAX_VRING_BUF_SIZE)

/* VRING base address, all VRINGs are put one after other in the below region.
 *
 * IMPORTANT: Make sure of below,
 * - The section defined below should be placed at the exact same location in memory for all the CPUs
 * - The memory should be marked as non-cached for all the CPUs
 * - The section should be marked as NOLOAD in all the CPUs linker command file
 */
/* In this case gRPMessageVringMem size is 2176 bytes */
uint8_t gRPMessageVringMem[IPC_RPMESSAGE_NUM_VRINGS][IPC_RPMESSAGE_VRING_SIZE] __attribute__((aligned(128), section(".bss.ipc_vring_mem")));



/*
 * MCSPI
 */

/* MCSPI atrributes */
static MCSPI_Attrs gMcspiAttrs[CONFIG_MCSPI_NUM_INSTANCES] =
{
    {
        .baseAddr           = CSL_MCSPI1_U_BASE,
        .inputClkFreq       = 50000000U,
        .intrNum            = CSLR_R5FSS0_CORE0_INTR_MCSPI1_INTR,
        .operMode           = MCSPI_OPER_MODE_POLLED,
        .intrPriority       = 4U,
        .chMode             = MCSPI_CH_MODE_SINGLE,
        .pinMode            = MCSPI_PINMODE_4PIN,
        .initDelay          = MCSPI_INITDLY_0,
    },
};
/* MCSPI objects - initialized by the driver */
static MCSPI_Object gMcspiObjects[CONFIG_MCSPI_NUM_INSTANCES];
/* MCSPI driver configuration */
MCSPI_Config gMcspiConfig[CONFIG_MCSPI_NUM_INSTANCES] =
{
    {
        &gMcspiAttrs[CONFIG_MCSPI_UCC],
        &gMcspiObjects[CONFIG_MCSPI_UCC],
    },
};

uint32_t gMcspiConfigNum = CONFIG_MCSPI_NUM_INSTANCES;

#include <drivers/mcspi/v0/dma/mcspi_dma.h>
MCSPI_DmaConfig gMcspiDmaConfig =
{
    .fxns        = NULL,
    .mcspiDmaArgs = (void *)NULL,
};


uint32_t gMcspiDmaConfigNum = CONFIG_MCSPI_NUM_DMA_INSTANCES;

/*
 * UART
 */

/* UART atrributes */
static UART_Attrs gUartAttrs[CONFIG_UART_NUM_INSTANCES] =
{
    {
        .baseAddr           = CSL_UART1_U_BASE,
        .inputClkFreq       = 48000000U,
    },
};
/* UART objects - initialized by the driver */
static UART_Object gUartObjects[CONFIG_UART_NUM_INSTANCES];
/* UART driver configuration */
UART_Config gUartConfig[CONFIG_UART_NUM_INSTANCES] =
{
    {
        &gUartAttrs[CONFIG_UART_CONSOLE],
        &gUartObjects[CONFIG_UART_CONSOLE],
    },
};

uint32_t gUartConfigNum = CONFIG_UART_NUM_INSTANCES;

#include <drivers/uart/v0/dma/uart_dma.h>


UART_DmaConfig gUartDmaConfig[CONFIG_UART_NUM_DMA_INSTANCES] =
{
};

uint32_t gUartDmaConfigNum = CONFIG_UART_NUM_DMA_INSTANCES;


void Drivers_uartInit(void)
{
    UART_init();
}


void Pinmux_init(void);
void PowerClock_init(void);
void PowerClock_deinit(void);

/*
 * Common Functions
 */
void System_init(void)
{
    /* DPL init sets up address transalation unit, on some CPUs this is needed
     * to access SCICLIENT services, hence this needs to happen first
     */
    Dpl_init();

    
    /* initialize PMU */
    CycleCounterP_init(SOC_getSelfCpuClk());


    PowerClock_init();
    /* Now we can do pinmux */
    Pinmux_init();
    /* finally we initialize all peripheral drivers */
    /* ADC */
    {
        /* Enable ADC Reference */
        SOC_enableAdcReference(4);
        SOC_enableAdcReference(1);
        SOC_enableAdcReference(2);
        SOC_enableAdcReference(3);
        SOC_enableAdcReference(0);
    }

    EDMA_init();
    /* EPWM */
    {
        /* Enable time base clock for the selected ePWM */
        SOC_setEpwmTbClk(7, TRUE);
        SOC_setEpwmGroup(7, 0);
        SOC_setEpwmTbClk(0, TRUE);
        SOC_setEpwmGroup(0, 0);
        SOC_setEpwmTbClk(1, TRUE);
        SOC_setEpwmGroup(1, 0);
        SOC_setEpwmTbClk(2, TRUE);
        SOC_setEpwmGroup(2, 0);
    }
    GPIO_init();
    /* IPC Notify */
    {
        IpcNotify_Params notifyParams;
        int32_t status;

        /* initialize parameters to default */
        IpcNotify_Params_init(&notifyParams);

        /* specify the core on which this API is called */
        notifyParams.selfCoreId = CSL_CORE_ID_R5FSS0_0;

        /* list the cores that will do IPC Notify with this core
        * Make sure to NOT list 'self' core in the list below
        */
        notifyParams.numCores = 1;
        notifyParams.coreIdList[0] = CSL_CORE_ID_R5FSS0_1;

        /* initialize the IPC Notify module */
        status = IpcNotify_init(&notifyParams);
        DebugP_assert(status==SystemP_SUCCESS);

    }
    /* IPC RPMessage */
    {
        RPMessage_Params rpmsgParams;
        int32_t status;

        /* initialize parameters to default */
        RPMessage_Params_init(&rpmsgParams);

        /* VRING mapping from source core to destination core, '-1' means NO VRING,
            r5fss0_0 => {"r5fss0_0":-1,"r5fss0_1":0}
            r5fss0_1 => {"r5fss0_0":1,"r5fss0_1":-1}
         */
        /* TX VRINGs */
        rpmsgParams.vringTxBaseAddr[CSL_CORE_ID_R5FSS0_1] = (uintptr_t)gRPMessageVringMem[0];
        /* RX VRINGs */
        rpmsgParams.vringRxBaseAddr[CSL_CORE_ID_R5FSS0_1] = (uintptr_t)gRPMessageVringMem[1];
        /* Other VRING properties */
        rpmsgParams.vringSize = IPC_RPMESSAGE_VRING_SIZE;
        rpmsgParams.vringNumBuf = IPC_RPMESSAGE_NUM_VRING_BUF;
        rpmsgParams.vringMsgSize = IPC_RPMESSAGE_MAX_VRING_BUF_SIZE;

        /* initialize the IPC RP Message module */
        status = RPMessage_init(&rpmsgParams);
        DebugP_assert(status==SystemP_SUCCESS);
    }

    MCSPI_init();
    Drivers_uartInit();
}

void System_deinit(void)
{
    EDMA_deinit();
    /* EPWM */
    {
        /* Disable time base clock for the selected ePWM */
        SOC_setEpwmTbClk(7, FALSE);
	
        SOC_setEpwmTbClk(0, FALSE);
	
        SOC_setEpwmTbClk(1, FALSE);
	
        SOC_setEpwmTbClk(2, FALSE);
	
    }
    GPIO_deinit();
    RPMessage_deInit();
    IpcNotify_deInit();

    MCSPI_deinit();
    UART_deinit();
    PowerClock_deinit();
    Dpl_deinit();
}
