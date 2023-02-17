//#############################################################################
//
// FILE:    ucc5870.c
//
// TITLE:   ucc5870 interface modules
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#define  _UCC5870
#include "ucc5870.h"

//
// Local enumerations
//
enum {
    FAIL = 0,
    PASS = 1
};
//
// UCC5870_Vars array indexing definitions
//
enum{
    UH = 0,
    UL = 1,
    VH = 2,
    VL = 3,
    WH = 4,
    WL = 5,
};

UCC5870_Vars  gUCC5870_DEV[6];

//
// UCC5870 gate driver chip address definitions
//
enum{
    UH_ADDR=1U,
    UL_ADDR=2U,
    VH_ADDR=3U,
    VL_ADDR=4U,
    WH_ADDR=5U,
    WL_ADDR=6U,
    BROADCAST=15U
};

uint16_t gUCC5870_ADDR[7] =
    { UH_ADDR, UL_ADDR, VH_ADDR, VL_ADDR, WH_ADDR, WL_ADDR, BROADCAST};

//
// UCC5870 status fault mask definitions
//

#define  STATUS1_FAULT_MASK      ((1 * GD_TWN_FAULT_MASK       )  | \
                                  (1 * PWM_COMP_CHK_FAULT_MASK )   )

#define  STATUS2_FAULT_MASK      ((0 * OR_NFLT2_MASK        )     | \
                                  (1 * OR_NFLT1_MASK        )     | \
                                  (0 * TRIM_CRC_FAULT_MASK  )     | \
                                  (0 * CFG_CRC_FAULT_MASK   )     | \
                                  (0 * CLK_MON_FAULT_MASK   )     | \
                                  (0 * BIST_FAULT_MASK      )     | \
                                  (0 * INT_COMM_FAULT_MASK  )     | \
                                  (0 * INT_REG_FAULT_MASK   )     | \
                                  (0 * SPI_FAULT_MASK       )     | \
                                  (0 * STP_FAULT_MASK       )     | \
                                  (0 * OVLO1_FAULT_MASK     )     | \
                                  (0 * UVLO1_FAULT_MASK     )      )

#define  STATUS3_FAULT_MASK      ((1 * DESAT_FAULT_MASK        )  | \
                                  (1 * OC_FAULT_MASK           )  | \
                                  (1 * SC_FAULT_MASK           )  | \
                                  (1 * VREG2_ILIMIT_FAULT_MASK )  | \
                                  (1 * PS_TSD_FAULT_MASK       )  | \
                                  (1 * VCEOV_FAULT_MASK        )  | \
                                  (1 * UVLO2_FAULT_MASK        )  | \
                                  (1 * OVLO2_FAULT_MASK        )  | \
                                  (1 * UVLO3_FAULT_MASK        )  | \
                                  (1 * OVLO3_FAULT_MASK        )  | \
                                  (0 * MCLP_STATE_MASK         )  | \
                                  (1 * INT_COMM_SEC_FAULT_MASK )  | \
                                  (1 * INT_REG_SEC_FAULT_MASK  )  | \
                                  (1 * GM_FAULT_MASK           )   )

#define  STATUS4_FAULT_MASK      ((0 * RIM_CRC_SEC_FAULT_MASK  )  | \
                                  (0 * CFG_CRC_SEC_FAULT_MASK  )  | \
                                  (0 * CLK_MON_SEC_FAULT_MASK  )  | \
                                  (0 * BIST_SEC_FAULT_MASK     )  | \
                                  (0 * OR_NFLT2_SEC_MASK       )  | \
                                  (1 * OR_NFLT1_SEC_MASK       )  | \
                                  (0 * GD_TSD_SEC_FAULT_MASK   )  | \
                                  (0 * GD_TWN_SEC_FAULT_MASK   )   )

#define  STATUS5_FAULT_MASK       (1 * ADC_FAULT_MASK)

#define UCC5870_NFLT_NUM   4

uint32_t gUCC5870_nFLT_BASE[UCC5870_NFLT_NUM] = {
                          CONFIG_GPIO_NFLT1_H_BASE_ADDR,
                          CONFIG_GPIO_NFLT1_L_BASE_ADDR,
                          CONFIG_GPIO_NFLT2_H_BASE_ADDR,
                          CONFIG_GPIO_NFLT2_L_BASE_ADDR
};
uint32_t gUCC5870_nFLT_PIN[UCC5870_NFLT_NUM] = {
                         CONFIG_GPIO_NFLT1_H_PIN,
                         CONFIG_GPIO_NFLT1_L_PIN,
                         CONFIG_GPIO_NFLT2_H_PIN,
                         CONFIG_GPIO_NFLT2_L_PIN
};

uint32_t    gUCC5870_nFLT_Status[UCC5870_NFLT_NUM] = {0};

uint16_t  gUCC5870_statusRegAdrs[5] =
        {
          STATUS1, STATUS2, STATUS3, STATUS4, STATUS5
        },
        gUCC5870_statusFaultMask[5] =
        {
          STATUS1_FAULT_MASK, STATUS2_FAULT_MASK, STATUS3_FAULT_MASK,
          STATUS4_FAULT_MASK, STATUS5_FAULT_MASK
        };

TripFlagDMC_t tripFlagDMC;

// make sure it consistent with syscfg
uint32_t gSPI_UCC_base=0;
uint32_t gChEnableRegVal, gChDisableRegVal;
uint32_t gCsAssertRegVal, gCsDeAssertRegVal;
uint32_t gChStatReg_tx, gChStatReg_rx;

uint32_t gPWM_base[gPWM_NUM] = {CONFIG_EPWM0_BASE_ADDR,
                                CONFIG_EPWM1_BASE_ADDR,
                                CONFIG_EPWM2_BASE_ADDR
};

uint16_t    gUCC5870_InitStatus[6] = {0, 0, 0, 0, 0, 0},
            gUCC5870_DiagStatus[6] = {0, 0, 0, 0, 0, 0},
            gUCC5870_InitFault = 0,
            gUCC5870_StatusFault = 0,
            gUCC5870_PriReadyFault = 0,
            gUCC5870_SecReadyFault = 0;

//
// Function prototypes
//
// prototypes for external use
//
void             Init_UCC5870_Regs(void);
UCC5870_Status_e Init_UCC5870(void);
uint16_t         diagnose_UCC5870(uint16_t i);
UCC5870_Status_e inverterDiagnostics(void);
void             clearFaultsUCC5870(void);

//
// local prototypes for use within this file
//
uint16_t    UCC5870_SPItransfer(uint16_t ctrlWord);
void        cmdUCC5870 (uint16_t cmd);
uint16_t    readRegUCC5870 (uint16_t CA, uint16_t RA);
void        writeRegUCC5870(uint16_t CA, uint16_t RA, uint16_t  data);
uint16_t    writeVerifyRegUCC5870 (uint16_t CA, uint16_t RA, uint16_t  data);
uint16_t    writeVerify_UCC5870(uint16_t i);


//
// Local function macros
//
#define  UCC5870_DRV_EN(CA)      cmdUCC5870((CA << 12) | 0x009);
#define  UCC5870_DRV_DIS(CA)     cmdUCC5870((CA << 12) | 0x00a);
#define  UCC5870_CFG_IN(CA)      cmdUCC5870((CA << 12) | 0x222);
#define  UCC5870_NOP(CA)         cmdUCC5870((CA << 12) | 0x542);
#define  UCC5870_SWRESET(CA)     cmdUCC5870((CA << 12) | 0x708);

#define  UCC5870_WR_CA(CA)       cmdUCC5870((BROADCAST << 12) | (0x0da0 + CA));
#define  UCC5870_WR_REG(CA, RA)  cmdUCC5870((CA << 12) | (0x0c00 + RA));
#define  UCC5870_WR_DH(CA, DH)   cmdUCC5870((CA << 12) | (0x0a00 + DH));
#define  UCC5870_WR_DL(CA, DL)   cmdUCC5870((CA << 12) | (0x0b00 + DL));
#define  UCC5870_RD_REG(CA, RA)  cmdUCC5870((CA << 12) | (0x0100 + RA));

#define  CLEAR_FAULTS(CA)  writeRegUCC5870 (CA, CONTROL2, 0x8000)


//
// Function definitions
//

void cmdUCC5870(uint16_t cmd)
{
    uint32_t            spibaseAddr, spichNum;

    spibaseAddr = gSPI_UCC_base;
    spichNum = gConfigMcspiUccChCfg[CONFIG_MCSPI_UCC].chNum;

    /* SPIEN line is forced to low state.*/
    MCSPI_writeChConfReg(spibaseAddr, spichNum, gCsAssertRegVal);

    /* Wait fot Tx FIFO to be empty before writing the data. */
    gChStatReg_tx = MCSPI_readChStatusReg(spibaseAddr, spichNum);
    while (0 == (gChStatReg_tx & CSL_MCSPI_CH1STAT_TXFFE_MASK))
    {
        gChStatReg_tx = MCSPI_readChStatusReg(spibaseAddr, spichNum);
    }

    /* write the data. */
    MCSPI_writeTxDataReg(spibaseAddr, (uint16_t) cmd, spichNum);

    /* Wait for Tx FIFO to be empty for the last set of data. */
    gChStatReg_tx = MCSPI_readChStatusReg(spibaseAddr, spichNum);
    while (0 == (gChStatReg_tx & CSL_MCSPI_CH1STAT_TXFFE_MASK))
    {
        gChStatReg_tx = MCSPI_readChStatusReg(spibaseAddr, spichNum);
    }

    /* Tx FIFO Empty is triggered when last word from FIFO is written to
       internal shift register. SO wait for the end of transfer of last word.
       The EOT gets set after every word when the transfer from shift
       register is complete and is reset when the transmission starts.
       So FIFO empty check is required to make sure the data in FIFO is
       sent out then wait for EOT for the last word. */
    gChStatReg_tx = MCSPI_readChStatusReg(spibaseAddr, spichNum);
    while (0 == (gChStatReg_tx & CSL_MCSPI_CH1STAT_EOT_MASK))
    {
        gChStatReg_tx = MCSPI_readChStatusReg(spibaseAddr, spichNum);
    }

    /* Force SPIEN line to the inactive state.*/
    MCSPI_writeChConfReg(spibaseAddr, spichNum, gCsDeAssertRegVal);

    return;
 }

uint16_t readRegUCC5870(uint16_t chipAddress, uint16_t regAddress)
{
    uint16_t tmp;
    uint32_t    spibaseAddr, spichNum;
    spibaseAddr = gSPI_UCC_base;
    spichNum = gConfigMcspiUccChCfg[0U].chNum;

    /* Enable the McSPI channel for communication.*/
    MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChEnableRegVal);

    UCC5870_RD_REG(chipAddress, regAddress);
    UCC5870_NOP(chipAddress);

    /* Wait fot Rx FIFO to be ready before reading the data. */
    gChStatReg_rx = MCSPI_readChStatusReg(spibaseAddr, spichNum);
    while (0 == (gChStatReg_rx & CSL_MCSPI_CH1STAT_RXS_MASK))
    {
        gChStatReg_rx = MCSPI_readChStatusReg(spibaseAddr, spichNum);
    };

    /* Read the data. */
    tmp = (uint16_t) MCSPI_readRxDataReg(spibaseAddr, spichNum);

    /* Disable the McSPI channel.*/
    MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChDisableRegVal);

    return(tmp);
}

void writeRegUCC5870(uint16_t chipAddress,
                        uint16_t regAddress, uint16_t  data)
{
    uint32_t    spibaseAddr, spichNum;
    spibaseAddr = gSPI_UCC_base;
    spichNum = gConfigMcspiUccChCfg[0U].chNum;

    /* Enable the McSPI channel for communication.*/
    MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChEnableRegVal);

    UCC5870_WR_REG(chipAddress,  regAddress);
    UCC5870_WR_DH (chipAddress, (data / 256  ));
    UCC5870_WR_DL (chipAddress, (data % 256  ));

    /* Disable the McSPI channel.*/
    MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChDisableRegVal);

    return;
}

uint16_t writeVerifyRegUCC5870(uint16_t chipAddress,
                               uint16_t regAddress, uint16_t  data)
{
    uint16_t tmp=0;

    while(tmp != data)
    {
        writeRegUCC5870(chipAddress, regAddress, data);
        tmp = readRegUCC5870(chipAddress, regAddress);
    }

    return(( (tmp == data) ? PASS : FAIL ));
}

void clearFaultsUCC5870(void)
{
    uint32_t    spibaseAddr, spichNum;
    spibaseAddr = gSPI_UCC_base;
    spichNum = gConfigMcspiUccChCfg[0U].chNum;

    /* Enable the McSPI channel for communication.*/
    MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChEnableRegVal);

    for (uint16_t i = UL; i <= WL; i++)
    {
        CLEAR_FAULTS(gUCC5870_ADDR[i]);
    }

    /* Disable the McSPI channel.*/
    MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChDisableRegVal);

    return;
}

void Init_UCC5870_Regs(void)
{
    //
    // Initialize Ucc5870 vars to zero
    //
    initUcc5870Vars(&gUCC5870_DEV[UH]);

    //
    // CFG1 register settings
    //
    gUCC5870_DEV[UH].cfg1.bit.GD_TWN_DIS     = OT_warning_enable;
    gUCC5870_DEV[UH].cfg1.bit.IO_DEGLITCH    = io_deglitch_70ns;
    gUCC5870_DEV[UH].cfg1.bit.NFLT2_DOUT_MUX = nFLT2_DOUT_nFLT2;
    gUCC5870_DEV[UH].cfg1.bit.OV1_DIS        = vcc1_ovlo_enable;
    gUCC5870_DEV[UH].cfg1.bit.UV1_DIS        = vcc1_uvlo_enable;
    gUCC5870_DEV[UH].cfg1.bit.OVLO1_LEVEL    = vcc1_3V3;
    gUCC5870_DEV[UH].cfg1.bit.UVLO1_LEVEL    = vcc1_3V3;

    //
    // in ns
    //
    gUCC5870_DEV[UH].cfg1.bit.TDEAD          = Tdead(245);

    //
    // CFG2 register settings
    //

    //
    //  PWM check fault
    //
    gUCC5870_DEV[UH].cfg2.bit.PWM_CHK_FAULT      = report_fault_on_nFLT1;

    //
    //  Vreg Ilimit fault
    //
    gUCC5870_DEV[UH].cfg2.bit.VREG1_ILIMIT_FAULT = report_fault_on_nFLT1;

    //
    //  temp warning fault
    //
    gUCC5870_DEV[UH].cfg2.bit.GD_TWN_FAULT       = warning_on_nFLT1;

    //
    //  Analog BIST fault
    //
    gUCC5870_DEV[UH].cfg2.bit.BIST_FAULT         = report_fault_on_nFLT1;

    //
    //  CRC fault
    //
    gUCC5870_DEV[UH].cfg2.bit.TRIM_CRC_FAULT     = report_fault_on_nFLT1;

    //
    //  Internal regulator fault
    //
    gUCC5870_DEV[UH].cfg2.bit.INT_REG_FAULT      = report_fault_on_nFLT1;

    //
    //  reg CRC fault
    //
    gUCC5870_DEV[UH].cfg2.bit.CFG_CRC_FAULT      = report_fault_on_nFLT1;

    //
    //  SPI fault
    //
    gUCC5870_DEV[UH].cfg2.bit.SPI_FAULT          = report_SPI_fault_on_nFLT1;

    //
    //  Clock monitor fault
    //
    gUCC5870_DEV[UH].cfg2.bit.CLK_MON_FAULT      = report_fault_on_nFLT1;

    //
    //  STP fault
    //
    gUCC5870_DEV[UH].cfg2.bit.STP_FAULT          = report_fault_on_nFLT1;

    //
    //  UVLO fault
    //
    gUCC5870_DEV[UH].cfg2.bit.UVLO1_FAULT        = report_fault_on_nFLT1;

    //
    //  OVLO fault
    //
    gUCC5870_DEV[UH].cfg2.bit.OVLO1_FAULT        = report_fault_on_nFLT1;

    //
    //  Inter-die comm failure
    //
    gUCC5870_DEV[UH].cfg2.bit.INT_COMM_FAULT     = report_comm_fault_on_nFLT1;

    //
    // CFG3 register settings
    //
    gUCC5870_DEV[UH].cfg3.bit.AI_IZTC_SEL             = AI1_AI3_AI5_bias_currents_off;
    gUCC5870_DEV[UH].cfg3.bit.FS_STATE_CFG_CRC_FAULT  = OUTx_LOW;
    gUCC5870_DEV[UH].cfg3.bit.ITO2_EN                 = CURRENT_SRC_OUT_AI2_4_6_DISABLED;
    gUCC5870_DEV[UH].cfg3.bit.ITO1_EN                 = CURRENT_SRC_OUT_AI1_3_5_DISABLED;
    gUCC5870_DEV[UH].cfg3.bit.FS_STATE_INT_COMM_FAULT = OUTx_LOW;
    gUCC5870_DEV[UH].cfg3.bit.FS_STATE_INT_REG_FAULT  = OUTx_LOW;
    gUCC5870_DEV[UH].cfg3.bit.FS_STATE_SPI_FAULT      = FS_SPI_OUTx_LOW;
    gUCC5870_DEV[UH].cfg3.bit.FS_STATE_STP_FAULT      = FS_STP_OUTx_LOW;
    gUCC5870_DEV[UH].cfg3.bit.FS_STATE_PWM_CHK        = OUTx_LOW;
    gUCC5870_DEV[UH].cfg3.bit.FS_STATE_OVLO1_FAULT    = OUTx_LOW;
    gUCC5870_DEV[UH].cfg3.bit.FS_STATE_UVLO1_FAULT    = OUTx_LOW;

    //
    // CFG4 register settings
    //
    gUCC5870_DEV[UH].cfg4.bit.UVOV3_EN        = VEE2_uvlo_ovlo_enable;
    gUCC5870_DEV[UH].cfg4.bit.PS_TSD_EN       = Switch_thermal_SD_disable;
    gUCC5870_DEV[UH].cfg4.bit.OCP_DIS         = OCP_disable;
    gUCC5870_DEV[UH].cfg4.bit.SCP_DIS         = SCP_disable;
    gUCC5870_DEV[UH].cfg4.bit.DESAT_EN        = DESAT_det_enable;
    gUCC5870_DEV[UH].cfg4.bit.VCECLP_EN       = VCE_CLAMP_enable;
    gUCC5870_DEV[UH].cfg4.bit.MCLP_DIS        = MILLER_CLAMP_enable;
    gUCC5870_DEV[UH].cfg4.bit.GM_EN           = Gate_Volt_Mon_disable;
    gUCC5870_DEV[UH].cfg4.bit.GM_BLK          = Gate_volt_mon_blank_2500ns;
    gUCC5870_DEV[UH].cfg4.bit.MCLP_CFG        = MILLER_CLAMP_internal;
    gUCC5870_DEV[UH].cfg4.bit.OV2_DIS         = VCC2_ovlo_enable;
    gUCC5870_DEV[UH].cfg4.bit.DESAT_DEGLITCH  = DESAT_deglitch_316ns;
    gUCC5870_DEV[UH].cfg4.bit.PS_TSD_DEGLITCH = TSD_1000ns;
    gUCC5870_DEV[UH].cfg4.bit.UV2_DIS         = VCC2_uvlo_enable;

    //
    // CFG5 register settings
    //
    gUCC5870_DEV[UH].cfg5.bit.PWM_MUTE_EN     = PWM_MUTE_FOR_SC_OC_OT_FAULTS_DISABLED;
    gUCC5870_DEV[UH].cfg5.bit.TLTOFF_STO_EN   = STO_FOR_SC_DESAT_OC;
    gUCC5870_DEV[UH].cfg5.bit.STO_CURR        = SOFT_TURN_OFF_0_3A;
    gUCC5870_DEV[UH].cfg5.bit.MCLPTH          = MILLER_CLAMP_VTH_3V;
    gUCC5870_DEV[UH].cfg5.bit.DESAT_DCHG_EN   = DESAT_DISCHARGE_ENABLE;
    gUCC5870_DEV[UH].cfg5.bit.DESAT_CHG_CURR  = BLANK_CAP_CHRG_CUR_600uA;
    gUCC5870_DEV[UH].cfg5.bit.DESATTH         = DESAT_VTH_2p5;
    gUCC5870_DEV[UH].cfg5.bit.GM_STO2LTO_DIS  = GATE_MON_FLT_DURING_STO_TLTOFF_ENABLED;

    //
    // CFG6 register settings
    //
    gUCC5870_DEV[UH].cfg6.bit.PS_TSDTH  = TSD_TH_2p75;
    gUCC5870_DEV[UH].cfg6.bit.TEMP_CURR = TEMP_CURR_0p1A;
    gUCC5870_DEV[UH].cfg6.bit.OC_BLK    = OCD_BLANK_TIME_0p5us;
    gUCC5870_DEV[UH].cfg6.bit.SC_BLK    = SCD_BLANK_TIME_0p2us;
    gUCC5870_DEV[UH].cfg6.bit.SCTH      = SCD_VTH_1p25V;
    gUCC5870_DEV[UH].cfg6.bit.OCTH      = OCD_VTH_0p5V;

    //
    // CFG7 register settings
    //
    gUCC5870_DEV[UH].cfg7.bit.FS_STATE_ADC_FAULT = FS_ADC_FLT_OUT_IGNORE;
    gUCC5870_DEV[UH].cfg7.bit.ADC_FAULT_P        = dont_report_adcfault_on_nFLT1;
    gUCC5870_DEV[UH].cfg7.bit.ADC_SAMP_DLY       = ADC_SAMP_DELAY_1120ns;
    gUCC5870_DEV[UH].cfg7.bit.ADC_SAMP_MODE      = ADC_SAMP_CENTER_ALIGN;
    gUCC5870_DEV[UH].cfg7.bit.ADC_EN             = ADC_SAMP_DISABLE;
    gUCC5870_DEV[UH].cfg7.bit.OVLO3TH            = VEE2_OVLO_VTH_M10V;
    gUCC5870_DEV[UH].cfg7.bit.UVLO3TH            = VEE2_UVLO_VTH_M3V;
    gUCC5870_DEV[UH].cfg7.bit.OVLO2TH            = VCC2_OVLO_VTH_P23V;
    gUCC5870_DEV[UH].cfg7.bit.UVLO2TH            = VCC2_UVLO_VTH_P12V;

    //
    // CFG8 register settings
    //
    gUCC5870_DEV[UH].cfg8.bit.IOUT_SEL         = GATE_DRIVE_STRENGTH_3rd;
    gUCC5870_DEV[UH].cfg8.bit.AI_ASC_MUX       = AI_ASC_MUX_AI;
    gUCC5870_DEV[UH].cfg8.bit.VREF_SEL         = VREF_EXTERNAL;
    gUCC5870_DEV[UH].cfg8.bit.GD_2LOFF_STO_EN  = GD_2LOFF_STO_ENABLE;
    gUCC5870_DEV[UH].cfg8.bit.CRC_DIS          = CRC_CHECK_ENABLE;
    gUCC5870_DEV[UH].cfg8.bit.GD_2LOFF_CURR    = GD_2LOFF_DISCHARGE_CURR_0p9A;
    gUCC5870_DEV[UH].cfg8.bit.GD_2LOFF_TIME    = GD_2LOFF_PLATEAU_TIME_150ns;
    gUCC5870_DEV[UH].cfg8.bit.GD_2LOFF_VOLT    = GD_2LOFF_PLATEAU_VOLT_6V;

    //
    // CFG9 register settings
    //
    gUCC5870_DEV[UH].cfg9.bit.CLK_MON_SEC_FAULT  = report_fault_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.VREG2_ILIMIT_FAULT = report_fault_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.BIST_SEC_FAULT     = report_fault_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.INT_REG_SEC_FAULT  = report_fault_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.TRIM_CRC_SEC_FAULT = report_fault_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.CFG_CRC_SEC_FAULT  = report_fault_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.INT_COMM_SEC_FAULT = report_comm_fault_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.GD_TSD_FAULT       = report_fault_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.PS_TSD_FAULT       = dont_report_TSD_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.OVLO23_FAULT       = report_fault_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.UVLO23_FAULT       = report_fault_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.GM_FAULT           = GM_FAULT_ON_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.OC_FAULT           = dont_report_fault_on_nFLT1;
    gUCC5870_DEV[UH].cfg9.bit.SC_FAULT           = dont_report_fault_on_nFLT1;

    //
    // CFG10 register settings
    //
    gUCC5870_DEV[UH].cfg10.bit.FS_STATE_INT_COMM_SEC  = PULLED_LOW;
    gUCC5870_DEV[UH].cfg10.bit.FS_STATE_GM            = PULLED_LOW;
    gUCC5870_DEV[UH].cfg10.bit.FS_STATE_GD_TSD        = PULLED_LOW;
    gUCC5870_DEV[UH].cfg10.bit.FS_STATE_PS_TSD        = NO_ACTION;
    gUCC5870_DEV[UH].cfg10.bit.FS_STATE_OCP           = NO_ACTION;
    gUCC5870_DEV[UH].cfg10.bit.FS_STATE_INT_REG_FAULT = PULLED_LOW;
    gUCC5870_DEV[UH].cfg10.bit.FS_STATE_DESAT_SCP     = PULLED_LOW;
    gUCC5870_DEV[UH].cfg10.bit.GD_TWN_SEC_EN          = GD_TWN_SEC_ENABLED;

    //
    // CFG11 register settings
    //
    gUCC5870_DEV[UH].cfg11.bit.FS_STATE_CLK_MON_SEC_FAULT  = PULLED_LOW;
    gUCC5870_DEV[UH].cfg11.bit.VCE_CLMP_HLD_TIME           = VCE_CLAMP_HOLD_TIME_100ns;
    gUCC5870_DEV[UH].cfg11.bit.FS_STATE_CFG_CRC_SEC_FAULT  = PULLED_LOW;
    gUCC5870_DEV[UH].cfg11.bit.FS_STATE_TRIM_CRC_SEC_FAULT = PULLED_LOW;
    gUCC5870_DEV[UH].cfg11.bit.FS_STATE_OVLO3              = PULLED_LOW;
    gUCC5870_DEV[UH].cfg11.bit.FS_STATE_UVLO3              = PULLED_LOW;
    gUCC5870_DEV[UH].cfg11.bit.FS_STATE_OVLO2              = PULLED_LOW;
    gUCC5870_DEV[UH].cfg11.bit.FS_STATE_UVLO2              = PULLED_LOW;

    //
    // ADCCFG register settings
    //
    gUCC5870_DEV[UH].adccfg.bit.ADC_ON_CH_SEL_7  = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_ON_CH_SEL_6  = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_ON_CH_SEL_5  = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_ON_CH_SEL_4  = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_ON_CH_SEL_3  = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_ON_CH_SEL_2  = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_ON_CH_SEL_1  = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_OFF_CH_SEL_7 = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_OFF_CH_SEL_6 = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_OFF_CH_SEL_5 = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_OFF_CH_SEL_4 = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_OFF_CH_SEL_3 = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_OFF_CH_SEL_2 = DONT_SAMPLE_CH;
    gUCC5870_DEV[UH].adccfg.bit.ADC_OFF_CH_SEL_1 = DONT_SAMPLE_CH;

    //
    // DOUTCFG register settings
    //
    gUCC5870_DEV[UH].doutcfg.bit.DOUT_TO_AI1 = DONT_CONNECT_TO_DOUT;
    gUCC5870_DEV[UH].doutcfg.bit.DOUT_TO_AI3 = DONT_CONNECT_TO_DOUT;
    gUCC5870_DEV[UH].doutcfg.bit.DOUT_TO_AI5 = DONT_CONNECT_TO_DOUT;
    gUCC5870_DEV[UH].doutcfg.bit.DOUT_TO_AI2 = DONT_CONNECT_TO_DOUT;
    gUCC5870_DEV[UH].doutcfg.bit.DOUT_TO_AI4 = DONT_CONNECT_TO_DOUT;
    gUCC5870_DEV[UH].doutcfg.bit.DOUT_TO_AI6 = DONT_CONNECT_TO_DOUT;
    gUCC5870_DEV[UH].doutcfg.bit.DOUT_TO_TJ  = DONT_CONNECT_TO_DOUT;
    gUCC5870_DEV[UH].doutcfg.bit.FREQ_DOUT   = FREQ_DOUT_111p4KHz;
    gUCC5870_DEV[UH].doutcfg.bit.AI6OCSC_EN  = PROT_DISABLED;
    gUCC5870_DEV[UH].doutcfg.bit.AI4OCSC_EN  = PROT_DISABLED;
    gUCC5870_DEV[UH].doutcfg.bit.AI2OCSC_EN  = PROT_DISABLED;
    gUCC5870_DEV[UH].doutcfg.bit.AI5OT_EN    = PROT_DISABLED;
    gUCC5870_DEV[UH].doutcfg.bit.AI3OT_EN    = PROT_DISABLED;
    gUCC5870_DEV[UH].doutcfg.bit.AI1OT_EN    = PROT_DISABLED;

    //
    // copying the configuration settings for all devices
    //
    for (uint16_t i = UL; i <= WL; i++)
    {
        gUCC5870_DEV[i] = gUCC5870_DEV[UH];
    }

    return;
}

//
// Initialize UCC5870
//

UCC5870_Status_e Init_UCC5870(void)
{

    uint32_t    spibaseAddr, spichNum;
    uint32_t    chCtrlRegVal, chConfRegVal;


    UCC5870_Status_e   status;
    uint16_t    i;

    uint32_t    gpioBaseAddr, pinNum;
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_ASC_EN_BASE_ADDR);
    pinNum       = CONFIG_GPIO_ASC_EN_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, CONFIG_GPIO_ASC_EN_DIR);
    GPIO_pinWriteLow(gpioBaseAddr, pinNum);

    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_ASC_H_BASE_ADDR);
    pinNum       = CONFIG_GPIO_ASC_H_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, CONFIG_GPIO_ASC_H_DIR);
    GPIO_pinWriteLow(gpioBaseAddr, pinNum);

    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_ASC_L_BASE_ADDR);
    pinNum       = CONFIG_GPIO_ASC_L_PIN;
    GPIO_setDirMode(gpioBaseAddr, pinNum, CONFIG_GPIO_ASC_L_DIR);
    GPIO_pinWriteLow(gpioBaseAddr, pinNum);


    /* Get SPI Channel Info */
    gSPI_UCC_base = MCSPI_getBaseAddr(gMcspiHandle[CONFIG_MCSPI_UCC]);
    spibaseAddr = gSPI_UCC_base;
    /* Initialize SPI Channel Number */
    spichNum = gConfigMcspiUccChCfg[CONFIG_MCSPI_UCC].chNum;

    /* set data frame */
    MCSPI_setDataWidth(spibaseAddr, spichNum, 16);

    /* Enable the TX RX FIFO of McSPI peripheral. */
    MCSPI_enableTxFIFO(spibaseAddr, spichNum, MCSPI_TX_FIFO_ENABLE);
    MCSPI_enableRxFIFO(spibaseAddr, spichNum, MCSPI_RX_FIFO_ENABLE);

    /*
     * Channel Control and config registers are updated after Open/Reconfigure.
     * Channel enable/disable and CS assert/deassert require updation of bits in
     * control and config registers. Also these registers will not be updated
     * during data transfer. So reg read modify write operations can be updated
     * to write only operations.
     * Store ch enable/disable reg val and cs assert/deassert reg vals.
     */
    chCtrlRegVal     = MCSPI_readChCtrlReg(spibaseAddr, spichNum);
    gChEnableRegVal  = chCtrlRegVal | CSL_MCSPI_CH1CTRL_EN_MASK;
    gChDisableRegVal = chCtrlRegVal & (~CSL_MCSPI_CH1CTRL_EN_MASK);

    chConfRegVal      = MCSPI_readChConf(spibaseAddr, spichNum);
    gCsAssertRegVal   = chConfRegVal | CSL_MCSPI_CH1CONF_FORCE_MASK;
    gCsDeAssertRegVal = chConfRegVal & (~CSL_MCSPI_CH1CONF_FORCE_MASK);

    //
    // RESET
    //

    for (i=0; i<UCC5870_NFLT_NUM;i++)
    {
        gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(gUCC5870_nFLT_BASE[i]);
        pinNum       = gUCC5870_nFLT_PIN[i];

        GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_DIRECTION_INPUT);
        gUCC5870_nFLT_Status[i]=GPIO_pinRead(gpioBaseAddr, pinNum);

        if(gUCC5870_nFLT_Status[i] == 0){
            //
            // wait for 1ms and check if BIST without failures
            //
            ClockP_usleep(1000L);
            gUCC5870_nFLT_Status[i]=GPIO_pinRead(gpioBaseAddr, pinNum);
        }

        switch(i){
            case 0:
                tripFlagDMC.fault.nFLT1H = (gUCC5870_nFLT_Status[i])? 1:0;
                break;
            case 1:
                tripFlagDMC.fault.nFLT1L = (gUCC5870_nFLT_Status[i])? 1:0;
                break;
            case 2:
                tripFlagDMC.fault.nFLT2H = (gUCC5870_nFLT_Status[i])? 1:0;
                break;
            case 3:
                tripFlagDMC.fault.nFLT2L = (gUCC5870_nFLT_Status[i])? 1:0;
                break;
            default:
                break;
        }

    }

    //
    // CONFIG1
    //

    for (i=0; i<gPWM_NUM;i++){
        EPWM_setRisingEdgeDeadBandDelayInput(gPWM_base[i], EPWM_DB_INPUT_EPWMA);
        EPWM_setFallingEdgeDeadBandDelayInput(gPWM_base[i], EPWM_DB_INPUT_EPWMB);
        EPWM_setDeadBandDelayMode(gPWM_base[i], EPWM_DB_RED, FALSE);
        EPWM_setDeadBandDelayMode(gPWM_base[i], EPWM_DB_FED, FALSE);

        EPWM_setActionQualifierContSWForceAction(gPWM_base[i],
                                        EPWM_AQ_OUTPUT_A,
                                        EPWM_AQ_SW_OUTPUT_LOW);

        EPWM_setActionQualifierContSWForceAction(gPWM_base[i],
                                        EPWM_AQ_OUTPUT_B,
                                        EPWM_AQ_SW_OUTPUT_LOW);
    }

    /* Enable the McSPI channel.*/
    MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChEnableRegVal);

    for (i=0; i<gPWM_NUM;i++)
    {
        //AQ ON PWMxA
        EPWM_setActionQualifierContSWForceAction(gPWM_base[i],
                                        EPWM_AQ_OUTPUT_A,
                                        EPWM_AQ_SW_OUTPUT_HIGH);
        ClockP_usleep(100L);
        UCC5870_WR_CA(gUCC5870_ADDR[i*2]);
        ClockP_usleep(100L);
        EPWM_setActionQualifierContSWForceAction(gPWM_base[i],
                                        EPWM_AQ_OUTPUT_A,
                                        EPWM_AQ_SW_OUTPUT_LOW);


        //AQ ON PWMxB
        EPWM_setActionQualifierContSWForceAction(gPWM_base[i],
                                        EPWM_AQ_OUTPUT_B,
                                        EPWM_AQ_SW_OUTPUT_HIGH);

        ClockP_usleep(100L);
        UCC5870_WR_CA(gUCC5870_ADDR[i*2+1]);
        ClockP_usleep(100L);
        EPWM_setActionQualifierContSWForceAction(gPWM_base[i],
                                        EPWM_AQ_OUTPUT_B,
                                        EPWM_AQ_SW_OUTPUT_LOW);

    }

    UCC5870_CFG_IN(gUCC5870_ADDR[6]);

    //
    // CONFIG2
    //

    for (i=UH; i<=WL;i++)
    {
        //
        // put the drivers in config mode
        //
        UCC5870_CFG_IN(gUCC5870_ADDR[i]);
    }

    /* Disable the McSPI channel.*/
    MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChDisableRegVal);

    gUCC5870_InitFault = 0;

    for (i=UH; i<=WL;i++)
    {
        //
        // logic HIGH bit positions indicate the driver at fault
        //
        gUCC5870_InitStatus[i] = writeVerify_UCC5870(i);
        gUCC5870_InitFault    += (gUCC5870_InitStatus[i] == FAIL) << i;
    }

    for (i=0; i<gPWM_NUM;i++){
        EPWM_setRisingEdgeDeadBandDelayInput(gPWM_base[i], EPWM_DB_INPUT_EPWMA);
        EPWM_setFallingEdgeDeadBandDelayInput(gPWM_base[i], EPWM_DB_INPUT_EPWMA);
        EPWM_setDeadBandDelayMode(gPWM_base[i], EPWM_DB_RED, TRUE);
        EPWM_setDeadBandDelayMode(gPWM_base[i], EPWM_DB_FED, TRUE);

        EPWM_setActionQualifierContSWForceAction(gPWM_base[i],
                                        EPWM_AQ_OUTPUT_A,
                                        EPWM_AQ_SW_DISABLED);

        EPWM_setActionQualifierContSWForceAction(gPWM_base[i],
                                        EPWM_AQ_OUTPUT_B,
                                        EPWM_AQ_SW_DISABLED);
    }

    /* Enable the McSPI channel.*/
    MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChEnableRegVal);

    for (i=UH; i<=WL;i++)
    {
        CLEAR_FAULTS(gUCC5870_ADDR[i]);
    }

    /* Disable the McSPI channel.*/
    MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChDisableRegVal);

    //
    // check if any driver's written data is not verified
    //

    if(gUCC5870_InitFault)
    {
        DebugP_log("INIT_FAULT \r\n");
        return(INIT_FAULT);
    }
    else
    {
        status = inverterDiagnostics();
        if(status == ALL_GOOD)
        {
            //
            // Enable UCC5870s of all phases
            //

            /* Enable the McSPI channel.*/
            MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChEnableRegVal);

            for (i=UH; i<=WL;i++)
            {
                UCC5870_DRV_EN(gUCC5870_ADDR[i]);
            }

            /* Disable the McSPI channel.*/
            MCSPI_writeChCtrlReg(spibaseAddr, spichNum, gChDisableRegVal);
        }
        DebugP_log("status %X\r\n", status);
        return(status);
    }
}

//
// Inverter fault diagnostics
//
UCC5870_Status_e inverterDiagnostics(void)
{
    //
    // Diagnose gate driver status and readiness before enabling
    //
    //
    // logic HIGH bit positions indicate driver at fault
    //
    gUCC5870_StatusFault   = 0;
    //
    // logic HIGH bit positions indicate driver at fault
    //
    gUCC5870_PriReadyFault = 0;
    //
    // logic HIGH bit positions indicate driver at fault
    //
    gUCC5870_SecReadyFault = 0;

    for (uint16_t i=UH; i<=WL;i++)
    {
        gUCC5870_DiagStatus[i]  = diagnose_UCC5870(i);
        gUCC5870_StatusFault   += (gUCC5870_DiagStatus[i] == FALSE) << i;
        gUCC5870_PriReadyFault += (gUCC5870_DEV[i].status2.bit.PRI_RDY == NOT_READY) << i;
        gUCC5870_SecReadyFault += (gUCC5870_DEV[i].status4.bit.SEC_RDY == NOT_READY) << i;
    }
    //
    // check if any driver's status register indicates fault
    //
    if(gUCC5870_StatusFault)
    {
        return(STATUS_FAULT);
    }

    //
    // check if any driver's primary ready flag is not set
    //
    if(gUCC5870_PriReadyFault)
    {
        return(PRI_RDY_FAULT);
    }

    //
    // check if any driver's secondary ready flag is not set
    //
    if(gUCC5870_SecReadyFault)
    {
        return(SEC_RDY_FAULT);
    }

    return(ALL_GOOD);
}

//
// UCC5870 fault diagnostics
//
uint16_t diagnose_UCC5870 (uint16_t i)
{
    uint16_t   j,
               faults[5] = {0, 0, 0, 0, 0},
               faultAcc;
    uint16_t * statusReg;

    statusReg = (uint16_t *) &gUCC5870_DEV[i].status1;
    faultAcc = 0;
    for (j = 0; j < 4; j++)
    {
        //
        // read STATUSx register / check / accumulate fault flags (HIGH bits)
        //
        statusReg[j] = readRegUCC5870 (gUCC5870_ADDR[i], gUCC5870_statusRegAdrs[j]);
        faults[j]    = statusReg[j] & gUCC5870_statusFaultMask[j];
        faultAcc    += faults[j];

    }

    //
    // faultAcc = 0 if all ok
    //
    return( (faultAcc == 0) ? PASS : FAIL );
}

//
// UCC5870 device configuration
//
uint16_t writeVerify_UCC5870(uint16_t i)
{
    uint16_t   failCnt = 0;
    uint16_t * ptr = (uint16_t *) &gUCC5870_DEV[i];

    //
    // set up the basis for enabling various functions
    //
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], CFG2,    ptr[CFG2]   ) == FAIL);
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], CFG3,    ptr[CFG3]   ) == FAIL);
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], CFG6,    ptr[CFG6]   ) == FAIL);
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], CFG7,    ptr[CFG7]   ) == FAIL);
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], CFG9,    ptr[CFG9]   ) == FAIL);
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], CFG11,   ptr[CFG11]  ) == FAIL);
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], ADCCFG,  ptr[ADCCFG] ) == FAIL);
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], DOUTCFG, ptr[DOUTCFG]) == FAIL);

    //
    // various function enable bits present in these CFG registers
    //
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], CFG1,    ptr[CFG1]   ) == FAIL);
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], CFG4,    ptr[CFG4]   ) == FAIL);
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], CFG5,    ptr[CFG5]   ) == FAIL);
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], CFG8,    ptr[CFG8]   ) == FAIL);
    failCnt += (writeVerifyRegUCC5870 (gUCC5870_ADDR[i], CFG10,   ptr[CFG10]  ) == FAIL);

    return( (failCnt == 0) ? PASS : FAIL );
}

