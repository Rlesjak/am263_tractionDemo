//#############################################################################
//
// FILE:  ucc5870_regs.h
//
// TITLE: header file for ucc5870 Registers and bit field definitions
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef UCC5870_REGS_H
#define UCC5870_REGS_H

//
// Register Addresses
//
//UCC5870 Register Addresses
//
#define  CFG1            0x00
#define  CFG2            0x01
#define  CFG3            0x02
#define  CFG4            0x03
#define  CFG5            0x04
#define  CFG6            0x05
#define  CFG7            0x06
#define  CFG8            0x07
#define  CFG9            0x08
#define  CFG10           0x09
#define  CFG11           0x0a
#define  ADCDATA1        0x0b
#define  ADCDATA2        0x0c
#define  ADCDATA3        0x0d
#define  ADCDATA4        0x0e
#define  ADCDATA5        0x0f
#define  ADCDATA6        0x10
#define  ADCDATA7        0x11
#define  ADCDATA8        0x12
#define  CRCDATA         0x13
#define  SPITEST         0x14
#define  GD_ADDR         0x15
#define  STATUS1         0x16
#define  STATUS2         0x17
#define  STATUS3         0x18
#define  STATUS4         0x19
#define  STATUS5         0x1a
#define  CONTROL1        0x1b
#define  CONTROL2        0x1c
#define  ADCCFG          0x1d
#define  DOUTCFG         0x1e

//
// REGISTER BIT FIELD DEFINITIONS
//
// UCC5870 CFG1 Register bit definitions:
//
struct  UCC5870_CFG1_REG_BITS
{                                 // bit    description
    uint16_t  TDEAD          : 6; //  5:0    dead time
    uint16_t  NFLT2_DOUT_MUX : 1; //  6         nFLT2/DOUT pin function
    uint16_t  rsvd7          : 1; //  7      PVDD ov flag
    uint16_t  OV1_DIS        : 1; //  8      VCC1 OVLO disable
    uint16_t  rsvd9          : 1; //  9      PVDD ov flag
    uint16_t  GD_TWN_DIS     : 1; // 10         OTW VCC1 side
    uint16_t  IO_DEGLITCH    : 2; // 12:11   IN deglitch time
    uint16_t  OVLO1_LEVEL    : 1; // 13      VCC1 OVLO setting
    uint16_t  UVLO1_LEVEL    : 1; // 14      VCC1 UVLO setting
    uint16_t  UV1_DIS        : 1; // 15      VCC1 UVLO disable
};

typedef union
{
   uint16_t                      all;
   struct UCC5870_CFG1_REG_BITS    bit;
} UCC5870_CFG1_REG;

//
// T in ns
//
#define  Tdead(T)   ( ((T - 105) / 70) + 1)

enum {
    nFLT2_DOUT_nFLT2 = 0,
    nFLT2_DOUT_DOUT  = 1
};

enum {
    vcc1_ovlo_enable  = 0,
    vcc1_ovlo_disable = 1
};

enum {
    OT_warning_enable  = 0,
    OT_warning_disable = 1
};

enum {
    io_deglitch_bypass = 0,
    io_deglitch_70ns   = 1,
    io_deglitch_140ns  = 2,
    io_deglitch_210ns  = 3
};

enum {
    vcc1_3V3 = 0,
    vcc1_5v0 = 1
};

enum {
    vcc1_uvlo_enable  = 0,
    vcc1_uvlo_disable = 1
};

//
// UCC5870 CFG2 Register bit definitions:
//
struct  UCC5870_CFG2_REG_BITS
{                                      // bit    description
    uint16_t  PWM_CHK_FAULT      : 1;  //  0      PWM check fault
    uint16_t  VREG1_ILIMIT_FAULT : 1;  //  1      Vreg Ilimit fault
    uint16_t  GD_TWN_FAULT       : 1;  //  2      temp warning fault
    uint16_t  rsvd               : 2;  //  4-3
    uint16_t  BIST_FAULT         : 1;  //  5      Analog BIST fault
    uint16_t  TRIM_CRC_FAULT     : 1;  //  6      TRIM CRC fault
    uint16_t  INT_REG_FAULT      : 1;  //  7      Internal regulator fault
    uint16_t  CFG_CRC_FAULT      : 1;  //  8      CFG reg CRC fault
    uint16_t  SPI_FAULT          : 2;  // 10-9    SPI fault
    uint16_t  CLK_MON_FAULT      : 1;  // 11      Clock monitor fault
    uint16_t  STP_FAULT          : 1;  // 12      STP fault
    uint16_t  UVLO1_FAULT        : 1;  // 13      UVLO fault
    uint16_t  OVLO1_FAULT        : 1;  // 14      OVLO fault
    uint16_t  INT_COMM_FAULT     : 1;  // 15      Inter-die comm failure
};

typedef union
{
    uint16_t                     all;
    struct UCC5870_CFG2_REG_BITS bit;
} UCC5870_CFG2_REG;

enum {
    report_fault_on_nFLT1      = 0,
    dont_report_fault_on_nFLT1 = 1
};

enum {
    dont_report_comm_fault_on_nFLT1 = 0,
    report_comm_fault_on_nFLT1 = 1
};

enum {
    no_warning_on_nFLT1 = 0,
    warning_on_nFLT1 = 1
};

enum {
    report_SPI_fault_on_nFLT1 = 0,
    report_SPI_fault_on_nFLT2 = 1,
    dont_report_SPI_fault     = 2
};

//
// UCC5870 CFG3 Register bit definitions:
//
struct  UCC5870_CFG3_REG_BITS
{                                         // bit     description
    uint16_t  AI_IZTC_SEL             : 3; //  2:0
    uint16_t  FS_STATE_CFG_CRC_FAULT  : 1; //  3
    uint16_t  ITO2_EN                 : 1; //  4
    uint16_t  ITO1_EN                 : 1; //  5
    uint16_t  FS_STATE_INT_COMM_FAULT : 1; //  6
    uint16_t  FS_STATE_INT_REG_FAULT  : 1; //  7
    uint16_t  FS_STATE_SPI_FAULT      : 2; //  9:8
    uint16_t  rsvd10                  : 1; // 10
    uint16_t  FS_STATE_STP_FAULT      : 2; // 12:11
    uint16_t  FS_STATE_PWM_CHK        : 1; // 13
    uint16_t  FS_STATE_OVLO1_FAULT    : 1; // 14
    uint16_t  FS_STATE_UVLO1_FAULT    : 1; // 15
};

typedef union
{
    uint16_t                     all;
    struct UCC5870_CFG3_REG_BITS bit;
} UCC5870_CFG3_REG;

enum {
    AI1_AI3_AI5_bias_currents_off = 0,
    AI1_bias_current_on           = 1,
    AI3_bias_current_on           = 2,
    AI1_AI3_bias_currents_on      = 3,
    AI5_bias_current_on           = 4,
    AI1_AI5_bias_currents_on      = 5,
    AI3_AI5_bias_currents_on      = 6,
    AI1_AI3_AI5_bias_currents_on  = 7
};

enum {
    OUTx_LOW    = 0,
    OUTx_IGNORE = 1
};

enum {
    FS_SPI_OUTx_LOW    = 0,
    FS_SPI_OUTx_HIGH   = 1,
    FS_SPI_OUTx_IGNORE = 2
};

enum {
    FS_STP_OUTx_LOW    = 0,
    FS_STP_OUTx_HIGH   = 1,
    FS_STP_OUTx_IGNORE = 3
};

enum {
    CURRENT_SRC_OUT_AI2_4_6_DISABLED = 0,
    CURRENT_SRC_OUT_AI2_4_6_ENABLED  = 1
};

enum {
    CURRENT_SRC_OUT_AI1_3_5_DISABLED = 0,
    CURRENT_SRC_OUT_AI1_3_5_ENABLED  = 1
};

//
// UCC5870 CFG4 Register bit definitions:
//
struct  UCC5870_CFG4_REG_BITS
{                                  // bit     description
    uint16_t  UVOV3_EN        : 1; //  0
    uint16_t  PS_TSD_EN       : 1; //  1
    uint16_t  OCP_DIS         : 1; //  2
    uint16_t  SCP_DIS         : 1; //  3
    uint16_t  DESAT_EN        : 1; //  4
    uint16_t  VCECLP_EN       : 1; //  5
    uint16_t  MCLP_DIS        : 1; //  6
    uint16_t  GM_EN           : 1; //  7
    uint16_t  GM_BLK          : 2; //  9:8
    uint16_t  MCLP_CFG        : 1; // 10
    uint16_t  OV2_DIS         : 1; // 11
    uint16_t  DESAT_DEGLITCH  : 1; // 12
    uint16_t  PS_TSD_DEGLITCH : 2; // 14:13
    uint16_t  UV2_DIS         : 1; // 15
};

typedef union  {
    uint16_t                     all;
    struct UCC5870_CFG4_REG_BITS bit;
} UCC5870_CFG4_REG;

enum {
    VEE2_uvlo_ovlo_disable = 0,
    VEE2_uvlo_ovlo_enable = 1
};

enum {
    Switch_thermal_SD_disable = 0,
    Switch_thermal_SD_enable = 1
};

enum {
    OCP_enable = 0,
    OCP_disable = 1
};

enum {
    SCP_enable = 0,
    SCP_disable = 1
};

enum {
    DESAT_det_disable = 0,
    DESAT_det_enable = 1
};

enum {
    VCE_CLAMP_disable = 0,
    VCE_CLAMP_enable = 1
};

enum {
    MILLER_CLAMP_enable = 0,
    MILLER_CLAMP_disable = 1
};

enum {
    Gate_Volt_Mon_enable = 0,
    Gate_Volt_Mon_disable = 1
};

enum {
    Gate_volt_mon_blank_500ns  = 0,
    Gate_volt_mon_blank_1000ns = 1,
    Gate_volt_mon_blank_2500ns = 2,
    Gate_volt_mon_blank_4000ns = 3
};

enum {
    MILLER_CLAMP_internal = 0,
    MILLER_CLAMP_external = 1
};

enum {
    VCC2_ovlo_enable = 0,
    VCC2_ovlo_disable = 1
};

enum {
    DESAT_deglitch_158ns = 0,
    DESAT_deglitch_316ns = 1
};

enum {
    TSD_250ns = 0,
    TSD_500ns = 1,
    TSD_750ns = 2,
    TSD_1000ns = 3
};

enum {
    VCC2_uvlo_enable = 0,
    VCC2_uvlo_disable = 1
};


//
// UCC5870 CFG5 Register bit definitions:
//
struct  UCC5870_CFG5_REG_BITS
{                                  // bit     description
    uint16_t  PWM_MUTE_EN     : 1; //  0
    uint16_t  TLTOFF_STO_EN   : 3; //  3:1
    uint16_t  STO_CURR        : 2; //  5:4
    uint16_t  MCLPTH          : 2; //  7:6
    uint16_t  DESAT_DCHG_EN   : 1; //  8
    uint16_t  DESAT_CHG_CURR  : 2; // 10:9
    uint16_t  DESATTH         : 4; // 14:11
    uint16_t  GM_STO2LTO_DIS  : 1; // 15
};

typedef union
{
    uint16_t                     all;
    struct UCC5870_CFG5_REG_BITS bit;
} UCC5870_CFG5_REG;

enum {
    PWM_MUTE_FOR_SC_OC_OT_FAULTS_DISABLED = 0,
    PWM_MUTE_FOR_SC_OC_OT_FAULTS_ENABLED  = 1
};

enum {
    STO_2LTOFF_DISABLED        = 0,
    STO_FOR_SC_DESAT           = 1,
    STO_FOR_SC_DESAT_OC        = 2,
    STO_FOR_SC_DESAT_OC_TSD    = 3,
    TLTOFF_FOR_SC_DESAT        = 5,
    TLTOFF_FOR_SC_DESAT_OC     = 6,
    TLTOFF_FOR_SC_DESAT_OC_TSD = 7
};

enum {
    SOFT_TURN_OFF_0_3A = 0,
    SOFT_TURN_OFF_0_6A = 1,
    SOFT_TURN_OFF_0_9A = 2,
    SOFT_TURN_OFF_1_2A = 3
};

enum {
    MILLER_CLAMP_VTH_1p5V = 0,
    MILLER_CLAMP_VTH_2V   = 1,
    MILLER_CLAMP_VTH_3V   = 2,
    MILLER_CLAMP_VTH_4V   = 3
};

enum {
    DESAT_DISCHARGE_DISABLE = 0,
    DESAT_DISCHARGE_ENABLE  = 1
};

enum {
    BLANK_CAP_CHRG_CUR_600uA = 0,
    BLANK_CAP_CHRG_CUR_700uA = 1,
    BLANK_CAP_CHRG_CUR_800uA = 2,
    BLANK_CAP_CHRG_CUR_1000uA = 3,
};

enum {
    DESAT_VTH_2p5 = 0,
    DESAT_VTH_3p0 = 1,
    DESAT_VTH_3p5 = 2,
    DESAT_VTH_4p0 = 3,
    DESAT_VTH_4p5 = 4,
    DESAT_VTH_5p0 = 5,
    DESAT_VTH_5p5 = 6,
    DESAT_VTH_6p0 = 7,
    DESAT_VTH_6p5 = 8,
    DESAT_VTH_7p0 = 9,
    DESAT_VTH_7p5 = 10,
    DESAT_VTH_8p0 = 11,
    DESAT_VTH_8p5 = 12,
    DESAT_VTH_9p0 = 13,
    DESAT_VTH_9p5 = 14,
    DESAT_VTH_10p0 = 15,
};

enum {
    GATE_MON_FLT_DURING_STO_TLTOFF_ENABLED = 0,
    GATE_MON_FLT_DURING_STO_TLTOFF_DISABLED = 1
};

//
// UCC5870 CFG6 Register bit definitions:
//
struct  UCC5870_CFG6_REG_BITS
{                                 // bit     description
    uint16_t  PS_TSDTH   : 3;     //  2:0
    uint16_t  TEMP_CURR  : 2;     //  4:3
    uint16_t  OC_BLK     : 3;     //  7:5
    uint16_t  SC_BLK     : 2;     //  9:8
    uint16_t  SCTH       : 2;     // 11:10
    uint16_t  OCTH       : 4;     // 15:12
};

typedef union  {
    uint16_t                     all;
    struct UCC5870_CFG6_REG_BITS bit;
} UCC5870_CFG6_REG;

enum {
    TSD_TH_1p00 = 0,
    TSD_TH_1p25 = 1,
    TSD_TH_1p50 = 2,
    TSD_TH_1p75 = 3,
    TSD_TH_2p00 = 4,
    TSD_TH_2p25 = 5,
    TSD_TH_2p50 = 6,
    TSD_TH_2p75 = 7
};

enum {
    TEMP_CURR_0p1A = 0,
    TEMP_CURR_0p3A = 1,
    TEMP_CURR_0p5A = 2,
    TEMP_CURR_1p0A = 3
};

enum {
    OCD_BLANK_TIME_0p5us = 0,
    OCD_BLANK_TIME_1p0us = 1,
    OCD_BLANK_TIME_1p5us = 2,
    OCD_BLANK_TIME_2p0us = 3,
    OCD_BLANK_TIME_2p5us = 4,
    OCD_BLANK_TIME_3p0us = 5,
    OCD_BLANK_TIME_5p0us = 6,
    OCD_BLANK_TIME_10p0us = 7
};

enum {
    SCD_BLANK_TIME_0p1us = 0,
    SCD_BLANK_TIME_0p2us = 1,
    SCD_BLANK_TIME_0p4us = 2,
    SCD_BLANK_TIME_0p8us = 3
};

enum {
    SCD_VTH_0p5V  = 0,
    SCD_VTH_0p75V = 1,
    SCD_VTH_1p0V  = 2,
    SCD_VTH_1p25V = 3
};

enum {
    OCD_VTH_0p2V  = 0,
    OCD_VTH_0p25V = 1,
    OCD_VTH_0p3V  = 2,
    OCD_VTH_0p35V = 3,
    OCD_VTH_0p4V  = 4,
    OCD_VTH_0p45V = 5,
    OCD_VTH_0p5V  = 6,
    OCD_VTH_0p55V = 7,
    OCD_VTH_0p6V  = 8,
    OCD_VTH_0p65V = 9,
    OCD_VTH_0p7V  = 10,
    OCD_VTH_0p75V = 11,
    OCD_VTH_0p8V  = 12,
    OCD_VTH_0p85V = 13,
    OCD_VTH_0p9V  = 14,
    OCD_VTH_0p95V = 15
};

//
// UCC5870 CFG7 Register bit definitions:
//
struct  UCC5870_CFG7_REG_BITS
{                                     // bit     description
    uint16_t  FS_STATE_ADC_FAULT : 2; // 1:0
    uint16_t  ADC_FAULT_P        : 1; //  2
    uint16_t  ADC_SAMP_DLY       : 2; //  4:3
    uint16_t  ADC_SAMP_MODE      : 2; //  6:5
    uint16_t  ADC_EN             : 1; //  7
    uint16_t  OVLO3TH            : 2; //  9:8
    uint16_t  UVLO3TH            : 2; // 11:10
    uint16_t  OVLO2TH            : 2; // 13:12
    uint16_t  UVLO2TH            : 2; // 15:14
};

typedef union  {
    uint16_t                     all;
    struct UCC5870_CFG7_REG_BITS bit;
} UCC5870_CFG7_REG;

enum {
    FS_ADC_FLT_OUT_LOW    = 0,
    FS_ADC_FLT_OUT_HIGH   = 1,
    FS_ADC_FLT_OUT_HiZ    = 2,
    FS_ADC_FLT_OUT_IGNORE = 3
};

enum {
    dont_report_adcfault_on_nFLT1 = 0,
    report_adcfault_on_nFLT1      = 1
};

enum {
    ADC_SAMP_DELAY_280ns = 0,
    ADC_SAMP_DELAY_560ns = 1,
    ADC_SAMP_DELAY_840ns = 2,
    ADC_SAMP_DELAY_1120ns = 3
};

enum {
    ADC_SAMP_CENTER_ALIGN  = 0,
    ADC_SAMP_EDGE_ALIGN    = 1,
    ADC_SAMP_CENTER_HYBRID = 2,
    ADC_SAMP_RESERVED      = 3
};

enum {
    ADC_SAMP_DISABLE = 0,
    ADC_SAMP_ENABLE  = 1
};

enum {
    VEE2_OVLO_VTH_M5V  = 0,
    VEE2_OVLO_VTH_M7V  = 1,
    VEE2_OVLO_VTH_M10V = 2,
    VEE2_OVLO_VTH_M12V = 3
};

enum {
    VEE2_UVLO_VTH_M3V  = 0,
    VEE2_UVLO_VTH_M5V  = 1,
    VEE2_UVLO_VTH_M8V  = 2,
    VEE2_UVLO_VTH_M10V = 3
};

enum {
    VCC2_OVLO_VTH_P23V  = 0,
    VCC2_OVLO_VTH_P21V  = 1,
    VCC2_OVLO_VTH_P19V  = 2,
    VCC2_OVLO_VTH_P17V  = 3
};

enum {
    VCC2_UVLO_VTH_P16V  = 0,
    VCC2_UVLO_VTH_P14V  = 1,
    VCC2_UVLO_VTH_P12V  = 2,
    VCC2_UVLO_VTH_P10V  = 3
};

//
// UCC5870 CFG8 Register bit definitions:
//
struct  UCC5870_CFG8_REG_BITS
{                                      // bit     description
    uint16_t  IOUT_SEL         : 3;    //  2:0
    uint16_t  AI_ASC_MUX       : 1;    //  3
    uint16_t  VREF_SEL         : 1;    //  4
    uint16_t  GD_2LOFF_STO_EN  : 1;    //  5
    uint16_t  CRC_DIS          : 1;    //  6
    uint16_t  rsvd7            : 1;    //  7
    uint16_t  GD_2LOFF_CURR    : 2;    //  9:8
    uint16_t  GD_2LOFF_TIME    : 3;    // 12:10
    uint16_t  GD_2LOFF_VOLT    : 3;    // 15:13
};

typedef union  {
    uint16_t                     all;
    struct UCC5870_CFG8_REG_BITS bit;
} UCC5870_CFG8_REG;

enum {
    GATE_DRIVE_STRENGTH_FULL = 0,
    GATE_DRIVE_STRENGTH_3rd  = 1,
    GATE_DRIVE_STRENGTH_6th  = 2
};

enum {
    AI_ASC_MUX_ASC = 0,
    AI_ASC_MUX_AI  = 1
};

enum {
    VREF_INTERNAL = 0,
    VREF_EXTERNAL = 1
};

enum {
    GD_2LOFF_STO_DISABLE = 0,
    GD_2LOFF_STO_ENABLE  = 1
};

enum {
    CRC_CHECK_ENABLE  = 0,
    CRC_CHECK_DISABLE = 1
};

enum {
    GD_2LOFF_DISCHARGE_CURR_0p3A = 0,
    GD_2LOFF_DISCHARGE_CURR_0p6A = 1,
    GD_2LOFF_DISCHARGE_CURR_0p9A = 2,
    GD_2LOFF_DISCHARGE_CURR_1p2A = 3
};

enum {
    GD_2LOFF_PLATEAU_TIME_150ns  = 0,
    GD_2LOFF_PLATEAU_TIME_300ns  = 1,
    GD_2LOFF_PLATEAU_TIME_450ns  = 2,
    GD_2LOFF_PLATEAU_TIME_600ns  = 3,
    GD_2LOFF_PLATEAU_TIME_1000ns = 4,
    GD_2LOFF_PLATEAU_TIME_1500ns = 5,
    GD_2LOFF_PLATEAU_TIME_2000ns = 6,
    GD_2LOFF_PLATEAU_TIME_2500ns = 7
};

enum {
    GD_2LOFF_PLATEAU_VOLT_6V = 0,
    GD_2LOFF_PLATEAU_VOLT_7V = 1,
    GD_2LOFF_PLATEAU_VOLT_8V = 2,
    GD_2LOFF_PLATEAU_VOLT_9V = 3,
    GD_2LOFF_PLATEAU_VOLT_10V = 4,
    GD_2LOFF_PLATEAU_VOLT_11V = 5,
    GD_2LOFF_PLATEAU_VOLT_12V = 6,
    GD_2LOFF_PLATEAU_VOLT_13V = 7

};

//
// UCC5870 CFG9 Register bit definitions:
//
struct  UCC5870_CFG9_REG_BITS
{                                      // bit     description
    uint16_t  CLK_MON_SEC_FAULT  : 1;  //  0
    uint16_t  VREG2_ILIMIT_FAULT : 1;  //  1
    uint16_t  BIST_SEC_FAULT     : 1;  //  2
    uint16_t  INT_REG_SEC_FAULT  : 1;  //  3
    uint16_t  TRIM_CRC_SEC_FAULT : 1;  //  4
    uint16_t  CFG_CRC_SEC_FAULT  : 1;  //  5
    uint16_t  INT_COMM_SEC_FAULT : 1;  //  6
    uint16_t  GD_TSD_FAULT       : 1;  //  7
    uint16_t  PS_TSD_FAULT       : 1;  //  8
    uint16_t  OVLO23_FAULT       : 1;  //  9
    uint16_t  UVLO23_FAULT       : 1;  // 10
    uint16_t  GM_FAULT           : 2;  // 12:11
    uint16_t  OC_FAULT           : 1;  // 13
    uint16_t  SC_FAULT           : 1;  // 14
    uint16_t  SPARE              : 1;  // 15
};

typedef union  {
    uint16_t                     all;
    struct UCC5870_CFG9_REG_BITS bit;
} UCC5870_CFG9_REG;

enum {
    GM_FAULT_IGNORE   = 0,
    GM_FAULT_ON_nFLT1 = 1,
    GM_FAULT_ON_nFLT2 = 2,
    GM_STATE_ON_nFLT2 = 3
};

enum {
    dont_report_TSD_on_nFLT1 = 0,
    report_TSD_on_nFLT1 = 1
};

//
// UCC5870 CFG10 Register bit definitions:
//
struct  UCC5870_CFG10_REG_BITS
{                                         // bit     description
    uint16_t  FS_STATE_INT_COMM_SEC  : 2; //  1:0
    uint16_t  FS_STATE_GM            : 2; //  3:2
    uint16_t  FS_STATE_GD_TSD        : 2; //  5:4
    uint16_t  FS_STATE_PS_TSD        : 2; //  7:6
    uint16_t  FS_STATE_OCP           : 2; //  9:8
    uint16_t  rsvd10                 : 1; // 10
    uint16_t  FS_STATE_INT_REG_FAULT : 1; // 11
    uint16_t  FS_STATE_DESAT_SCP     : 2; // 13:12
    uint16_t  SPARE                  : 1; // 14
    uint16_t  GD_TWN_SEC_EN          : 1; // 15
};

typedef union  {
    uint16_t                      all;
    struct UCC5870_CFG10_REG_BITS bit;
} UCC5870_CFG10_REG;

enum {
    PULLED_LOW = 0,
    PULLED_HIGH = 1,
    NO_ACTION = 3
};

enum {
    GD_TWN_SEC_DISABLED = 0,
    GD_TWN_SEC_ENABLED  = 1
};

//
// UCC5870 CFG11 Register bit definitions:
//
struct  UCC5870_CFG11_REG_BITS
{                                               // bit     description
    uint16_t  FS_STATE_CLK_MON_SEC_FAULT  : 2;  //  1:0
    uint16_t  VCE_CLMP_HLD_TIME           : 2;  //  3:2
    uint16_t  FS_STATE_CFG_CRC_SEC_FAULT  : 2;  //  5:4
    uint16_t  FS_STATE_TRIM_CRC_SEC_FAULT : 2;  //  7:6
    uint16_t  FS_STATE_OVLO3              : 2;  //  9:8
    uint16_t  FS_STATE_UVLO3              : 2;  // 11:10
    uint16_t  FS_STATE_OVLO2              : 2;  // 13:12
    uint16_t  FS_STATE_UVLO2              : 2;  // 15:14
};

typedef union  {
    uint16_t                      all;
    struct UCC5870_CFG11_REG_BITS bit;
} UCC5870_CFG11_REG;

enum {
    VCE_CLAMP_HOLD_TIME_100ns = 0,
    VCE_CLAMP_HOLD_TIME_200ns = 1,
    VCE_CLAMP_HOLD_TIME_300ns = 2,
    VCE_CLAMP_HOLD_TIME_400ns = 3
};

//
// UCC5870 ADCDATA1 Register bit definitions:
//
struct  UCC5870_ADCDATA1_REG_BITS
{                                // bit     description
    uint16_t DATA_AI1   : 10;    //  9-0
    uint16_t TIME_STAMP :  6;    // 15-10
};

typedef union  {
    uint16_t                         all;
    struct UCC5870_ADCDATA1_REG_BITS bit;
} UCC5870_ADCDATA1_REG;

//
// UCC5870 ADCDATA2 Register bit definitions:
//
struct  UCC5870_ADCDATA2_REG_BITS
{                                // bit     description
    uint16_t DATA_AI3   : 10;    //  9-0
    uint16_t TIME_STAMP :  6;    // 15-10
};

typedef union  {
    uint16_t                         all;
    struct UCC5870_ADCDATA2_REG_BITS bit;
} UCC5870_ADCDATA2_REG;

//
// UCC5870 ADCDATA3 Register bit definitions:
//
struct  UCC5870_ADCDATA3_REG_BITS
{                                // bit     description
    uint16_t DATA_AI5   : 10;    //  9-0
    uint16_t TIME_STAMP :  6;    // 15-10
};

typedef union  {
    uint16_t                         all;
    struct UCC5870_ADCDATA3_REG_BITS bit;
} UCC5870_ADCDATA3_REG;

//
// UCC5870 ADCDATA4 Register bit definitions:
//
struct  UCC5870_ADCDATA4_REG_BITS
{                                // bit     description
    uint16_t DATA_AI2   : 10;    //  9-0
    uint16_t TIME_STAMP :  6;    // 15-10
};

typedef union  {
    uint16_t                         all;
    struct UCC5870_ADCDATA4_REG_BITS bit;
} UCC5870_ADCDATA4_REG;

//
// UCC5870 ADCDATA5 Register bit definitions:
//
struct  UCC5870_ADCDATA5_REG_BITS
{                                // bit     description
    uint16_t DATA_AI4   : 10;    //  9-0
    uint16_t TIME_STAMP :  6;    // 15-10
};

typedef union  {
    uint16_t                         all;
    struct UCC5870_ADCDATA5_REG_BITS bit;
} UCC5870_ADCDATA5_REG;

//
// UCC5870 ADCDATA6 Register bit definitions:
//
struct  UCC5870_ADCDATA6_REG_BITS
{                                // bit     description
    uint16_t DATA_AI6   : 10;    //  9-0
    uint16_t TIME_STAMP :  6;    // 15-10
};

typedef union  {
    uint16_t                         all;
    struct UCC5870_ADCDATA6_REG_BITS bit;
} UCC5870_ADCDATA6_REG;

//
// UCC5870 ADCDATA7 Register bit definitions:
//
struct  UCC5870_ADCDATA7_REG_BITS
{                                // bit     description
    uint16_t DATA_DTEMP : 10;    //  9-0
    uint16_t TIME_STAMP :  6;    // 15-10
};

typedef union  {
    uint16_t                         all;
    struct UCC5870_ADCDATA7_REG_BITS bit;
} UCC5870_ADCDATA7_REG;

//
// UCC5870 ADCDATA8 Register bit definitions:
//
struct  UCC5870_ADCDATA8_REG_BITS
{                                // bit     description
    uint16_t DATA_OUTH  : 10;    //  9-0
    uint16_t TIME_STAMP :  6;    // 15-10
};

typedef union  {
    uint16_t                         all;
    struct UCC5870_ADCDATA8_REG_BITS bit;
} UCC5870_ADCDATA8_REG;

//
// UCC5870 CRCDATA Register bit definitions:
//
struct  UCC5870_CRCDATA_REG_BITS
{                            // bit     description
    uint16_t CRC_RX : 8;     //  7-0
    uint16_t CRC_TX : 8;     // 15-8
};

typedef union  {
    uint16_t                        all;
    struct UCC5870_CRCDATA_REG_BITS bit;
} UCC5870_CRCDATA_REG;

//
// UCC5870 SPITEST Register bit definitions:
//
struct  UCC5870_SPITEST_REG_BITS
{                           // bit     description
    uint16_t rsvd:16;       // 15-10
};

typedef union  {
    uint16_t                        all;
    struct UCC5870_SPITEST_REG_BITS bit;
} UCC5870_SPITEST_REG;

//
// UCC5870 GD_ADDR Register bit definitions:
//
struct  UCC5870_GD_ADDR_REG_BITS
{                              // bit     description
    uint16_t GD_ADDRESS :  4;     //  3-0
    uint16_t rsvd       : 12;     // 15-4
};

typedef union  {
    uint16_t                        all;
    struct UCC5870_GD_ADDR_REG_BITS bit;
} UCC5870_GD_ADDR_REG;

//
// UCC5870 STATUS1 Register bit definitions:
//
struct  UCC5870_STATUS1_REG_BITS
{                                       // bit     description
    uint16_t  rsvd0              : 1;   //  0
    uint16_t  GD_TWN_FAULT       : 1;   //  1
    uint16_t  rsvd2              : 3;   //  4:2
    uint16_t  PWM_COMP_CHK_FAULT : 1;   //  5
    uint16_t  OPM                : 3;   //  8:6
    uint16_t  rsvd9              : 2;   // 10:9
    uint16_t  EN_STATE           : 1;   // 11
    uint16_t  rsvd12             : 2;   // 13:12
    uint16_t  INN_STATE          : 1;   // 14
    uint16_t  INP_STATE          : 1;   // 15
};

typedef union  {
    uint16_t                        all;
    struct UCC5870_STATUS1_REG_BITS bit;
} UCC5870_STATUS1_REG;

enum {
    NO_FAULT = 0,
    FAULT    = 1
};

enum {
    LOW  = 0,
    HIGH = 1
};

enum {
    OP_MODE_ERROR0 = 0,
    OP_MODE_CFG1   = 1,
    OP_MODE_CFG2   = 2,
    OP_MODE_ACTIVE = 3,
    OP_MODE_ERROR4 = 4,
    OP_MODE_ERROR5 = 5,
    OP_MODE_ERROR6 = 6,
    OP_MODE_ERROR7 = 7
};

//
// UCC5870 STATUS1 Register's FAULT bit mask definitions
//
#define  GD_TWN_FAULT_MASK         (1U << 1)
#define  PWM_COMP_CHK_FAULT_MASK   (1U << 5)

//
// UCC5870 STATUS2 Register bit definitions:
//
struct  UCC5870_STATUS2_REG_BITS
{                                   // bit     description
    uint16_t  OR_NFLT2_PRI   : 1;   //  0
    uint16_t  OR_NFLT1_PRI   : 1;   //  1
    uint16_t  DRV_EN_RCVD    : 1;   //  2
    uint16_t  TRIM_CRC_FAULT : 1;   //  3
    uint16_t  CFG_CRC_FAULT  : 1;   //  4
    uint16_t  CLK_MON_FAULT  : 1;   //  5
    uint16_t  BIST_FAULT     : 1;   //  6
    uint16_t  INT_COMM_FAULT : 1;   //  7
    uint16_t  INT_REG_FAULT  : 1;   //  8
    uint16_t  SPI_FAULT      : 1;   //  9
    uint16_t  rsvd10         : 1;   // 10
    uint16_t  STP_FAULT      : 1;   // 11
    uint16_t  OVLO1_FAULT    : 1;   // 12
    uint16_t  UVLO1_FAULT    : 1;   // 13
    uint16_t  PRI_RDY        : 1;   // 14
    uint16_t  rsvd15         : 1;   // 15
};

typedef union  {
    uint16_t                        all;
    struct UCC5870_STATUS2_REG_BITS bit;
} UCC5870_STATUS2_REG;

enum {
    NOT_READY = 0,
    READY     = 1
};

enum {
    DRIVER_NOT_ENABLED = 0,
    DRIVER_ENABLED     = 1
};

//
// UCC5870 STATUS2 Register's FAULT bit mask definitions
//
#define  OR_NFLT2_MASK            (1U << 0)
#define  OR_NFLT1_MASK            (1U << 1)
#define  TRIM_CRC_FAULT_MASK      (1U << 3)
#define  CFG_CRC_FAULT_MASK       (1U << 4)
#define  CLK_MON_FAULT_MASK       (1U << 5)
#define  BIST_FAULT_MASK          (1U << 6)
#define  INT_COMM_FAULT_MASK      (1U << 7)
#define  INT_REG_FAULT_MASK       (1U << 8)
#define  SPI_FAULT_MASK           (1U << 9)
#define  STP_FAULT_MASK           (1U << 11)
#define  OVLO1_FAULT_MASK         (1U << 12)
#define  UVLO1_FAULT_MASK         (1U << 13)

//
// UCC5870 STATUS3 Register bit definitions:
//
struct  UCC5870_STATUS3_REG_BITS
{                                      // bit     description
    uint16_t  DESAT_FAULT        : 1;  //  0
    uint16_t  OC_FAULT           : 1;  //  1
    uint16_t  SC_FAULT           : 1;  //  2
    uint16_t  VREG2_ILIMIT_FAULT : 1;  //  3
    uint16_t  rsvd4              : 1;  //  4
    uint16_t  PS_TSD_FAULT       : 1;  //  5
    uint16_t  VCEOV_FAULT        : 1;  //  6
    uint16_t  UVLO2_FAULT        : 1;  //  7
    uint16_t  OVLO2_FAULT        : 1;  //  8
    uint16_t  UVLO3_FAULT        : 1;  //  9
    uint16_t  OVLO3_FAULT        : 1;  // 10
    uint16_t  MCLP_STATE         : 1;  // 11
    uint16_t  INT_COMM_SEC_FAULT : 1;  // 12
    uint16_t  INT_REG_SEC_FAULT  : 1;  // 13
    uint16_t  GM_FAULT           : 1;  // 14
    uint16_t  GM_STATE           : 1;  // 15
};

typedef union  {
    uint16_t                        all;
    struct UCC5870_STATUS3_REG_BITS bit;
} UCC5870_STATUS3_REG;

enum {
    NOT_ACTIVE = 0,
    ACTIVE     = 1
};

//
// UCC5870 STATUS3 Register's FAULT bit mask definitions
//
#define  DESAT_FAULT_MASK         (1U << 0)
#define  OC_FAULT_MASK            (1U << 1)
#define  SC_FAULT_MASK            (1U << 2)
#define  VREG2_ILIMIT_FAULT_MASK  (1U << 3)
#define  PS_TSD_FAULT_MASK        (1U << 5)
#define  VCEOV_FAULT_MASK         (1U << 6)
#define  UVLO2_FAULT_MASK         (1U << 7)
#define  OVLO2_FAULT_MASK         (1U << 8)
#define  UVLO3_FAULT_MASK         (1U << 9)
#define  OVLO3_FAULT_MASK         (1U << 10)
#define  MCLP_STATE_MASK          (1U << 11)
#define  INT_COMM_SEC_FAULT_MASK  (1U << 12)
#define  INT_REG_SEC_FAULT_MASK   (1U << 13)
#define  GM_FAULT_MASK            (1U << 14)

//
// UCC5870 STATUS4 Register bit definitions:
//
struct  UCC5870_STATUS4_REG_BITS
{                                    // bit     description
    uint16_t  SEC_RDY           : 1;  //  0
    uint16_t  rsvd              : 4;  //  4:1
    uint16_t  RIM_CRC_SEC_FAULT : 1;  //  5
    uint16_t  CFG_CRC_SEC_FAULT : 1;  //  6
    uint16_t  CLK_MON_SEC_FAULT : 1;  //  7
    uint16_t  BIST_SEC_FAULT    : 1;  //  8
    uint16_t  OR_NFLT2_SEC      : 1;  //  9
    uint16_t  OR_NFLT1_SEC      : 1;  // 10
    uint16_t  rsvd11            : 1;  // 11
    uint16_t  GD_TSD_SEC_FAULT  : 1;  // 12
    uint16_t  GD_TWN_SEC_FAULT  : 1;  // 13
    uint16_t  VCE_STATE         : 1;  // 14
    uint16_t  rsvd15            : 1;  // 15
};

typedef union  {
    uint16_t                        all;
    struct UCC5870_STATUS4_REG_BITS bit;
} UCC5870_STATUS4_REG;

//
// UCC5870 STATUS4 Register's FAULT bit mask definitions
//
#define  RIM_CRC_SEC_FAULT_MASK   (1U << 5)
#define  CFG_CRC_SEC_FAULT_MASK   (1U << 6)
#define  CLK_MON_SEC_FAULT_MASK   (1U << 7)
#define  BIST_SEC_FAULT_MASK      (1U << 8)
#define  OR_NFLT2_SEC_MASK        (1U << 9)
#define  OR_NFLT1_SEC_MASK        (1U << 10)
#define  GD_TSD_SEC_FAULT_MASK    (1U << 12)
#define  GD_TWN_SEC_FAULT_MASK    (1U << 13)

//
// UCC5870 STATUS5 Register bit definitions:
//
struct  UCC5870_STATUS5_REG_BITS
{                                // bit     description
    uint16_t rsvd      : 15;     // 14-0
    uint16_t ADC_FAULT :  1;     // 15
};

typedef union  {
    uint16_t                        all;
    struct UCC5870_STATUS5_REG_BITS bit;
} UCC5870_STATUS5_REG;

//
// UCC5870 STATUS5 Register's FAULT bit mask definitions
//
#define ADC_FAULT_MASK            (1U << 15)

//
// UCC5870 CONTROL1 Register bit definitions:
//
struct  UCC5870_CONTROL1_REG_BITS
{                                  // bit     description
    uint16_t  CLK_MON_CHK_PRI : 1; //  0
    uint16_t  rsvd4           : 4; //  4:1
    uint16_t  STP_CHK         : 1; //  5
    uint16_t  rsvd6           : 1; //  6
    uint16_t  PWM_COMP_CHK    : 1; //  7
    uint16_t  CFG_CRC_CHK_PRI : 1; //  8
    uint16_t  rsvd14          : 6; // 14-9
    uint16_t  CLR_SPI_CRC     : 1; // 15
};

typedef union  {
    uint16_t                         all;
    struct UCC5870_CONTROL1_REG_BITS bit;
} UCC5870_CONTROL1_REG;

//
// UCC5870 CONTROL2 Register bit definitions:
//
struct  UCC5870_CONTROL2_REG_BITS
{                                   // bit     description
    uint16_t  rsvd0           : 1;  //  0
    uint16_t  PS_TSD_CHK_SEC  : 1;  //  1
    uint16_t  CFG_CRC_CHK_SEC : 1;  //  2
    uint16_t  CLK_MON_CHK_SEC : 1;  //  3
    uint16_t  RESERVED4       : 1;  //  4
    uint16_t  VGTH_MEAS       : 1;  //  5
    uint16_t  rsvd6           : 1;  //  6
    uint16_t  OCP_CHK         : 1;  //  7
    uint16_t  SCP_CHK         : 1;  //  8
    uint16_t  DESAT_CHK       : 1;  //  9
    uint16_t  rsvd10          : 1;  // 10
    uint16_t  VCECLP_CHK      : 1;  // 11
    uint16_t  GATE_ON_CHK     : 1;  // 12
    uint16_t  GATE_OFF_CHK    : 1;  // 13
    uint16_t  rsvd14          : 1;  // 14
    uint16_t  CLR_STAT_REG    : 1;  // 15
};

typedef union  {
    uint16_t                         all;
    struct UCC5870_CONTROL2_REG_BITS bit;
} UCC5870_CONTROL2_REG;

//
// UCC5870 ADCCFG Register bit definitions:
//
struct  UCC5870_ADCCFG_REG_BITS
{                                     // bit     description
    uint16_t  rsvd0            : 1;   //  0
    uint16_t  ADC_ON_CH_SEL_7  : 1;   //  1
    uint16_t  ADC_ON_CH_SEL_6  : 1;   //  2
    uint16_t  ADC_ON_CH_SEL_5  : 1;   //  3
    uint16_t  ADC_ON_CH_SEL_4  : 1;   //  4
    uint16_t  ADC_ON_CH_SEL_3  : 1;   //  5
    uint16_t  ADC_ON_CH_SEL_2  : 1;   //  6
    uint16_t  ADC_ON_CH_SEL_1  : 1;   //  7
    uint16_t  rsvd7            : 1;   //  8
    uint16_t  ADC_OFF_CH_SEL_7 : 1;   //  9
    uint16_t  ADC_OFF_CH_SEL_6 : 1;   // 10
    uint16_t  ADC_OFF_CH_SEL_5 : 1;   // 11
    uint16_t  ADC_OFF_CH_SEL_4 : 1;   // 12
    uint16_t  ADC_OFF_CH_SEL_3 : 1;   // 13
    uint16_t  ADC_OFF_CH_SEL_2 : 1;   // 14
    uint16_t  ADC_OFF_CH_SEL_1 : 1;   // 15
};

typedef union  {
    uint16_t                       all;
    struct UCC5870_ADCCFG_REG_BITS bit;
} UCC5870_ADCCFG_REG;

enum {
    DONT_SAMPLE_CH = 0,
    SAMPLE_CH      = 1
};

//
// UCC5870 DOUTCFG Register bit definitions:
//
struct  UCC5870_DOUTCFG_REG_BITS
{                               // bit     description
    uint16_t  DOUT_TO_AI1 : 1;  //  0
    uint16_t  DOUT_TO_AI3 : 1;  //  1
    uint16_t  DOUT_TO_AI5 : 1;  //  2
    uint16_t  DOUT_TO_AI2 : 1;  //  3
    uint16_t  DOUT_TO_AI4 : 1;  //  4
    uint16_t  DOUT_TO_AI6 : 1;  //  5
    uint16_t  DOUT_TO_TJ  : 1;  //  6
    uint16_t  rsvd        : 1;  //  7
    uint16_t  FREQ_DOUT   : 2;  //  9:8
    uint16_t  AI6OCSC_EN  : 1;  // 10
    uint16_t  AI4OCSC_EN  : 1;  // 11
    uint16_t  AI2OCSC_EN  : 1;  // 12
    uint16_t  AI5OT_EN    : 1;  // 13
    uint16_t  AI3OT_EN    : 1;  // 14
    uint16_t  AI1OT_EN    : 1;  // 15
};

typedef union  {
    uint16_t                        all;
    struct UCC5870_DOUTCFG_REG_BITS bit;
} UCC5870_DOUTCFG_REG;

enum {
    PROT_DISABLED = 0,
    PROT_ENABLED  = 1
};

enum {
    FREQ_DOUT_13p9KHz = 0,
    FREQ_DOUT_27p8KHz = 1,
    FREQ_DOUT_55p7KHz = 2,
    FREQ_DOUT_111p4KHz = 3
};

enum {
    DONT_CONNECT_TO_DOUT = 0,
    CONNECT_TO_DOUT      = 1
};

typedef struct  {
    UCC5870_CFG1_REG          cfg1;         // 0x00
    UCC5870_CFG2_REG          cfg2;         // 0x01
    UCC5870_CFG3_REG          cfg3;         // 0x02
    UCC5870_CFG4_REG          cfg4;         // 0x03
    UCC5870_CFG5_REG          cfg5;         // 0x04
    UCC5870_CFG6_REG          cfg6;         // 0x05
    UCC5870_CFG7_REG          cfg7;         // 0x06
    UCC5870_CFG8_REG          cfg8;         // 0x07
    UCC5870_CFG9_REG          cfg9;         // 0x08
    UCC5870_CFG10_REG         cfg10;        // 0x09
    UCC5870_CFG11_REG         cfg11;        // 0x0a
    UCC5870_ADCDATA1_REG      adcdata1;     // 0x0b
    UCC5870_ADCDATA2_REG      adcdata2;     // 0x0c
    UCC5870_ADCDATA3_REG      adcdata3;     // 0x0d
    UCC5870_ADCDATA4_REG      adcdata4;     // 0x0e
    UCC5870_ADCDATA5_REG      adcdata5;     // 0x0f
    UCC5870_ADCDATA6_REG      adcdata6;     // 0x10
    UCC5870_ADCDATA7_REG      adcdata7;     // 0x11
    UCC5870_ADCDATA8_REG      adcdata8;     // 0x12
    UCC5870_CRCDATA_REG       crcdata;      // 0x13
    UCC5870_SPITEST_REG       spitest;      // 0x14
    UCC5870_GD_ADDR_REG       gd_address;   // 0x15
    UCC5870_STATUS1_REG       status1;      // 0x16
    UCC5870_STATUS2_REG       status2;      // 0x17
    UCC5870_STATUS3_REG       status3;      // 0x18
    UCC5870_STATUS4_REG       status4;      // 0x19
    UCC5870_STATUS5_REG       status5;      // 0x1a
    UCC5870_CONTROL1_REG      control1;     // 0x1b
    UCC5870_CONTROL2_REG      control2;     // 0x1c
    UCC5870_ADCCFG_REG        adccfg;       // 0x1d
    UCC5870_DOUTCFG_REG       doutcfg;      // 0x1e
} UCC5870_Vars;

static inline void initUcc5870Vars(UCC5870_Vars *ucc5870)
{
    ucc5870->cfg1.all         = 0U;
    ucc5870->cfg2.all         = 0U;
    ucc5870->cfg3.all         = 0U;
    ucc5870->cfg4.all         = 0U;
    ucc5870->cfg5.all         = 0U;
    ucc5870->cfg6.all         = 0U;
    ucc5870->cfg7.all         = 0U;
    ucc5870->cfg8.all         = 0U;
    ucc5870->cfg9.all         = 0U;
    ucc5870->cfg10.all        = 0U;
    ucc5870->cfg11.all        = 0U;
    ucc5870->adcdata1.all     = 0U;
    ucc5870->adcdata2.all     = 0U;
    ucc5870->adcdata3.all     = 0U;
    ucc5870->adcdata4.all     = 0U;
    ucc5870->adcdata5.all     = 0U;
    ucc5870->adcdata6.all     = 0U;
    ucc5870->adcdata7.all     = 0U;
    ucc5870->adcdata8.all     = 0U;
    ucc5870->crcdata.all      = 0U;
    ucc5870->spitest.all      = 0U;
    ucc5870->gd_address.all   = 0U;
    ucc5870->status1.all      = 0U;
    ucc5870->status2.all      = 0U;
    ucc5870->status3.all      = 0U;
    ucc5870->status4.all      = 0U;
    ucc5870->status5.all      = 0U;
    ucc5870->control1.all     = 0U;
    ucc5870->control2.all     = 0U;
    ucc5870->adccfg.all       = 0U;
    ucc5870->doutcfg.all      = 0U;
}

#endif
