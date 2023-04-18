//#############################################################################
//
// FILE:  trinv_param.h
//
// TITLE: header file for traction inverter
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef TRINV_PARAM_H_
#define TRINV_PARAM_H_

//
// Define the system frequency (MHz)
//
#define CONTROLSS_FREQUENCY    200.0
#define SYSTEM_FREQUENCY       CONTROLSS_FREQUENCY

//
// PWM, SAMPLING FREQUENCY and Current Loop Band width definitions (KHz)
//
#define PWM_FREQUENCY           10.0

//
// This line sets the SAMPLING FREQUENCY to one of the available choices
//
#define  SINGLE_SAMPLING        1
#define  DOUBLE_SAMPLING        2

//
// User can select choices from available control configurations
//
#define  SAMPLING_METHOD     SINGLE_SAMPLING
//#define  SAMPLING_METHOD     DOUBLE_SAMPLING

#if(SAMPLING_METHOD == SINGLE_SAMPLING)
#define ISR_FREQUENCY           (PWM_FREQUENCY)
#define ISR_RES_RATIO           0U

#elif(SAMPLING_METHOD == DOUBLE_SAMPLING)
#define ISR_FREQUENCY           (2.0 * PWM_FREQUENCY)
#define ISR_RES_RATIO           1U

#endif

//
// Keep PWM Period same between single sampling and double sampling
//
// Change to SYSTEM_FREQUENCY / 1 after changing EPWM Clock Divide Select to /1
//
#define INV_PWM_TICKS        ((SYSTEM_FREQUENCY / PWM_FREQUENCY) * 1000.0)
#define INV_PWM_DB            (200.0)
#define QEP_UNIT_TIMER_TICKS  \
    (SYSTEM_FREQUENCY / (2.0 * PWM_FREQUENCY) * 1000.0)

#define INV_PWM_TBPRD         (INV_PWM_TICKS / 2.0)
#define INV_PWM_HALF_TBPRD    (INV_PWM_TBPRD / 2.0)
#define SAMPLING_FREQ         (ISR_FREQUENCY * 1000)
#define CUR_LOOP_BANDWIDTH    (2.0 * PI * SAMPLING_FREQ / 18.0)

#define TPWM_CARRIER          (1000.0 / PWM_FREQUENCY)

//
// Define the base quantites
//
#define BASE_VOLTAGE          340.0f  // Base peak phase voltage (volt)
#define BASE_CURRENT          10.0f  // Base peak phase current (amp)
#define BASE_TORQUE                   // Base torque (N.m)
#define BASE_FLUX                     // Base flux linkage (volt.sec/rad)
#define BASE_FREQ             50.0f  // Base electrical frequency (Hz)

#define MOTOR_TEMP_LIMIT      90.0f

#define CURRENT_LIMIT         30.0f

#define VDC_FLT_BW_HZ       2.0f

#define VQ_LIMIT            1.15f
#define MODULATION_LIMIT    1.33f

//
// Current sensors scaling
//
#define AMC1302(A)     (2048 * A / BASE_CURRENT)
#define AMC1311(A)     (4096 * A / BASE_VOLTAGE)

//
// Analog scaling with ADC
//
// 1/2^12
//
#define ADC_PU_SCALE_FACTOR        0.000244140625f

// Skaliranje adc ocitanja napona za referentni napon i djelitelj napona
// (1/broj razina adc) * (referentni napon) * (djelitelj napona)
#define ADC_V_REFRENCE_SCALE       (ADC_PU_SCALE_FACTOR) * (3.22693f) * (125.0f)



//
// 1/2^11
//
#define ADC_PU_PPB_SCALE_FACTOR    0.000488281250f

#define SD_VOLTAGE_SENSE_SCALE     (SD_PU_SCALE_FACTOR * (100.0f / 0.212f))

//
// Constants for ADC sampling delay calculation
//
// Sample and hold time
//
#define ADC_S_H_TIME_NS     75.0f
//
// Conversion time
//
#define ADC_CONV_TIME_NS    175.0f
#define NS_TO_S             1.0e-9f

//
// ADC Related defines
//
#define R_REF            ADC_readResult   (CSL_CONTROLSS_ADC0_RESULT_U_BASE, ADC_SOC_NUMBER1)

#define R_EXC            ADC_readResult   (CSL_CONTROLSS_ADC4_RESULT_U_BASE, ADC_SOC_NUMBER0)
//#define R_EXC_PPB        ADC_readPPBResult(CONFIG_ADC4, ADC_PPB_NUMBER1)

#define R_SIN1       ADC_readResult(CSL_CONTROLSS_ADC4_RESULT_U_BASE, ADC_SOC_NUMBER0)
#define R_SIN2       ADC_readResult(CSL_CONTROLSS_ADC4_RESULT_U_BASE, ADC_SOC_NUMBER3)
//#define R_SIN_PPB   ADC_readPPBResult(CONFIG_ADC4, ADC_PPB_NUMBER1)

#define R_COS1       ADC_readResult(CSL_CONTROLSS_ADC4_RESULT_U_BASE, ADC_SOC_NUMBER1)
#define R_COS2       ADC_readResult(CSL_CONTROLSS_ADC4_RESULT_U_BASE, ADC_SOC_NUMBER1)
//#define R_COS_PPB   ADC_readPPBResult(CONFIG_ADC4, ADC_PPB_NUMBER2)

#define IFBU        ADC_readResult(CSL_CONTROLSS_ADC1_RESULT_U_BASE, ADC_SOC_NUMBER0)
#define IFBV        ADC_readResult(CSL_CONTROLSS_ADC2_RESULT_U_BASE, ADC_SOC_NUMBER0)

// IFBW nije struja faze W, vec struja zvijezdista (Ifb-ret)
#define IFBW        ADC_readResult(CSL_CONTROLSS_ADC3_RESULT_U_BASE, ADC_SOC_NUMBER0)
#define IFBU_PPB    ADC_readPPBResult(CSL_CONTROLSS_ADC1_RESULT_U_BASE, ADC_PPB_NUMBER1)
#define IFBV_PPB    ADC_readPPBResult(CSL_CONTROLSS_ADC2_RESULT_U_BASE, ADC_PPB_NUMBER1)

// IFBW nije struja faze W, vec struja zvijezdista (Ifb-ret)
#define IFBW_PPB    ADC_readPPBResult(CSL_CONTROLSS_ADC3_RESULT_U_BASE, ADC_PPB_NUMBER1)

#define VDC_EVT     ADC_readResult(CSL_CONTROLSS_ADC0_RESULT_U_BASE, ADC_SOC_NUMBER0)

#define TEMP_SENS   ADC_readResult(CSL_CONTROLSS_ADC1_RESULT_U_BASE, ADC_SOC_NUMBER3)

#endif /* TRINV_PARAM_H_ */
