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


// Frekvencija takta procesora koja je odabrana u SysConfig (MHz)
#define CONTROLSS_FREQUENCY    200.0
#define SYSTEM_FREQUENCY       CONTROLSS_FREQUENCY

// Frekvencija uzorkovanja i PWM signala (KHz)
#define PWM_FREQUENCY           10.0
#define ISR_FREQUENCY           (PWM_FREQUENCY)
#define ISR_RES_RATIO           0U

//...
// Konfiguracija PWM modula
#define INV_PWM_TICKS        ((SYSTEM_FREQUENCY / PWM_FREQUENCY) * 1000.0)
#define INV_PWM_DB            (200.0)
#define QEP_UNIT_TIMER_TICKS  \
    (SYSTEM_FREQUENCY / (2.0 * PWM_FREQUENCY) * 1000.0)

#define INV_PWM_TBPRD         (INV_PWM_TICKS / 2.0)
#define INV_PWM_HALF_TBPRD    (INV_PWM_TBPRD / 2.0)
#define SAMPLING_FREQ         (ISR_FREQUENCY * 1000)
#define CUR_LOOP_BANDWIDTH    (2.0 * PI * SAMPLING_FREQ / 18.0)

#define TPWM_CARRIER          (1000.0 / PWM_FREQUENCY)
//...

// Bazne vrijednosti po kojima se normaliziraju veličine
#define BASE_VOLTAGE          340.0f  // (volt)
#define BASE_CURRENT          10.0f   // (amp)
#define BASE_FREQ             50.0f   // (Hz)

#define VQ_LIMIT            1.15f
#define MODULATION_LIMIT    1.33f

// Granična frekvencija NP mjerenog DC napona (Hz)
#define VDC_FLT_BW_HZ       2.0f


// 1/2^12
#define ADC_PU_SCALE_FACTOR        0.000244140625f

// Skaliranje adc ocitanja napona za referentni napon i djelitelj napona
// (1/rezoucija adc) * (referentni napon) * (djelitelj napona)
#define ADC_V_REFRENCE_SCALE       (ADC_PU_SCALE_FACTOR) * (3.22693f) * (125.0f)



// 1/2^11
#define ADC_PU_PPB_SCALE_FACTOR    0.000488281250f

// Skaliranje adc ocitanja struje za vrijednost shunta i pojacanje diff pojacala
// (referentni napon) / ((pojacanje diff pojacala) * (vrijednost shunta) * (rezolucija adc))
#define ADC_I_SCALE_FCT 0.0012605195f

// Trajanje uzimanja uzorka za ADC (Sample and Hold time)
#define ADC_S_H_TIME_NS     75.0f
// Trajanje konverzije ADC-a
#define ADC_CONV_TIME_NS    175.0f
// NanoSekunde u Sekunde
#define NS_TO_S             1.0e-9f



//
// ADC Related defines
//
#define R_REF            ADC_readResult   (CSL_CONTROLSS_ADC0_RESULT_U_BASE, ADC_SOC_NUMBER1)

#define R_EXC            ADC_readResult   (CSL_CONTROLSS_ADC4_RESULT_U_BASE, ADC_SOC_NUMBER0)

#define IFBU        ADC_readResult(CSL_CONTROLSS_ADC1_RESULT_U_BASE, ADC_SOC_NUMBER0)
#define IFBV        ADC_readResult(CSL_CONTROLSS_ADC2_RESULT_U_BASE, ADC_SOC_NUMBER0)
// IFBRET nije struja faze W, vec struja zvijezdista (Ifb-ret)
#define IFBRET        ADC_readResult(CSL_CONTROLSS_ADC3_RESULT_U_BASE, ADC_SOC_NUMBER0)

#define IFBU_PPB    ADC_readPPBResult(CSL_CONTROLSS_ADC1_RESULT_U_BASE, ADC_PPB_NUMBER1)
#define IFBV_PPB    ADC_readPPBResult(CSL_CONTROLSS_ADC2_RESULT_U_BASE, ADC_PPB_NUMBER1)
// IFBRET nije struja faze W, vec struja zvijezdista (Ifb-ret)
#define IFBRET_PPB    ADC_readPPBResult(CSL_CONTROLSS_ADC3_RESULT_U_BASE, ADC_PPB_NUMBER1)

#define VDC_EVT     ADC_readResult(CSL_CONTROLSS_ADC0_RESULT_U_BASE, ADC_SOC_NUMBER0)

#define TEMP_SENS   ADC_readResult(CSL_CONTROLSS_ADC1_RESULT_U_BASE, ADC_SOC_NUMBER3)

#endif /* TRINV_PARAM_H_ */
