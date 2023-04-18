//#############################################################################
//
// FILE:  foc_loop.c
//
// TITLE: source file for FOC loop
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/

//
// ALL RIGHTS RESERVED
//
//#############################################################################

#include <device.h>
#include <math.h>

#include "ucc5870.h"
#include "RDC_HAL.h"

#include "Motor_param.h"
#include "Resolver_param.h"
#include "Trinv_param.h"

#include "filter.h"
#include "foc.h"
#include "resolver.h"

#include "LoopLog.h"

//#define BENCHMARK

//
// Function prototypes
//
void FOC_init(void);
void FOC_cal(void);
void FOC_run(void);
void FOCcal_ISR(void *handle);

__attribute__ ((section(".tcmb_code"))) void FOCrun_ISR(void *handle);

static inline void TRINV_HAL_setPwmOutput(PWMData_t *pwm);

//
// Test variables
//
/* Isr Counter */
__attribute__ ((section(".tcmb_data"))) uint32_t gIsrCnt = 0;
/* Run switch */
volatile MotorRunStop_e runMotor = MOTOR_STOP;
/* Vd reference */
volatile float32_t VdTesting = 0.0;
/* Vq reference */
volatile float32_t VqTesting = 0.0;
/* Id reference */
volatile float32_t IdRef     = 0.0;
/* Iq reference */
volatile float32_t IqRef     = 0.0;
/* Speed reference */
volatile float32_t SpdRef  = 0.0;

/* Test flags */
volatile uint32_t gTFlag_MockTheta = TRUE;
volatile uint32_t gTFlag_MockVdq = FALSE;
volatile uint32_t gTFlag_MockId = TRUE;
volatile uint32_t gTFlag_MockIq = TRUE;

/* Speed loop demo */
volatile uint32_t gTFlag_SpdDemo = FALSE;
/* Speed reference */
volatile float32_t gSpeedRef  = 0.0;
volatile float32_t gDemoSpd = 60;
__attribute__ ((section(".tcmb_data"))) float32_t gDemoRPM;
/* Demo time counter */
volatile uint32_t gDemoCnt = 0;
volatile uint32_t gDemoPos = 50000;
volatile uint32_t gDemoNeg = 150000;
volatile uint32_t gDemoMax = 250000;

/* Toggle Pin*/
__attribute__ ((section(".tcmb_data"))) uint32_t    gTgpioBaseAddr, gTpinNum;

//
// Interrupt routine
//
/* Hardware interrupt instance */
static HwiP_Object gAdcHwiObject;

//
// Offset calibration routine is run to calibrate for any offsets on the opamps
//
/* Calibration routine */
volatile uint32_t gFlag_AdcCal = FALSE;
volatile uint32_t offsetCalCounter = 0;
/* Current offset */
volatile float32_t offset_curU = 0.0f;   // offset in AMC1302 current U fbk channel @ 0A
volatile float32_t offset_curV = 0.0f;   // offset in AMC1302 current V fbk channel @ 0A
volatile float32_t offset_curW = 0.0f;   // offset in AMC1302 current W fbk channel @ 0A
/* Resovler voltage */
volatile float32_t offset_Rsin = 0.0f;   // offset for resolver feedback
volatile float32_t offset_Rcos = 0.0f;   // offset for resolver feedback
/* Offset filter coefficient*/
float32_t K1 = 0.9980001f;      // Offset filter coefficient K1: 0.05/(T+0.05);
float32_t K2 = 0.0019999f;      // Offset filter coefficient K2: T/(T+0.05);
/* DC bus voltage filter */
__attribute__ ((section(".tcmb_data"))) Filter_t filterVdc;

//
// Resolver
//
__attribute__ ((section(".tcmb_data"))) Resolver_t resolver1;
__attribute__ ((section(".tcmb_data"))) float32_t resolver_bias[2];
__attribute__ ((section(".tcmb_data"))) float32_t resolver_omega;
__attribute__ ((section(".tcmb_data"))) float32_t resolver_theta;
__attribute__ ((section(".tcmb_data"))) float32_t resolver_motor_ratio;

__attribute__ ((section(".tcmb_data"))) uint16_t gFlag_RDCexcUpdate = FALSE;
__attribute__ ((section(".tcmb_data"))) uint16_t gFlag_RDCexcRight = FALSE;
__attribute__ ((section(".tcmb_data"))) uint16_t gFlag_RDCexcLeft = FALSE;
__attribute__ ((section(".tcmb_data"))) uint16_t *gRDCtable_ptr = &resolverExecTable[19];

//
// Field Oriented Control
//
/* Samping period (sec), see parameter.h */
float32_t T = 0.001 / ISR_FREQUENCY;
/* Instance a ramp controller to smoothly ramp the frequency */
__attribute__ ((section(".tcmb_data"))) RMPCNTL rc1;
/* Instance a ramp generator */
__attribute__ ((section(".tcmb_data"))) RAMPGEN rg1;
/* Variables for Motor control */
__attribute__ ((section(".tcmb_data"))) Motor_t motor1;
/* Variables for PWM update */
__attribute__ ((section(".tcmb_data"))) PWMData_t pwm1;
/* Variables for induction machine current model */
__attribute__ ((section(".tcmb_data"))) ACI_Model_t aci1;


void FOC_init(void){

    //
    // Initialize the calibration module
    //
    offsetCalCounter = 0;
    offset_curU = 0;
    offset_curV = 0;
    offset_curW = 0;
    offset_Rsin = 0;
    offset_Rcos = 0;

    //
    // Initialize the RAMPGEN module
    //
    initRampGen(&rg1);
    rg1.StepAngleMax = BASE_FREQ * T;
    rg1.Angle = 0.0f;
    rg1.Out = 0.0f;
    rg1.Gain = 1.0f;
    rg1.Offset = 1.0f;

    //
    // Initialize the RAMP control module
    //
    initRampControl(&rc1);

    //
    // set mock REFERENCES for Speed and current loops
    //
    gSpeedRef = 0.0f;
    SpdRef    = 1.0f;
    IdRef     = 0.01f;
    IqRef     = 0.1f;

    //
    // Initialize the motor structure
    //
    motor_init(&motor1);

    motor1.Ls_d = LS_D;
    motor1.Ls_q = LS_Q;
    motor1.Rs = RS;
    motor1.Phi_e = FLUX / (2.0f * PI);

    motor1.I_scale = ADC_PU_PPB_SCALE_FACTOR * BASE_CURRENT;

    /*
     * Original kod
     * motor1.V_scale = ADC_PU_SCALE_FACTOR * 225.0f * (3.0f / 1.0f);
     */
    motor1.V_scale = ADC_V_REFRENCE_SCALE;

    pwm1.modulationLimit = 1.0f;
    pwm1.inv_half_prd = INV_PWM_HALF_TBPRD;

    motor1.pi_spd.Kp = 1.00;
    motor1.pi_spd.Ki = 0.01;
    motor1.pi_spd.outMax = 20;
    motor1.pi_spd.outMin = -motor1.pi_spd.outMax;

    //
    // Setup PI parameter for current loop
    //
    motor1.vqLimit = VQ_LIMIT;
    motor1.modulationLimitSquare = MODULATION_LIMIT * MODULATION_LIMIT;

    /* Please modify according to motor and inverter parameters */
//    motor1.pi_id.Kp = motor1.Ls_d * CUR_LOOP_BW;
//    motor1.pi_id.Ki = (motor1.Rs / motor1.Ls_d) / (ISR_FREQUENCY * 1000.0f);
    motor1.pi_id.Kp = 1;
    motor1.pi_id.Ki = 0.1;
    motor1.pi_id.outMax = RATED_VOLTAGE * motor1.vqLimit;
    motor1.pi_id.outMin = -motor1.pi_id.outMax;

    /* Please modify according to motor and inverter parameters */
//    motor1.pi_iq.Kp = motor1.Ls_q * CUR_LOOP_BW;
//    motor1.pi_iq.Ki = (motor1.Rs / motor1.Ls_q) / (ISR_FREQUENCY * 1000.0f);
    motor1.pi_iq.Kp = 1;
    motor1.pi_iq.Ki = 0.1;
    motor1.pi_iq.outMax = RATED_VOLTAGE * motor1.vqLimit;
    motor1.pi_iq.outMin = -motor1.pi_iq.outMax;

    motor1.Vout_max = motor1.pi_id.outMax;

    motor1.Vff_max = 0.05f * RATED_VOLTAGE;
    motor1.Vff_min = -0.05f * RATED_VOLTAGE;

    motor1.sampleTime = 1.0f / (ISR_FREQUENCY * 1000.0f);
    motor1.outputTimeCompDelay = motor1.sampleTime -
        ADC_S_H_TIME_NS * NS_TO_S / 2.0f;
    /* Please modify according to the interface between control card and resovler front end */
//    motor1.resolverCompDelay = motor1.sampleTime -
//        (ADC_S_H_TIME_NS / 2.0f + ADC_CONV_TIME_NS + (ADC_S_H_TIME_NS * 4.0f + ADC_CONV_TIME_NS * 3.0f) / 2.0f) * NS_TO_S;
    motor1.resolverCompDelay = motor1.sampleTime -
        ((ADC_S_H_TIME_NS * 4.0f + ADC_CONV_TIME_NS * 4.0f) / 2.0f) * NS_TO_S;
    motor1.resolverCompIdx = 0U;
    motor1.isrResRatio = ISR_RES_RATIO;

    //
    // Initialize Induction Machine Current Model
    //
    aci1.Kt = 10;
    aci1.Kr = 0.1;
    aci1.IMDs = 0;
    aci1.Wslip = 0;

    //
    // Initialize resolver object
    //
    resolver_init(&resolver1);
    resolver1.resolver_theta = &resolver_theta;
    resolver1.resolver_omega = &resolver_omega;
    resolver1.sample_time = 1.0f / (RESOLVER_EXC_FREQUENCY * 1000.0f);
//    resolver1.pll_gain_in = RESOLVER_OMEGA * RESOLVER_OMEGA;
    resolver1.pll_gain_in = 1500000.0f;
//    resolver1.pll_gain_ff = 2.0f * RESOLVER_ZETA / RESOLVER_OMEGA;
    resolver1.pll_gain_ff = 0.005f;
    resolver1.bias = RESOLVER_BIAS;
    resolver1.phase_comp_gain = RESOLVER_PHASE_COMP_GAIN;
    resolver1.theta_ratio = 2; // elec
    FILTER_FO_init(&resolver1.lpf_spd, RESOLVER_LPF_SPD_BW,
                   (RESOLVER_EXC_FREQUENCY * 1000.0f));

    //
    // Initialize LPF for VDC
    //
    FILTER_FO_init(&filterVdc, (VDC_FLT_BW_HZ * TWO_PI),
                   (ISR_FREQUENCY * 1000.0f));

    //
    // Init FLAGS
    //
    runMotor = MOTOR_STOP;

    gLog_ptr[0] = &motor1.omega_e;
    gLog_ptr[1] = &motor1.theta_e;
    gLog_ptr[2] = &motor1.Sine;
    gLog_ptr[3] = &motor1.Cosine;
    gLog_ptr[4] = &pwm1.Vabc_pu[0];
    gLog_ptr[5] = &pwm1.Vabc_pu[1];
    gLog_ptr[6] = &pwm1.Vabc_pu[2];
    gLog_ptr[7] = &motor1.I_abc_A[0];
    gLog_ptr[8] = &motor1.I_abc_A[1];
    gLog_ptr[9] = &motor1.I_abc_A[2];
    gLog_ptr[10] = &motor1.I_dq_A[0];
    gLog_ptr[11] = &motor1.I_dq_A[1];
    gLog_ptr[12] = &resolver_theta;
    gLog_ptr[13] = &resolver_omega;
    gLog_ptr[14] = &gDemoRPM;
    gLog_ptr[15] = NULL;

    LoopLog_init();
}

void FOC_cal(void){

    HwiP_Params hwiPrms;
    int32_t status;

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback = &FOCcal_ISR;
    hwiPrms.priority = 1U;
    hwiPrms.isPulse = 1U;
    status = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(SystemP_SUCCESS == status);

    ADC_clearInterruptStatus(CONFIG_ADC4_BASE_ADDR,ADC_INT_NUMBER1);

    while(gFlag_AdcCal == FALSE)
    {
        ClockP_usleep(200000L);
    }

    HwiP_destruct(&gAdcHwiObject);

}

void FOC_run(void){

    HwiP_Params hwiPrms;
    int32_t status;

    /* Register & enable interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    hwiPrms.callback = &FOCrun_ISR;
    hwiPrms.priority = 1U;
    hwiPrms.isPulse = 1U;
    status = HwiP_construct(&gAdcHwiObject, &hwiPrms);
    DebugP_assert(SystemP_SUCCESS == status);

    gTgpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_GPIO_TEST_TOGGLE_BASE_ADDR);
    gTpinNum       = CONFIG_GPIO_TEST_TOGGLE_PIN;
    GPIO_setDirMode(gTgpioBaseAddr, gTpinNum, CONFIG_GPIO_TEST_TOGGLE_DIR);
    GPIO_pinWriteLow(gTgpioBaseAddr, gTpinNum);

    ADC_clearInterruptStatus(CONFIG_ADC4_BASE_ADDR,ADC_INT_NUMBER1);
}

void FOCcal_ISR(void *handle)
{

    //
    // Feedbacks OFFSET Calibration Routine
    //
    if(offsetCalCounter < 22000)
    {
        if(offsetCalCounter > 2000)
        {
            //
            // Offsets in phase current sensing U, V and W from ADC
            //
            offset_curU = K1 * offset_curU + K2 * (IFBU) * ADC_PU_SCALE_FACTOR;
            offset_curV = K1 * offset_curV + K2 * (IFBV) * ADC_PU_SCALE_FACTOR;
            offset_curW = K1 * offset_curW + K2 * (IFBW) * ADC_PU_SCALE_FACTOR;

            //
            // Offsets in phase current sensing from Resolver
            //
            offset_Rsin = K1 * offset_Rsin + K2 * (R_SIN1+R_SIN2)*0.5 * ADC_PU_SCALE_FACTOR;
            offset_Rcos = K1 * offset_Rcos + K2 * (R_COS1+R_COS2)*0.5 * ADC_PU_SCALE_FACTOR;
        }
        offsetCalCounter++;
    }else{
        //
        // Init OFFSET regs with identified values
        //

        //
        // setting phase current Iu offset
        //
        ADC_setPPBReferenceOffset(CONFIG_ADC1_BASE_ADDR, ADC_PPB_NUMBER1,
                                  (uint16_t)(offset_curU * 4096.0f));
        //
        // setting phase current Iv offset
        //
        ADC_setPPBReferenceOffset(CONFIG_ADC2_BASE_ADDR, ADC_PPB_NUMBER1,
                                  (uint16_t)(offset_curV * 4096.0f));
        //
        // setting phase current Iw offset
        //
        ADC_setPPBReferenceOffset(CONFIG_ADC3_BASE_ADDR, ADC_PPB_NUMBER1,
                                  (uint16_t)(offset_curW * 4096.0f));

        resolver_bias[0] = offset_Rsin * 4096.0f;
        resolver_bias[1] = offset_Rcos * 4096.0f;

        CacheP_wb((void *)resolverExecTable, resolverExecTable_size*2, CacheP_TYPE_ALL);
        RDCexc_start(gRDCtable_ptr,20,gEdmaHandle[0],DMA_TRIG_XBAR_EDMA_MODULE_0,CONFIG_DAC0_BASE_ADDR);
        gFlag_RDCexcUpdate = TRUE;

        gFlag_AdcCal = TRUE;
    }

    if (gFlag_AdcCal == FALSE)
    {
        ADC_clearInterruptStatus(CONFIG_ADC4_BASE_ADDR,ADC_INT_NUMBER1);
    }
}

__attribute__ ((section(".tcmb_code"))) void FOCrun_ISR(void *handle)
{
    GPIO_pinWriteHigh(gTgpioBaseAddr, gTpinNum);

    if(runMotor == MOTOR_RUN)
    {
        /* Connect inputs of the RMP module and call the ramp control module
         * Convert to elec spd for POLEs = 8
         * */
        rc1.TargetValue = SpdRef;
        rampControl(&rc1);

        /*  Connect inputs of the RAMP GEN module and call the ramp generator module  */
        rg1.Freq = rc1.SetpointValue;
        rampGen(&rg1);

        /* Overwrite speed reference for speed loop demo
         * Convert to elec spd for POLEs = 8
         * */
        if (gTFlag_SpdDemo == TRUE)
        {
            gDemoCnt++;
            if(gDemoCnt < gDemoPos )
            {
                gSpeedRef = 0;
            }else if(gDemoCnt < gDemoNeg)
            {
                gSpeedRef = gDemoSpd;
            }else if(gDemoCnt < gDemoMax)
            {
                gSpeedRef = -gDemoSpd;
            }
            else
            {
                gDemoCnt = 0;
            }
            /* mech rpm to elec rad/s */
            motor1.pi_spd.refValue = gSpeedRef / 60 * PAIRS * TWO_PI;

        }else
        {
            gDemoCnt = 0;
            gSpeedRef = 0;
            motor1.pi_spd.refValue = 0;
        }

    }else
    {
        SpdRef = 0;
    }

#ifdef BENCHMARK
    GPIO_pinWriteLow(gTgpioBaseAddr, gTpinNum);
#endif
    /* Read 3ph current */

    // IFBW nije struja faze W, vec struja zvijezdista (Ifb-ret)
    motor1.I_abc_A[0] = (float32_t)IFBU_PPB;
    motor1.I_abc_A[1] = (float32_t)IFBV_PPB;

    // Pa se faza W dobije kao W = RET - U - V
    motor1.I_abc_A[2] = (float32_t)(IFBW_PPB - IFBU_PPB - IFBV_PPB);



    /* read resolver sin/cos */
    resolver1.sin_samples[0] = (float32_t)R_SIN1;
    resolver1.sin_samples[1] = (float32_t)R_SIN2;
    resolver1.cos_samples[0] = (float32_t)R_COS1;
    resolver1.cos_samples[1] = (float32_t)R_COS2;
    /* Read DC bus voltage */
    motor1.dcBus_V = (float32_t)VDC_EVT;
#ifdef BENCHMARK
    GPIO_pinWriteHigh(gTgpioBaseAddr, gTpinNum);
#endif

    /* Process 3ph current */
    motor1.I_abc_A[0] = motor1.I_abc_A[0] * motor1.I_scale;
    motor1.I_abc_A[1] = motor1.I_abc_A[1] * motor1.I_scale;
    motor1.I_abc_A[2] = motor1.I_abc_A[2] * motor1.I_scale;


    /* Process resolver sin/cos */
    resolver1.sin_os = ((resolver1.sin_samples[0]+resolver1.sin_samples[1])*0.5
                       - resolver_bias[0]) * ADC_PU_SCALE_FACTOR;
    resolver1.cos_os = ((resolver1.cos_samples[0]+resolver1.cos_samples[1])*0.5
                       - resolver_bias[1]) * ADC_PU_SCALE_FACTOR;


    /* Process DC bus voltage */
    motor1.dcBus_V = motor1.dcBus_V * motor1.V_scale;
    motor1.dcBus_V = FILTER_FO_run(&filterVdc, motor1.dcBus_V);
    motor1.dcBus_V = (motor1.dcBus_V > 1.0) ? motor1.dcBus_V : 1.0;

    /* Overwrite DC bus voltage for low voltage code bring-up */
    // motor1.dcBus_V = 12;
    motor1.oneOverDcBus_invV = 1.0 / motor1.dcBus_V;

    /* Adjust excitation phase if needed in future
     * (to be replaced by user algorithm)
     */
    if(gFlag_RDCexcUpdate == TRUE)
    {
        if(gFlag_RDCexcRight == TRUE)
        {
            gRDCtable_ptr ++;
            gFlag_RDCexcRight = FALSE;
        }

        if (gFlag_RDCexcLeft == TRUE)
        {
            gRDCtable_ptr --;
            gFlag_RDCexcLeft = FALSE;
        }

        RDCexc_update(gRDCtable_ptr,20,gEdmaHandle[0],DMA_TRIG_XBAR_EDMA_MODULE_0,CONFIG_DAC0_BASE_ADDR);
        gFlag_RDCexcUpdate = FALSE;
    }

#ifdef BENCHMARK
    GPIO_pinWriteLow(gTgpioBaseAddr, gTpinNum);
#endif
    /* Calculate sin/cos for resolver pllLoopError
     * C std lib takes 900ns in this step
     * (to be replaced by user algorithm)
     * */
    resolver1.res_theta0_sin = sinf(resolver1.res_theta0);
    resolver1.res_theta0_cos = cosf(resolver1.res_theta0);
#ifdef BENCHMARK
    GPIO_pinWriteHigh(gTgpioBaseAddr, gTpinNum);
#endif

    // compute motor omega and theta in elec
    resolver_run(&resolver1);

    /* Converter resolver omega to motor omega */
    resolver_omega *= resolver1.theta_ratio;

    gDemoRPM = resolver_omega * 60 / PAIRS / TWO_PI;
    motor1.pi_spd.fbackValue = resolver_omega;
    /* run speed loop regulation */
    motor1.pi_iq.refValue = PI_run_series(&(motor1.pi_spd));

    /* Slip compensation on position feedback from resolver for TBD Induction motor
     * (to be replaced by user algorithm)
     * */
    aci1.IMDs += aci1.Kr * (motor1.I_dq_A[0] - aci1.IMDs);

    aci1.IMDs = ((aci1.IMDs <  0.001) && (aci1.IMDs >= 0)) ? 0.001 : aci1.IMDs;
    aci1.IMDs = ((aci1.IMDs > -0.001) && (aci1.IMDs <  0)) ? -0.001 : aci1.IMDs;

    aci1.Wslip = aci1.Kt * motor1.I_dq_A[1] / aci1.IMDs;

    /* Get electrical angle for TBD Induction motor
     * (to be replaced by user algorithm)
     * */
    motor1.omega_e = resolver_omega + aci1.Wslip;
    motor1.theta_e += motor1.sampleTime * motor1.omega_e;
    theta_limiter(&(motor1.theta_e));

    /* Output time delay compensation
     * (to be replaced by user algorithm)
     * */
    motor1.theta_e_out = motor1.theta_e +
                         (motor1.outputTimeCompDelay * resolver_omega);
    theta_limiter(&(motor1.theta_e_out));

    /* Overwrite speed and position information with rampGen
     * if mock omega and theta needed
     * */
    if(gTFlag_MockTheta == TRUE)
    {
        /* mock theta_e and omega_e */
        motor1.omega_e = rg1.Freq * BASE_FREQ * TWO_PI;

        motor1.theta_e = rg1.Out * TWO_PI;
        theta_limiter(&(motor1.theta_e));

        motor1.theta_e_out = motor1.theta_e +
                             (motor1.outputTimeCompDelay * motor1.omega_e);
        theta_limiter(&(motor1.theta_e_out));

        aci1.IMDs = 0;
        aci1.Wslip = 0;
    }

#ifdef BENCHMARK
    GPIO_pinWriteLow(gTgpioBaseAddr, gTpinNum);
#endif
    /* Calculate phaser for Park transformation
     * C std lib takes 900ns in this step
     * (to be replaced by user algorithm)
     * */
    motor1.Sine = sinf(motor1.theta_e);
    motor1.Cosine = cosf(motor1.theta_e);
#ifdef BENCHMARK
    GPIO_pinWriteHigh(gTgpioBaseAddr, gTpinNum);
#endif

#ifdef BENCHMARK
    GPIO_pinWriteLow(gTgpioBaseAddr, gTpinNum);
#endif
    /* Calculate phaser for iPark transformation
     * C std lib takes 900ns in this step
     * (to be replaced by user algorithm)
     * */
    motor1.Sine_out = sinf(motor1.theta_e_out);
    motor1.Cosine_out = cosf(motor1.theta_e_out);
#ifdef BENCHMARK
    GPIO_pinWriteHigh(gTgpioBaseAddr, gTpinNum);
#endif

    /* run transformation */
    clarke_run(&motor1);
    park_run(&motor1);

    /* Ignore Iq from speed loop if not in speed loop demo
     * */
    if (gTFlag_SpdDemo == FALSE)
    {
        /* setup idref  */
        motor1.pi_id.refValue = (runMotor == MOTOR_STOP) ? 0 : IdRef;
        /* setup iqref  */
        motor1.pi_iq.refValue = (runMotor == MOTOR_STOP) ? 0 : IqRef;
    }

    /* Overwrite Iq information
     * if mock values needed
     * */

    if(gTFlag_MockId == TRUE)
    {
        /* setup idref  */
        motor1.pi_id.refValue = (runMotor == MOTOR_STOP) ? 0 : IdRef;
    }

    if(gTFlag_MockIq == TRUE)
    {
        /*  setup iqref  */
        motor1.pi_iq.refValue = (runMotor == MOTOR_STOP) ? 0 : IqRef;
    }

    /* run current loop regulation */
    motor1.pi_id.outMax = motor1.vqLimit * motor1.dcBus_V;
    motor1.pi_id.outMin = - motor1.pi_id.outMax;
    motor1.pi_iq.outMax = motor1.pi_id.outMax;
    motor1.pi_iq.outMin = motor1.pi_id.outMin;
    motor1.Vout_max = motor1.pi_id.outMax;

    motor1.pi_id.fbackValue = motor1.I_dq_A[0];
    motor1.pi_iq.fbackValue = motor1.I_dq_A[1];
    motor1.Vout_dq_V[0] = PI_run_series(&(motor1.pi_id));
    motor1.Vout_dq_V[1] = PI_run_series(&(motor1.pi_iq));

    /* Overwrite Vd Vq information
     * if mock values needed
     * */
    if(gTFlag_MockVdq == TRUE)
    {
        motor1.Vout_dq_V[0] = VdTesting;
        motor1.Vout_dq_V[1] = VqTesting;
    }

    dq_limiter_run(&motor1);

    /* run transformation */
    ipark_run(&motor1);
    /* Generate SVPWM */
    SVGEN_run(&motor1, &pwm1);
    /* Clamp duty cycle */
    PWM_clamp(&pwm1);

#ifdef BENCHMARK
    GPIO_pinWriteLow(gTgpioBaseAddr, gTpinNum);
#endif
    /* Write ePWM */
     TRINV_HAL_setPwmOutput(&pwm1);

//    EPWM_setCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_A,
//                            (uint16_t)((&pwm1->inv_half_prd * &pwm1->Vabc_pu[0]) +
//                                    &pwm1->inv_half_prd));
//        EPWM_setCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_A,
//                            (uint16_t)((&pwm1->inv_half_prd * &pwm1->Vabc_pu[1]) +
//                                    &pwm1->inv_half_prd));
//        EPWM_setCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_A,
//                            (uint16_t)((&pwm1->inv_half_prd * &pwm1->Vabc_pu[2]) +
//                                    &pwm1->inv_half_prd));

#ifdef BENCHMARK
    GPIO_pinWriteHigh(gTgpioBaseAddr, gTpinNum);
#endif

    GPIO_pinWriteLow(gTgpioBaseAddr, gTpinNum);

    gIsrCnt+=1;
    if (gIsrCnt > 9999)
    {
        gIsrCnt = 0;
    }

    LoopLog_run();

    ADC_clearInterruptStatus(CONFIG_ADC4_BASE_ADDR,ADC_INT_NUMBER1);

}

//
//  Computed Duty and Write to CMPA register
//
static inline void TRINV_HAL_setPwmOutput(PWMData_t *pwm)
{
    EPWM_setCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_A,
                        (uint16_t)((pwm->inv_half_prd * pwm->Vabc_pu[0]) +
                                    pwm->inv_half_prd));
    EPWM_setCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_A,
                        (uint16_t)((pwm->inv_half_prd * pwm->Vabc_pu[1]) +
                                    pwm->inv_half_prd));
    EPWM_setCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_A,
                        (uint16_t)((pwm->inv_half_prd * pwm->Vabc_pu[2]) +
                                    pwm->inv_half_prd));
}


// --Interface metode
void FOC_setMotorRunState(MotorRunStop_e state)
{
    if (state == MOTOR_RUN)
    {
        runMotor = MOTOR_RUN;
        return;
    }
    runMotor = MOTOR_STOP;
}
void FOC_setVd(float32_t Vd)
{
    // Clamp na raspon od 0 do 20
    VdTesting = fminf(RATED_VOLTAGE, fmaxf(Vd, 0.0));
}
void FOC_setVq(float32_t Vq)
{
    // Clamp na raspon od 0 do 20
    VqTesting = fminf(RATED_VOLTAGE, fmaxf(Vq, 0.0));
}
void FOC_setSpeedRef(float32_t SpeedSetpoint)
{
    // Clamp na raspon od 0 do 1
    SpdRef = fminf(1.0, fmaxf(SpeedSetpoint, 0.0));
}

float32_t FOC_getSpeedRef(void)
{
    return SpdRef;
}

float32_t FOC_getVd(void)
{
    return VdTesting;
}
float32_t FOC_getVq(void)
{
    return VqTesting;
}
float32_t FOC_getMotorDCBus(void)
{
    return motor1.dcBus_V;
}
Motor_t* FOC_DANGER_getMotorStructPointer()
{
    return &motor1;
}

float32_t FOC_getIdref(void)
{
    return IdRef;
}
float32_t FOC_getIqref(void)
{
    return IqRef;
}

void FOC_setIdref(float32_t value)
{
    IdRef = fminf(1.0, fmaxf(value, 0.0));
}
void FOC_setIqref(float32_t value)
{
    IqRef = fminf(1.0, fmaxf(value, 0.0));
}
