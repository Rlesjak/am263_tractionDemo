#include <device.h>
#include <math.h>

#include "Motor_param.h"
#include "Trinv_param.h"

#include "filter.h"
#include "foc.h"
#include "Encoder.h"
#include "IPC_RPC_Comm.h"
#include "limits.h"

#include "LoopLog.h"

#define DEG120_IN_RAD 2.094395102f

void FOC_init(void);
void FOC_cal(void);
void FOC_run(void);
void FOCcal_ISR(void *handle);
__attribute__ ((section(".tcmb_code"))) void FOCrun_ISR(void *handle);
static inline void TRINV_HAL_setPwmOutput(PWMData_t *pwm);


/* Brojač izvođenja interrupta vektorskog upravljanja */
__attribute__ ((section(".tcmb_data"))) uint32_t gIsrCnt = 0;

/* Upravljanje upaljeno/ugaseno */
volatile MotorRunStop_e runMotor = MOTOR_STOP;

/* Id referentan vrijednost */
volatile float32_t IdRef     = 0.0;
/* Iq referentna vrijednost */
volatile float32_t IqRef     = 0.0;
/* Željena brzina, normalizirana na sinkronu [-1 do 1] */
volatile float32_t SpdRef  = 0.0;

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
/* Offset filter coefficient*/
float32_t K1 = 0.9980001f;      // Offset filter coefficient K1: 0.05/(T+0.05);
float32_t K2 = 0.0019999f;      // Offset filter coefficient K2: T/(T+0.05);
/* DC bus voltage filter */
__attribute__ ((section(".tcmb_data"))) Filter_t filterVdc;


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


//
// ENCODER
//
__attribute__ ((section(".tcmb_data"))) PosSpeed_Object posSpeed;
__attribute__ ((section(".tcmb_data"))) float32_t encoder_omega;


void FOC_init(void){

    //
    // Init encoder struct
    //
    posSpeed.thetaElec = 0;
    posSpeed.thetaMech = 0;
    posSpeed.directionQEP = 0;
    posSpeed.thetaRaw = 0;
    posSpeed.mechScaler = 0.0001220703125f; //1/(2048*4)
    posSpeed.polePairs = PAIRS;
    posSpeed.calAngle = 0;
    posSpeed.speedPR = 0;
    posSpeed.K1 = 1/(BASE_FREQ*T); // Skaliranje razlike kuta u brzinu
    posSpeed.K2 = 1/(1+T*2*PI*5); // Low Pass filter za mjerenje brzine
    posSpeed.K3 = 1-posSpeed.K2;
    posSpeed.speedRPMPR = 0;
    posSpeed.oldPos = 0;
    posSpeed.speedFR = 0;
    posSpeed.speedRPMFR = 0;

    //
    // Initialize the calibration module
    //
    offsetCalCounter = 0;
    offset_curU = 0;
    offset_curV = 0;
    offset_curW = 0;

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
    // Parametri motora
    //
    motor_init(&motor1);
    motor1.Ls_d = LS_D;
    motor1.Ls_q = LS_Q;
    motor1.Rs = RS;
    motor1.Phi_e = FLUX / (2.0f * PI);
    motor1.I_scale = ADC_I_SCALE_FCT;
    motor1.V_scale = ADC_V_REFRENCE_SCALE;


    //
    // Parametara regulatora brzine
    //
    motor1.pi_spd.Kp = 0.001;
    motor1.pi_spd.Ki = 0.0008;
    motor1.pi_spd.outMax = 2.0f;
    motor1.pi_spd.outMin = -motor1.pi_spd.outMax;

    //
    // Setup PI parameter for current loop
    //
    motor1.vqLimit = VQ_LIMIT;
    motor1.modulationLimitSquare = MODULATION_LIMIT * MODULATION_LIMIT;

    // Parametri regulatora D struje
    motor1.pi_id.Kp = 1;
    motor1.pi_id.Ki = 0.1;
    motor1.pi_id.outMax = RATED_VOLTAGE * motor1.vqLimit;
    motor1.pi_id.outMin = -motor1.pi_id.outMax;

    // Parametri regulatora Q struje
    motor1.pi_iq.Kp = 1;
    motor1.pi_iq.Ki = 0.1;
    motor1.pi_iq.outMax = RATED_VOLTAGE * motor1.vqLimit;
    motor1.pi_iq.outMin = -motor1.pi_iq.outMax;

    // Granice izlaza regulatora struje
    motor1.Vout_max = motor1.pi_id.outMax;
    motor1.Vff_max = 0.05f * RATED_VOLTAGE;
    motor1.Vff_min = -0.05f * RATED_VOLTAGE;

    motor1.sampleTime = 1.0f / (ISR_FREQUENCY * 1000.0f);
    motor1.outputTimeCompDelay = motor1.sampleTime -
        ADC_S_H_TIME_NS * NS_TO_S / 2.0f;
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
    // Initialize LPF for VDC
    //
    FILTER_FO_init(&filterVdc, (VDC_FLT_BW_HZ * TWO_PI),
                   (ISR_FREQUENCY * 1000.0f));

    /* Inicijalizacija PWM strukture */
    pwm1.modulationLimit = 1.0f;
    pwm1.inv_half_prd = INV_PWM_HALF_TBPRD;


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
    gLog_ptr[12] = NULL;
    gLog_ptr[13] = &encoder_omega;
    gLog_ptr[14] = NULL;
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

        gFlag_AdcCal = TRUE;
    }

    if (gFlag_AdcCal == FALSE)
    {
        ADC_clearInterruptStatus(CONFIG_ADC4_BASE_ADDR,ADC_INT_NUMBER1);
    }
}

__attribute__ ((section(".tcmb_code"))) void FOCrun_ISR(void *handle)
{
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

        motor1.pi_spd.refValue = rg1.Freq * BASE_FREQ * TWO_PI;

    }else
    {
        SpdRef = 0;
        motor1.pi_spd.refValue = 0;
    }

    /* Read 3ph current */
    // IFBW nije struja faze W, vec struja zvijezdista (Ifb-ret)
    motor1.I_abc_A[0] = (float32_t) IFBU_PPB;
    motor1.I_abc_A[1] = (float32_t) IFBV_PPB;

    // Pa se faza W dobije kao W = RET - U - V
    motor1.I_abc_A[2] = (float32_t)(IFBW_PPB - motor1.I_abc_A[0] - motor1.I_abc_A[1]);

    // Procitaj enkoder
    PosSpeed_calculate(&posSpeed, CONFIG_EQEP2_BASE_ADDR);

    /* Read DC bus voltage */
    motor1.dcBus_V = (float32_t)VDC_EVT;

    /* Process 3ph current */
    motor1.I_abc_A[0] = motor1.I_abc_A[0] * motor1.I_scale;
    motor1.I_abc_A[1] = motor1.I_abc_A[1] * motor1.I_scale;
    motor1.I_abc_A[2] = motor1.I_abc_A[2] * motor1.I_scale;

    /* Process DC bus voltage */
    motor1.dcBus_V = motor1.dcBus_V * motor1.V_scale;
    motor1.dcBus_V = FILTER_FO_run(&filterVdc, motor1.dcBus_V);
    motor1.dcBus_V = (motor1.dcBus_V > 1.0) ? motor1.dcBus_V : 1.0;
    motor1.oneOverDcBus_invV = 1.0 / motor1.dcBus_V;

    encoder_omega = posSpeed.speedPR * BASE_FREQ * TWO_PI;

    motor1.pi_spd.fbackValue = encoder_omega;
    /* run speed loop regulation */
    motor1.pi_iq.refValue = PI_run_series(&(motor1.pi_spd));

    /* Slip compensation on position feedback from resolver for TBD Induction motor
     * (to be replaced by user algorithm)
     * */
    aci1.IMDs += aci1.Kr * (motor1.I_dq_A[0] - aci1.IMDs);

    // Limitiranje na +/- 0.001, da se izbjegne djeljenje s 0
    aci1.IMDs = ((aci1.IMDs <  0.001) && (aci1.IMDs >= 0)) ? 0.001 : aci1.IMDs;
    aci1.IMDs = ((aci1.IMDs > -0.001) && (aci1.IMDs <  0)) ? -0.001 : aci1.IMDs;

    aci1.Wslip = aci1.Kt * motor1.I_dq_A[1] / aci1.IMDs;


    /* Get electrical angle for TBD Induction motor
     * (to be replaced by user algorithm)
     * */
    motor1.omega_e = encoder_omega + aci1.Wslip;
    motor1.theta_e += motor1.sampleTime * motor1.omega_e;
    theta_limiter(&(motor1.theta_e));

    // Dodaj kompenzaciju za kašnjenje koje unosi ADC
    motor1.theta_e_out = motor1.theta_e +
                         (motor1.outputTimeCompDelay * encoder_omega);
    theta_limiter(&(motor1.theta_e_out));


    /* Calculate phaser for Park transformation
     * C std lib takes 900ns in this step
     * (to be replaced by user algorithm)
     * */
    motor1.Sine = sinf(motor1.theta_e);
    motor1.Cosine = cosf(motor1.theta_e);

    /* Calculate phaser for iPark transformation
     * C std lib takes 900ns in this step
     * (to be replaced by user algorithm)
     * */
    motor1.Sine_out = sinf(motor1.theta_e_out);
    motor1.Cosine_out = cosf(motor1.theta_e_out);

    /* run transformation */
    clarke_run(&motor1);
    park_run(&motor1);

    motor1.I_dq_A[0] = -motor1.I_dq_A[0];
    motor1.I_dq_A[1] = -motor1.I_dq_A[1];

    /* Postavljanje referentne/željene vrijdnosti struje Id */
    motor1.pi_id.refValue = (runMotor == MOTOR_STOP) ? 0 : IdRef;

    /* run current loop regulation */
    // motor1.pi_id.outMax = motor1.vqLimit * motor1.dcBus_V;
    motor1.pi_id.outMax = 100.0f;
    motor1.pi_id.outMin = - motor1.pi_id.outMax;
    // motor1.pi_id.outMin = 0.0f;

    motor1.pi_iq.outMax = motor1.pi_id.outMax;
    motor1.pi_iq.outMin = motor1.pi_id.outMin;
    motor1.Vout_max = motor1.pi_id.outMax;

    motor1.pi_id.fbackValue = motor1.I_dq_A[0];
    motor1.pi_iq.fbackValue = motor1.I_dq_A[1];
    motor1.Vout_dq_V[0] = PI_run_series(&(motor1.pi_id));
    motor1.Vout_dq_V[1] = PI_run_series(&(motor1.pi_iq));

    dq_limiter_run(&motor1);

    /* run transformation */
    ipark_run(&motor1);
    /* Generate SVPWM */
    SVGEN_run(&motor1, &pwm1);
    /* Clamp duty cycle */
    PWM_clamp(&pwm1);

    /* Write ePWM */
    TRINV_HAL_setPwmOutput(&pwm1);

    gIsrCnt+=1;
    if (gIsrCnt >= ULONG_MAX - 1)
    {
        gIsrCnt = 0;
    }

    // LoopLog_run();

    ADC_clearInterruptStatus(CONFIG_ADC4_BASE_ADDR,ADC_INT_NUMBER1);

    // Postavi flag da je izvrsen jedan period u svrhu slanja mjerenja na tcp server jezgru
    raiseIPCTransmissionFlag(gIsrCnt);
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
unsigned int FOC_getMotorRunState()
{
    return runMotor;
}

void FOC_setSpeedRef(float32_t SpeedSetpoint)
{
    // Clamp na raspon od 0 do 1
    SpdRef = fminf(1.0, fmaxf(SpeedSetpoint, -1.0));
}

float32_t FOC_getSpeedRef(void)
{
    return SpdRef;
}

float32_t FOC_getMotorDCBus(void)
{
    return motor1.dcBus_V;
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

// Getteri struct handlea
Motor_t* FOC_DANGER_getMotorStructPointer()
{
    return &motor1;
}

PosSpeed_Object* FOC_DANGER_getPosSpeedStructPointer()
{
    return &posSpeed;
}
