//#############################################################################
//
// FILE:  foc.h
//
// TITLE: header file for FOC control library
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef FOC_H
#define FOC_H

#include "math.h"

#ifndef PI
#define PI 3.14159265358979f
#endif
#ifndef TWO_PI
#define TWO_PI 6.2831853071795862f
#endif

#define ONE_BY_SQRT3  0.57735026918963f
#define ONE_BY_3      0.33333333333333f

#define SVGEN_SQRT3_OVER_2     0.8660254038f

//
// Math functions
//
static inline float32_t
MATH_abs(float32_t x)
{
    if (x < 0)
        return -x;
    return x;
}

static inline float32_t
MATH_max(float32_t x, float32_t y)
{
    if (x > y)
        return x;
    return y;
}

static inline float32_t
MATH_min(float32_t x, float32_t y)
{
    if (x < y)
        return x;
    return y;
}

static inline float32_t
MATH_sat(const float32_t in, const float32_t max, const float32_t min)
{
    float32_t out = in;

    out = MATH_max(out, min);
    out = MATH_min(out, max);

    return(out);
}

//*****************************************************************************
//
//! \brief Defines ramp acceleration control module
//
//*****************************************************************************

typedef struct {
//
// Input: Target input (pu)
//
    float32_t TargetValue;
//
// Parameter: Maximum delay rate (Q0) - independently with global Q
//
    uint32_t  RampDelayMax;
//
// Parameter: Minimum limit (pu)
//
    float32_t RampLowLimit;
//
// Parameter: Maximum limit (pu)
//
    float32_t RampHighLimit;
//
// Variable: Incremental delay (Q0) - independently with global Q
//
    uint32_t  RampDelayCount;
//
// Output: Target output (pu)
//
    float32_t SetpointValue;
//
// Output: Flag output (Q0) - independently with global Q
//
    uint32_t  EqualFlag;
//
// Variable: Temp variable
//
    float32_t Tmp;
} RMPCNTL;

//
// initalizer for the RMPCNTL object.
//
static inline void initRampControl(RMPCNTL * rc1)
{
    rc1->TargetValue = 0.0f;
    rc1->RampDelayMax = 1U;
    rc1->RampLowLimit = -1.0f;
    rc1->RampHighLimit = 1.0f;
    rc1->RampDelayCount = 0U;
    rc1->SetpointValue = 0.0f;
    rc1->EqualFlag = 0U;
    rc1->Tmp = 0.0f;
}

//
// RAMP Controller Macro Definition
//
static inline void rampControl(RMPCNTL * rc1)
{
    rc1->Tmp = (rc1->TargetValue) - (rc1->SetpointValue);

    if(MATH_abs(rc1->Tmp) >= 0.00001525f)
    {
        rc1->RampDelayCount++;

        if((rc1->RampDelayCount) >= (rc1->RampDelayMax))
        {
            if(rc1->TargetValue >= rc1->SetpointValue)
            {
                rc1->SetpointValue += 0.00001525f;
            }
            else
            {
                rc1->SetpointValue -= 0.00001525f;
            }

            rc1->RampDelayCount = 0U;
        }
    }
    else
    {
        rc1->SetpointValue = rc1->TargetValue;
        rc1->EqualFlag = 0x7FFFFFFF;
    }

    rc1->SetpointValue = MATH_max((MATH_min(rc1->SetpointValue,
                                            rc1->RampHighLimit)),
                                   rc1->RampLowLimit);
    return;
}

//*****************************************************************************
//
//! \brief Defines the ramp generation module
//
//*****************************************************************************

typedef struct
{
//
// Input: Ramp frequency (pu)
//
    float32_t  Freq;
//
// Parameter: Maximum step angle (pu)
//
    float32_t  StepAngleMax;
//
// Variable: Step angle (pu)
//
    float32_t  Angle;
//
// Input: Ramp gain (pu)
//
    float32_t  Gain;
//
// Output: Ramp signal (pu)
//
    float32_t  Out;
//
// Input: Ramp offset (pu)
//
    float32_t  Offset;
} RAMPGEN;

//
// Object Initializers
//
static inline void initRampGen(RAMPGEN * in)
{
    in->Freq = 0.0f;
    in->StepAngleMax = 0.0f;
    in->Angle = 0.0f;
    in->Gain = 1.0f;
    in->Out = 0.0f;
    in->Offset = 1.0f;
}

//
// RAMP(Sawtooh) Generator Macro Definition
//
static inline void rampGen(RAMPGEN * in)
{
    //
    // Compute the angle rate
    //
    in->Angle += in->StepAngleMax * in->Freq;

    //
    // Saturate the angle rate within (-1,1)
    //
    if(in->Angle > 1.0f)
    {
        in->Angle -= 1.0f;
    }
    else if(in->Angle < -1.0f)
    {
        in->Angle += 1.0f;
    }

    in->Out = in->Angle;
}

//*****************************************************************************
//
//! \brief Defines the PI controller object
//
//*****************************************************************************
typedef struct _PI_Obj_
{
    float32_t Kp;              //!< the proportional gain for the PI controller
    float32_t Ki;              //!< the integral gain for the PI controller
    float32_t Ui;              //!< the integrator start value for the PI
                               //!< controller
    float32_t refValue;        //!< the reference input value
    float32_t fbackValue;      //!< the feedback input value
    float32_t ffwdValue;       //!< the feedforward input value

    float32_t error;
    float32_t up;
    float32_t outValue;

    float32_t outMin;          //!< the minimum output value allowed for the PI
                               //!< controller
    float32_t outMax;          //!< the maximum output value allowed for the PI
                               //!< controller

    uint16_t integralLock;     //!< Lock integral gain for anti-windup
} PI_Obj;

//*****************************************************************************
//
//! \brief     Runs the series form of the PI controller
//! \param[in] pi          The PI controller object
//! \param[in] fbackValue  The feedback value to the controller
//! \return    The control effort
//
//*****************************************************************************
__attribute__ ((section(".tcmb_code"))) static inline float32_t
PI_run_series(PI_Obj * pi)
{

    pi->error = pi->refValue - pi->fbackValue;

    //
    // Compute the proportional output
    //
    pi->up = pi->Kp * pi->error;

    //
    // Compute the integral output with saturation
    //
    pi->Ui = MATH_sat(pi->Ui + (pi->Ki * pi->up), pi->outMax, pi->outMin);

    //
    // Saturate the output
    //
    pi->outValue = MATH_sat(pi->up + pi->Ui + pi->ffwdValue, pi->outMax, pi->outMin);

    return(pi->outValue);
}

//*****************************************************************************
//
//! \brief Defines the Motor data
//
//*****************************************************************************
typedef struct _Motor_t_
{
    float32_t Ls_d;
    float32_t Ls_q;
    float32_t Rs;
    float32_t Phi_e;

    float32_t I_abc_A[3];           //!< the current values
    float32_t V_abc_V[3];           //!< the voltage values

    float32_t I_scale;
    float32_t V_scale;

    float32_t dcBus_V;              //!< the dcBus value
    float32_t oneOverDcBus_invV;    //!< the DC Bus inverse, 1/V

    float32_t I_ab_A[2];            //!< the current values
    float32_t I_dq_A[2];            //!< the current values

    float32_t theta_e;              //!< the rotor position
    float32_t omega_e;              //!< the current values

    float32_t Sine;
    float32_t Cosine;

    float32_t theta_e_out;
    float32_t Sine_out;
    float32_t Cosine_out;

    float32_t Vff_dq_V[2];          //!< PI_dq FF value for decoupling
    float32_t Vout_dq_V[2];         //!< the current values
    float32_t Vout_ab_V[2];         //!< the current values

    float32_t Vout_dq_out_norm;
    float32_t Vout_dq_sat_gain;

    float32_t Vff_max;
    float32_t Vff_min;

    PI_Obj pi_spd;

    PI_Obj pi_id;
    PI_Obj pi_iq;

    float32_t vqLimit;
    float32_t modulationLimitSquare;

    float32_t Vout_max;

    float32_t sampleTime;
    float32_t outputTimeCompDelay;
    float32_t resolverCompDelay;
    uint16_t resolverCompIdx;
    uint16_t isrResRatio;
} Motor_t;

//*****************************************************************************
//
//! \brief Defines Induction Machine Current Model
//
//*****************************************************************************
typedef struct _ACI_Model_t_
{
    float32_t Kt;
    float32_t Kr;
    float32_t IMDs;
    float32_t Wslip;

} ACI_Model_t;

//*****************************************************************************
//
//! \brief Defines the PWM data
//
//*****************************************************************************
typedef struct _PWMData_t_
{
    float32_t Vabc_pu[3];     //!< the PWM time-durations for each motor phase
    float32_t modulationLimit;

    // SVGEN
    float32_t vmax_pu;
    float32_t vmin_pu;
    float32_t vcom_pu;
    float32_t va_pu;
    float32_t vbeta_pu;
    float32_t va_tmp;
    float32_t vb_tmp;
    float32_t vb_pu;
    float32_t vc_pu;

    // SVGEN
    uint16_t  cmpValue[3];
    uint16_t  deadband[3];
    uint16_t  noiseWindow;
    uint16_t  period;
    uint16_t  socCMP;

    //  PWM out
    uint16_t inv_half_prd;

} PWMData_t;

//*****************************************************************************
//
//! \brief Run clarke transformation
//
//*****************************************************************************
static inline void clarke_run(Motor_t * in)
{
    in->I_ab_A[0] = ((in->I_abc_A[0] * 2.0f) -
                    (in->I_abc_A[1] + in->I_abc_A[2])) * ONE_BY_3;
    in->I_ab_A[1] = ((in->I_abc_A[1] - in->I_abc_A[2]) * ONE_BY_SQRT3);
}

//*****************************************************************************
//
//! \brief run Park transformation
//
//*****************************************************************************
static inline void park_run(Motor_t * in)
{
    in->I_dq_A[0] = (in->I_ab_A[0] * in->Cosine) + (in->I_ab_A[1] * in->Sine);
    in->I_dq_A[1] = (in->I_ab_A[1] * in->Cosine) - (in->I_ab_A[0] * in->Sine);
}

//*****************************************************************************
//
//! \brief run inverse park transformation
//
//*****************************************************************************
static inline void ipark_run(Motor_t * in)
{
    in->Vout_ab_V[0] = (in->Vout_dq_V[0] * in->Cosine) -
                       (in->Vout_dq_V[1] * in->Sine_out);
    in->Vout_ab_V[1] = (in->Vout_dq_V[1] * in->Cosine) +
                       (in->Vout_dq_V[0] * in->Sine_out);
}

//*****************************************************************************
//
//! \brief space vector PWM generation
//
//*****************************************************************************
static inline void SVGEN_run(Motor_t * m, PWMData_t * pwm)
{
    pwm->vmax_pu = 0;
    pwm->vmin_pu = 0;
    pwm->va_pu = m->Vout_ab_V[0] * m->oneOverDcBus_invV;
    pwm->vbeta_pu = m->Vout_ab_V[1] * m->oneOverDcBus_invV;

    pwm->va_tmp = (float32_t)(0.5) * (-pwm->va_pu);
    pwm->vb_tmp = SVGEN_SQRT3_OVER_2 * pwm->vbeta_pu;

    //
    // -0.5*Valpha + sqrt(3)/2 * Vbeta
    //
    pwm->vb_pu = pwm->va_tmp + pwm->vb_tmp;

    //
    // -0.5*Valpha - sqrt(3)/2 * Vbeta
    //
    pwm->vc_pu = pwm->va_tmp - pwm->vb_tmp;

    //
    // Find Vmax and Vmin
    //
    pwm->vmax_pu = MATH_max(MATH_max(pwm->va_pu, pwm->vb_pu), pwm->vc_pu);
    pwm->vmin_pu = MATH_min(MATH_min(pwm->va_pu, pwm->vb_pu), pwm->vc_pu);

    //
    // Compute Vcom = 0.5*(Vmax+Vmin)
    //
    pwm->vcom_pu = (float32_t)0.5 * (pwm->vmax_pu + pwm->vmin_pu);

    //
    // Subtract common-mode term to achieve SV modulation
    //
    pwm->Vabc_pu[0] = (pwm->va_pu - pwm->vcom_pu);
    pwm->Vabc_pu[1] = (pwm->vb_pu - pwm->vcom_pu);
    pwm->Vabc_pu[2] = (pwm->vc_pu - pwm->vcom_pu);

    return;
}

//*****************************************************************************
//
//! \brief Clamp the PWM output to PU range during overmodulation
//
//*****************************************************************************
static inline void PWM_clamp(PWMData_t * pwm)
{
    pwm->Vabc_pu[0] = MATH_max(MATH_min(pwm->Vabc_pu[0], pwm->modulationLimit),
                           - pwm->modulationLimit);
    pwm->Vabc_pu[1] = MATH_max(MATH_min(pwm->Vabc_pu[1], pwm->modulationLimit),
                           - pwm->modulationLimit);
    pwm->Vabc_pu[2] = MATH_max(MATH_min(pwm->Vabc_pu[2], pwm->modulationLimit),
                           - pwm->modulationLimit);
}

//*****************************************************************************
//
//! \brief Run dynamic decoupling of DQ in FOC
//
//*****************************************************************************
static inline void decoupling_run(Motor_t * in)
{
    in->Vff_dq_V[0] = MATH_sat((- in->I_dq_A[1] * in->Ls_q * in->omega_e),
                               in->Vff_max, in->Vff_min);
    in->Vff_dq_V[1] = MATH_sat(((in->I_dq_A[0] * in->Ls_d + in->Phi_e) *
                               in->omega_e), in->Vff_max, in->Vff_min);
}

//*****************************************************************************
//
//! \brief Run saturation based on vector output of Vdq
//
//*****************************************************************************
static inline void dq_limiter_run(Motor_t * in)
{

    in->Vout_dq_out_norm = sqrtf(in->Vout_dq_V[0] * in->Vout_dq_V[0] +
                     in->Vout_dq_V[1] * in->Vout_dq_V[1]);

    if(in->Vout_dq_out_norm > in->Vout_max)
    {
        in->Vout_dq_sat_gain = in->Vout_max / in->Vout_dq_out_norm;
        in->Vout_dq_V[0] = in->Vout_dq_sat_gain * in->Vout_dq_V[0];
        in->Vout_dq_V[1] = in->Vout_dq_sat_gain * in->Vout_dq_V[1];
    }
}

//*****************************************************************************
//
//! \brief Initialization for motor structure
//
//*****************************************************************************
extern void motor_init(Motor_t * in);

#endif
