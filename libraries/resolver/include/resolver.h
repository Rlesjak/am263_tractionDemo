//#############################################################################
//
// FILE:    resolver.h
//
// TITLE:   header file for software resolver
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef RESOLVER_H
#define RESOLVER_H

//
// includes
//
#include "math.h"
#include "device.h"
#include "filter.h"

//#define RESOLVER_CORE RUN_CLA

#ifndef TWO_PI
#define TWO_PI 6.2831853071795862f
#endif
#define ONE_OVER_TWO_PI 0.159154937f

typedef struct _Resolver_t_
{
    //
    // Input variables
    //
    float32_t sin_samples[2];
    float32_t cos_samples[2];

    float32_t sin_os;
    float32_t cos_os;

    //
    // Output variables (pointer)
    //
    float32_t *resolver_theta;
    float32_t *resolver_omega;

    //
    // Parameters
    //
    float32_t sample_time;
    float32_t pll_gain_in;
    float32_t pll_gain_ff;
    float32_t phase_comp_gain;
    float32_t bias;

    //
    // Motor Poles / Resolver Poles for theta_e conversion
    //
    float32_t theta_ratio;

    //
    // Internal variables
    //
    float32_t exc_inv;
    float32_t res_int1;
    float32_t res_int2;
    float32_t res_theta0;
    float32_t res_theta0_sin;
    float32_t res_theta0_cos;

    //
    // function internal variables
    //
    float32_t postFilterSpeed;
    float32_t pllLoopError ;
    float32_t pllPreFilterSpeed;
    float32_t pllLoopInt;
    float32_t resolverThetaPreWrap;
    float32_t pllSpeedInt;
    float32_t pllFmod;
    float32_t exeInvMinus;

    Filter_t lpf_spd;



} Resolver_t;

static inline void resolver_init(Resolver_t *resolver)
{
    resolver->sin_os = 0.0f;
    resolver->cos_os = 0.0f;
    resolver->resolver_theta = (void *)0;
    resolver->resolver_omega = (void *)0;
    resolver->sample_time = 0.0f;
    resolver->pll_gain_in = 0.0f;
    resolver->pll_gain_ff = 0.0f;
    resolver->phase_comp_gain = 0.0f;
    resolver->bias = 0.0f;

    resolver->lpf_spd.a1 = 0.0f;
    resolver->lpf_spd.b0 = 0.0f;
    resolver->lpf_spd.b1 = 0.0f;
    resolver->lpf_spd.x1 = 0.0f;
    resolver->lpf_spd.y1 = 0.0f;

    resolver->exc_inv = 1.0f;
    resolver->res_int1 = 0.0f;
    resolver->res_int2 = 0.0f;
    resolver->res_theta0 = 0.0f;
}

static inline void theta_limiter(float32_t *theta)
{
    while (*theta > TWO_PI)
    {
        *theta -= TWO_PI;
    }
    while (*theta < 0)
    {
        *theta += TWO_PI;
    }
}

static inline void resolver_run(Resolver_t *resolver)
{

    //
    // exc_inv is flipping between 1 and -1 to determain
    // plus and minus peak of carrier sine signal
    //
//    resolver->exeInvMinus = -resolver->exc_inv;

    //
    // Error calculation for PLL tracking loop with input gain
    //
//    resolver->pllLoopError = (resolver->sin_os * -resolver->exc_inv *
//                   resolver->res_theta0_cos - resolver->cos_os *
//                   - resolver->exc_inv * resolver->res_theta0_sin) *
//                   resolver->pll_gain_in;

    resolver->pllLoopError = (resolver->sin_os * resolver->res_theta0_cos
                            - resolver->cos_os * resolver->res_theta0_sin)
                            * resolver->pll_gain_in;

    //
    // Integrator 1 for first order PLL tracking loop
    // Input gain: Omega ^ 2
    //
    resolver->pllLoopInt = resolver->sample_time * resolver->pllLoopError  +
                           resolver->res_int1;

    //
    // Speed before filtering in PLL tracking loop
    //
    resolver->pllPreFilterSpeed = resolver->pll_gain_ff * resolver->pllLoopError + resolver->pllLoopInt;

    //
    // Integrator 2 from speed to position
    //
    resolver->pllSpeedInt = resolver->sample_time * resolver->pllPreFilterSpeed +
                            resolver->res_int2;

    //
    // Fmod using multiplicaiton
    //
    resolver->pllFmod = resolver->pllSpeedInt -
              (float32_t)(int32_t)(ONE_OVER_TWO_PI  * resolver->pllSpeedInt) * TWO_PI;

    //
    // Low pass filter for speed output
    //
    resolver->postFilterSpeed = FILTER_FO_run(&resolver->lpf_spd, resolver->pllPreFilterSpeed);

    //
    // Phase delay compensation
    //
    resolver->resolverThetaPreWrap =
        (resolver->phase_comp_gain * resolver->postFilterSpeed + resolver->pllFmod) +
        resolver->bias;

    //
    // Wrap PLL output between 0 and 2 PI
    //
    *resolver->resolver_theta = resolver->resolverThetaPreWrap;
    theta_limiter(resolver->resolver_theta);

//    if(resolver->resolverThetaPreWrap > TWO_PI)
//    {
//        *resolver->resolver_theta = resolver->resolverThetaPreWrap - TWO_PI;
//    }
//    else if(resolver->resolverThetaPreWrap < 0.0F)
//    {
//        *resolver->resolver_theta = resolver->resolverThetaPreWrap + TWO_PI;
//    }
//    else
//    {
//        *resolver->resolver_theta = resolver->resolverThetaPreWrap;
//    }

    *resolver->resolver_omega = resolver->postFilterSpeed;

    resolver->exc_inv = resolver->exeInvMinus;

    resolver->res_theta0 = resolver->pllFmod;

    resolver->res_int1 = resolver->pllLoopInt;

    resolver->res_int2 = resolver->pllFmod;
}

#endif

