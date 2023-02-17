//#############################################################################
//
// FILE:  resolver_param.h
//
// TITLE: header file for motor resolver
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef RESOLVER_PARAM_H_
#define RESOLVER_PARAM_H_

#if(USER_MOTOR == TESLA_MODEL_3)
//
// Resolver Related defines
//
// Decouple the main PWM frequency and resolver frequency
// Note that the ratio between PWM frequency resolver sampling frequency
// have to be integer value
//
#define RESOLVER_EXC_FREQUENCY           20.0f

#define RESOLVER_OMEGA                  (200.0f * 2.0f * PI)
#define RESOLVER_ZETA                   0.7f
#define RESOLVER_PHASE_COMP_GAIN        - 4.6585E-5f

#define RESOLVER_LPF_SPD_BW             100.0
#endif

#if(USER_MOTOR == TBD)
//
// Resolver Related defines
//
// Decouple the main PWM frequency and resolver frequency
// Note that the ratio between PWM frequency resolver sampling frequency
// have to be integer value
//
#define RESOLVER_EXC_FREQUENCY           10.0f

#define RESOLVER_OMEGA                  (200.0f * 2.0f * PI)
#define RESOLVER_ZETA                   0.7f
#define RESOLVER_PHASE_COMP_GAIN        - 4.6585E-5f

#define RESOLVER_LPF_SPD_BW             100.0
#endif


#endif /* RESOLVER_PARAM_H_ */
