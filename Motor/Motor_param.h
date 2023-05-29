//#############################################################################
//
// FILE:  motor_param.h
//
// TITLE: header file for motor
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef MOTOR_PARAM_H_
#define MOTOR_PARAM_H_

//
// Define the electrical motor parameters
//
#define RS          11.05f               // Stator resistance (ohm) - TBD, around 0.055
#define RR          6.11f                // Rotor resistance (ohm) - TBD
#define LS_D        0.005f              // Stator inductance (H) - TBD
#define LS_Q        0.005f              // Stator inductance (H) - TBD
#define LS          ((LS_D + LS_Q) / 2.0f)      // Stator inductance (H) - TBD
#define LR          NULL                // Rotor inductance (H) - TBD
#define LM          NULL                // Magnetizing inductance (H) - TBD
#define FLUX        0.0904f             // BEMF constant (V/Hz) -TBD
#define POLES       4                   // Number of poles
#define PAIRS       2                   // Number of pole pairs

#define RATED_VOLTAGE   100.0f           // Motor rated voltage

#define RESOLVER_BIAS   2.40f           // Bias between Resolver zero and Id

#define CUR_LOOP_BW (2.0f * PI * ISR_FREQUENCY * 1000.0f / 40.0f)

//
// Enumeration for Motor run/ stop command
//
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_RUN = 1
} MotorRunStop_e;

#endif /* MOTOR_PARAM_H_ */
