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
// set the motor parameters to the one available
//
#define TESLA_MODEL_3            1
#define TEKNIC_M2310PLN04K       2
#define TBD                      3

#define USER_MOTOR               TBD

//
// Define the electrical motor parameters
//
#if(USER_MOTOR == TESLA_MODEL_3)
#define RS          0.005f              // Stator resistance (ohm)
#define RR          NULL                // Rotor resistance (ohm)
#define LS_D        0.0001075f          // Stator inductance (H)
#define LS_Q        0.0002983f          // Stator inductance (H)
#define LS          ((LS_D + LS_Q) / 2.0f)   // Stator inductance (H)
#define LR          NULL                // Rotor inductance (H)
#define LM          NULL                // Magnetizing inductance (H)
#define FLUX        0.0398557819f       // BEMF constant (V/Hz)
#define POLES       6                   // Number of poles

#define RATED_VOLTAGE   450.0f          // Motor rated voltage

#define RESOLVER_BIAS   - 2.499f         // Bias between Resolver zero and Id

#define CUR_LOOP_BW (2.0f * PI * ISR_FREQUENCY * 1000.0f / 20.0f)

#elif(USER_MOTOR == TEKNIC_M2310PLN04K)
#define RS          0.373820424f        // Stator resistance (ohm)
#define RR          NULL                // Rotor resistance (ohm)
#define LS_D        0.000198548049f     // Stator inductance (H)
#define LS_Q        0.000198548049f     // Stator inductance (H)
#define LS          ((LS_D + LS_Q) / 2.0f)      // Stator inductance (H)
#define LR          NULL                // Rotor inductance (H)
#define LM          NULL                // Magnetizing inductance (H)
#define FLUX        0.0904f             // BEMF constant (V/Hz)
#define POLES       6                   // Number of poles

#define RATED_VOLTAGE   24.0f           // Motor rated voltage

#define RESOLVER_BIAS   2.40f           // Bias between Resolver zero and Id

#define CUR_LOOP_BW (2.0f * PI * ISR_FREQUENCY * 1000.0f / 40.0f)

#elif(USER_MOTOR == TBD)                //TBD
#define RS          0.05f               // Stator resistance (ohm) - TBD, around 0.055
#define RR          NULL                // Rotor resistance (ohm) - TBD
#define LS_D        0.005f              // Stator inductance (H) - TBD
#define LS_Q        0.005f              // Stator inductance (H) - TBD
#define LS          ((LS_D + LS_Q) / 2.0f)      // Stator inductance (H) - TBD
#define LR          NULL                // Rotor inductance (H) - TBD
#define LM          NULL                // Magnetizing inductance (H) - TBD
#define FLUX        0.0904f             // BEMF constant (V/Hz) -TBD
#define POLES       8                   // Number of poles
#define PAIRS       4                   // Number of pole pairs

#define RATED_VOLTAGE   230.0f           // Motor rated voltage

#define RESOLVER_BIAS   2.40f           // Bias between Resolver zero and Id

#define CUR_LOOP_BW (2.0f * PI * ISR_FREQUENCY * 1000.0f / 40.0f)

#else
#error No motor type specified
#endif

#ifndef USER_MOTOR
#error Motor type is not defined in user.h
#endif

//
// Enumeration for Motor run/ stop command
//
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_RUN = 1
} MotorRunStop_e;

#endif /* MOTOR_PARAM_H_ */
