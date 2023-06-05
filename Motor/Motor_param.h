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

// Električni parametri motora
#define RS          11.05f              // Otpor statora (ohm)
#define RR          6.11f               // Otpor rotora (ohm)
#define LS_D        0.005f              // Induktivitet statora u D osi (H)
#define LS_Q        0.005f              // Induktivitet statora u D osi (H)
#define LS          0.31642f            // Induktivitet statora
#define FLUX        0.0904f             // Back EMF (V/Hz)
#define POLES       4                   // Broj polova
#define PAIRS       2                   // Broj pari polova

// Nazivni napon je 230V, ali je zbog sigurnosti pri testiranju
// postavljeno ograničenje na 100V
#define RATED_VOLTAGE   100.0f          // Nazivni napon motora (V)

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_RUN = 1
} MotorRunStop_e;

#endif
