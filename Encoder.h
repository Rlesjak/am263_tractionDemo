/*
 * Encoder.h
 *
 *  Created on: Apr 26, 2023
 *      Author: laptop
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <drivers/eqep.h>


/* Sysclk frequency */
#define DEVICE_SYSCLK_FREQ  (200000000U)
// See Equation 5 in eqep_ex2_calculation.c
#define SPEED_SCALER  ((((uint64_t)32 * DEVICE_SYSCLK_FREQ / 64) * 60) / (24000000))


typedef struct
{
    /* Output: Motor electrical angle (Q15) */
    float thetaElec;
    /* Output: Motor mechanical angle (Q15) */
    float thetaMech;
    /* Output: Motor rotation direction (Q0) */
    int16_t directionQEP;
    /* Variable: Raw angle from timer 2 (Q0) */
    int16_t thetaRaw;

    /* Parameter: 0.9999 / total count, total count = 4000 (Q26) */
    float mechScaler;
    /* Parameter: Number of pole pairs (Q0) */
    int16_t polePairs;
    /* Parameter: Raw angular offset between encoder and Phase A (Q0) */
    int16_t calAngle;
    /* Low speed scaler */
    uint32_t speedScaler;


    /* Output: Speed in per-unit */
    float speedPR;


    /* Skaliranje razlike kuta u brzinu */
    uint32_t K1;

    /* Skaliranje razlike kuta u brzinu */
    float K2;
    /* Skaliranje razlike kuta u brzinu */
    float K3;


    /* Output: Speed in rpm (Q0) - independently with global Q */
    float speedRPMPR;
    /* Output: Speed in per-unit */
    float oldPos;
    float speedFR;
    /* Output: Speed in rpm (Q0) - independently with global Q */
    float speedRPMFR;
} PosSpeed_Object;

typedef PosSpeed_Object *PosSpeed_Handle;

void PosSpeed_calculate(PosSpeed_Object *p, uint32_t gEqepBaseAddr);

#endif /* ENCODER_H_ */
