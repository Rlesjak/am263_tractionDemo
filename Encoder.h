#ifndef ENCODER_H_
#define ENCODER_H_

#include <drivers/eqep.h>

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


    /* Output: Speed in per-unit */
    float speedPR;

    /* Skaliranje razlike kuta u brzinu */
    uint32_t K1;

    /* NP filtar mjerene brzine */
    float K2;
    /* NP filtar mjerene brzine */
    float K3;


    /* Output: Speed in per-unit */
    float oldPos;
} PosSpeed_Object;

typedef PosSpeed_Object *PosSpeed_Handle;

void PosSpeed_calculate(PosSpeed_Object *p, uint32_t gEqepBaseAddr);

#endif /* ENCODER_H_ */
