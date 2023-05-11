#include "Encoder.h"
#include "math.h"

#define _SATURATE(A, Pos, Neg)  (fmax(((fmin((A),(Pos)))),(Neg)))

#define MAP(IN, INmin, INmax, OUTmin, OUTmax) ((((IN - INmin)*(OUTmax - OUTmin))/(INmax - INmin)) + OUTmin)

float Tmp_fr;

void PosSpeed_calculate(PosSpeed_Object *p, uint32_t gEqepBaseAddr)
{
    int32_t temp;
    uint16_t pos16bVal, temp1;
    long temp2, newPosCnt, oldPosCnt;

    /* Position calculation - mechanical and electrical motor angle
       Get the motor direction: -1 = CCW/reverse, 1 = CW/forward */
    p->directionQEP = EQEP_getDirection(gEqepBaseAddr);

    /* Capture position once per QA/QB period */
    pos16bVal = (uint16_t)EQEP_getPosition(gEqepBaseAddr);

    /* Raw theta = current pos. + ang. offset from QA */
    p->thetaRaw = pos16bVal + p->calAngle;

    /* p->thetaMech ~= QPOSCNT / mechScaler [current cnt/(total cnt in 1 rev)]
       where mechScaler = 4000 cnts/revolution */

    p->thetaMech = (float)p->thetaRaw * p->mechScaler;

    /* The following lines calculate p->elec_mech */
    p->thetaElec = p->polePairs * p->thetaMech;

    /* Check for an index occurrence */
    if((EQEP_getInterruptStatus(gEqepBaseAddr) & EQEP_INT_INDEX_EVNT_LATCH) != 0U)
    {
        EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_INDEX_EVNT_LATCH);
    }

    // Ne mjeri brzinu u trenutku indexa
    // Izbjegavanje naglog skoka brzine koji nije stvaran
    if ((p->thetaMech < 0.95) & (p->thetaMech > 0.05)) {
        Tmp_fr = p->K1 * (p->thetaElec - p->oldPos);
    }
    else if ((p->thetaMech > -0.95) & (p->thetaMech < -0.05)) {
        Tmp_fr = p->K1 * (p->thetaElec - p->oldPos);
    }
    else {
        Tmp_fr = p->speedPR;
    }

    // Niskopropusni filtar brzine
    Tmp_fr = (p->K2 * p->speedPR) + (p->K3 * Tmp_fr);
    /* Saturate the output */
    Tmp_fr=_SATURATE(Tmp_fr,1,-1);
    p->speedPR = Tmp_fr;

    p->oldPos = p->thetaElec;

}
