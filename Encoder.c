#include "Encoder.h"
#include "math.h"

#define _SATURATE(A, Pos, Neg)  (fmax(((fmin((A),(Pos)))),(Neg)))

#define MAP(IN, INmin, INmax, OUTmin, OUTmax) ((((IN - INmin)*(OUTmax - OUTmin))/(INmax - INmin)) + OUTmin)

float Tmp_fr;

void PosSpeed_calculate(PosSpeed_Object *p, uint32_t gEqepBaseAddr)
{
    uint16_t posCount;

    // Procitaj smjer vrtnje
    p->directionQEP = EQEP_getDirection(gEqepBaseAddr);
    // Procitaj brojac
    posCount = (uint16_t)EQEP_getPosition(gEqepBaseAddr);
    // Dodaj kalibracijski kut izmjerenome
    p->thetaRaw = posCount + p->calAngle;
    // Skaliraj broj impulsa u meh radijane
    p->thetaMech = (float)p->thetaRaw * p->mechScaler;
    // Kaliraj meh. kut u el. kut
    p->thetaElec = p->polePairs * p->thetaMech;

    // Ako se dogodio prekid radi prelaksa indexa
    if((EQEP_getInterruptStatus(gEqepBaseAddr) & EQEP_INT_INDEX_EVNT_LATCH) != 0U)
    {
        // Resetiraj zastavicu da se dogodio prekid
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
