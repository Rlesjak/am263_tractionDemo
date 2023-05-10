#include "Encoder.h"
#include "math.h"

#define _SATURATE(A, Pos, Neg)  (fmax(((fmin((A),(Pos)))),(Neg)))

#define MAP(IN, INmin, INmax, OUTmin, OUTmax) ((((IN - INmin)*(OUTmax - OUTmin))/(INmax - INmin)) + OUTmin)


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

    p->speedPR = p->K1 * (p->thetaElec - p->oldPos);
    p->oldPos = p->thetaElec;


//    /* High Speed Calculation using QEP Position counter
//       Check for unit position event */
//    if((EQEP_getInterruptStatus(gEqepBaseAddr) & EQEP_INT_UNIT_TIME_OUT) != 0)
//    {
//        /* The following lines calculate position:
//           (x2 - x1) / 4000 (position in 1 revolution) */
//
//        // (*c)++; /* Incrementing the count value */
//        pos16bVal = (uint16_t)EQEP_getPositionLatch(gEqepBaseAddr);
//        temp = (int32_t)pos16bVal * (int32_t)p->mechScaler;
//
//        newPosCnt = (long)temp;
//        oldPosCnt = p->oldPos;
//
//        /* POSCNT is counting down */
//        if(p->directionQEP == -1)
//        {
//            if(newPosCnt > oldPosCnt)
//            {
//                /* x2 - x1 should be negative */
//                temp2 = -(1 - newPosCnt + oldPosCnt);
//            }
//            else
//            {
//                temp2 = newPosCnt - oldPosCnt;
//            }
//        }
//        /* POSCNT is counting up */
//        else
//        {
//            if(newPosCnt < oldPosCnt)
//            {
//                temp2 = 1 + newPosCnt - oldPosCnt;
//            }
//            else
//            {
//                /* x2 - x1 should be positive */
//                temp2 = newPosCnt - oldPosCnt;
//            }
//        }
//
//        if(temp2 > 1)
//        {
//            p->speedFR = 1;
//        }
//        else if(temp2 < -1)
//        {
//            p->speedFR = -1;
//        }
//        else
//        {
//            p->speedFR = temp2;
//        }
//
//        /* Update the electrical angle */
//        p->oldPos = newPosCnt;
//
//        /* Change motor speed from pu value to rpm value */
//        p->speedRPMFR = p->K1 * p->speedFR;
//
//        /* Clear unit time out flag */
//        EQEP_clearInterruptStatus(gEqepBaseAddr, EQEP_INT_UNIT_TIME_OUT);
//    }
//
//    /* Low-speed computation using QEP capture counter
//       Check for unit position event */
//    if((EQEP_getStatus(gEqepBaseAddr) & EQEP_STS_UNIT_POS_EVNT) != 0)
//    {
//        /* No Capture overflow */
//        if((EQEP_getStatus(gEqepBaseAddr) & EQEP_STS_CAP_OVRFLW_ERROR) == 0)
//        {
//            temp1 = (uint32_t)EQEP_getCapturePeriodLatch(gEqepBaseAddr);
//        }
//        else
//        {
//            /* Capture overflow, saturate the result */
//            temp1 = 0xFFFF;
//        }
//
//        p->speedPR = (float)p->speedScaler / (float)temp1;
//        temp2 = p->speedPR;
//
//        if(temp2 > 1)
//        {
//           p->speedPR = 1;
//        }
//        else
//        {
//           p->speedPR = temp2;
//        }
//
//        /* Convert p->speedPR to RPM
//           Reverse direction = negative */
//        if(p->directionQEP == -1)
//        {
//            p->speedRPMPR = - p->K1 * p->speedPR;
//        }
//        /* Forward direction = positive */
//        else
//        {
//            p->speedRPMPR = p->K1 * p->speedPR;
//        }
//
//        /* Clear unit position event flag and overflow error flag */
//        EQEP_clearStatus(gEqepBaseAddr, (EQEP_STS_UNIT_POS_EVNT |
//                                      EQEP_STS_CAP_OVRFLW_ERROR));
//    }

}
