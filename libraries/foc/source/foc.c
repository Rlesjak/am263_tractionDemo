//#############################################################################
//
// FILE:  foc.c
//
// TITLE: source file for FOC control library
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#include "device.h"
#include "foc.h"

void motor_init(Motor_t * in)
{
    in->Ls_d = 0.0;
    in->Ls_q = 0.0;
    in->Rs = 1.0;
    in->Phi_e = 0.0;

    in->I_scale = 0.0;
    in->V_scale = 0.0;

    in->dcBus_V = 1.0;
    in->oneOverDcBus_invV = 1.0;

    in->theta_e = 0.0;
    in->omega_e = 0.0;

    in->Sine = 0.0;
    in->Cosine = 1.0;

    in->Vff_dq_V[0] = 0.0;
    in->Vff_dq_V[1] = 0.0;

    in->I_dq_A[0] = 0.0;
    in->I_dq_A[1] = 0.0;

    in->I_ab_A[0] = 0.0;
    in->I_ab_A[1] = 0.0;

    in->I_abc_A[0] = 0.0;
    in->I_abc_A[1] = 0.0;
    in->I_abc_A[2] = 0.0;

    //
    // Initialize the PI controllers
    //
    in->pi_spd.Kp = 1.0;
    in->pi_spd.Ki = 0.0;
    in->pi_spd.Ui = 0.0;
    in->pi_spd.refValue = 0.0;
    in->pi_spd.fbackValue = 0.0;
    in->pi_spd.ffwdValue = 0.0;
    in->pi_spd.outMax = 1.0;
    in->pi_spd.outMin = -1.0;

    in->pi_id.Kp = 1.0;
    in->pi_id.Ki = 0.0;
    in->pi_id.Ui = 0.0;
    in->pi_id.refValue = 0.0;
    in->pi_id.fbackValue = 0.0;
    in->pi_id.ffwdValue = 0.0;
    in->pi_id.outMax = 1.0;
    in->pi_id.outMin = -1.0;

    in->pi_iq.Kp = 1.0;
    in->pi_iq.Ki = 0.0;
    in->pi_iq.Ui = 0.0;
    in->pi_iq.refValue = 0.0;
    in->pi_iq.fbackValue = 0.0;
    in->pi_iq.ffwdValue = 0.0;
    in->pi_iq.outMax = 1.0;
    in->pi_iq.outMin = -1.0;
}
