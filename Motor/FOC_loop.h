//#############################################################################
//
// FILE:  foc_loop.h
//
// TITLE: header file for FOC loop
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef FOC_LOOP_H_
#define FOC_LOOP_H_

#include "Motor_param.h"
#include "foc.h"
#include "Encoder.h"

extern uint16_t gIsrTicker;
extern uint16_t gIsrLock1;

extern void FOC_init(void);
extern void FOC_cal(void);
extern void FOC_run(void);

void FOC_setMotorRunState(MotorRunStop_e state);
unsigned int FOC_getMotorRunState();
void FOC_setVd(float32_t Vd);
void FOC_setVq(float32_t Vq);
void FOC_setSpeedRef(float32_t SpdRef);
float32_t FOC_getSpeedRef(void);
float32_t FOC_getVd(void);
float32_t FOC_getVq(void);
float32_t FOC_getMotorDCBus(void);

float32_t FOC_getIdref(void);
float32_t FOC_getIqref(void);
void FOC_setIdref(float32_t value);
void FOC_setIqref(float32_t value);

float32_t FOC_getVdBefMock(void);
float32_t FOC_getVqBefMock(void);

Motor_t* FOC_DANGER_getMotorStructPointer();
PosSpeed_Object* FOC_DANGER_getPosSpeedStructPointer();


#endif /* FOC_LOOP_H_ */
