//#############################################################################
//
// FILE:  Serial_Cmd_HAL.h
//
// TITLE: header file for Serial Command Monitor Hardware Abstraction Layer
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef SERIAL_CMD_HAL_H_
#define SERIAL_CMD_HAL_H_

#include "device.h"

extern void SerialCmd_init(void);
extern void SerialCmd_read(void);
extern void SerialCmd_send(int a);

#endif /* SERIAL_CMD_HAL_H_ */
