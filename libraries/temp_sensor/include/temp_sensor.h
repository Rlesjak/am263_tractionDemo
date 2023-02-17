//#############################################################################
//
// FILE:  temp_sensor.h
//
// TITLE: header file for temp_sensor interface modules
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#include "device.h"

//
// temperature look up table
//
extern float32_t temp_lut[256];

//
// function prototypes
//

static inline float32_t
calcTemp(uint16_t adc_reading)
{
    uint16_t index = adc_reading >> 4;
    return(temp_lut[index]);
}

#endif
