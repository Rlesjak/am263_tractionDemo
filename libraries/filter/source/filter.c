//#############################################################################
//
// FILE:  filter.c
//
// TITLE: source file for filter library
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#include "device.h"
#include "filter.h"


#define TWO_PI 6.28318530718f

void FILTER_FO_init(Filter_t *filter, const float32_t bandwidth_rps,
                    const float32_t sample_freq_Hz)
{
    float32_t beta = bandwidth_rps / sample_freq_Hz;
    filter->a1 = (beta - 2.0f) / (beta + 2.0f);
    filter->b0 = beta / (beta + 2.0f);
    filter->b1 = filter->b0;
    filter->x1 = 0.0f;
    filter->y1 = 0.0f;
}

