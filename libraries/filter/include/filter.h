//#############################################################################
//
// FILE:  filter.h
//
// TITLE: header file for filter library
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef FILTER_H
#define FILTER_H

//*****************************************************************************
//
//! \brief Defines the first-order filter (FILTER_FO) object
//
//*****************************************************************************
typedef struct _FILTER_FO_Obj_
{
    float32_t a1;    //!< the denominator filter coefficient value for z^(-1)
    float32_t b0;    //!< the numerator filter coefficient value for z^0
    float32_t b1;    //!< the numerator filter coefficient value for z^(-1)
    float32_t x1;    //!< the input value at time sample n=-1
    float32_t y1;    //!< the output value at time sample n=-1
} Filter_t;

//*****************************************************************************
//
//! \brief     Runs a first-order filter of the form
//!            y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
//!
//! \param[in] handle      The filter handle
//!
//! \param[in] inputValue  The input value to filter
//!
//! \return    The output value from the filter
//
//*****************************************************************************
static inline float32_t
FILTER_FO_run(Filter_t *filter, const float32_t inputValue)
{
    //
    // Compute the output
    //
    filter->y1 = (filter->b0 * inputValue) + (filter->b1 * filter->x1) -
                 (filter->a1 * filter->y1);

    //
    // Store values for next time
    //
    filter->x1 = inputValue;

    return(filter->y1);
}

extern void FILTER_FO_init(Filter_t *filter, const float32_t bandwidth_rps,
                           const float32_t sample_freq_Hz);

#endif

