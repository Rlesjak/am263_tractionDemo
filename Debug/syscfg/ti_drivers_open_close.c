/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Auto generated file 
 */

#include "ti_drivers_open_close.h"
#include <kernel/dpl/DebugP.h>

void Drivers_open(void)
{

    Drivers_edmaOpen();
    Drivers_adcOpen();
    Drivers_dacOpen();
    Drivers_epwmOpen();
    Drivers_mcspiOpen();
    Drivers_uartOpen();
    Drivers_intXbarOpen();
    Drivers_dmaTrigXbarOpen();
    Drivers_dmaXbarOpen();
}

void Drivers_close(void)
{
    Drivers_mcspiClose();
    Drivers_uartClose();
    Drivers_edmaClose();
}

void Drivers_adcOpen()
{
	/* CONFIG_ADC4 initialization */

	/* Configures the analog-to-digital converter module prescaler. */
	ADC_setPrescaler(CONFIG_ADC4_BASE_ADDR, ADC_CLK_DIV_3_0);
	/* Configures the analog-to-digital converter resolution and signal mode. */
	ADC_setMode(CONFIG_ADC4_BASE_ADDR, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	/* Sets the priority mode of the SOCs. */
	ADC_setSOCPriority(CONFIG_ADC4_BASE_ADDR, ADC_PRI_ALL_ROUND_ROBIN);

	/* Start of Conversion 0 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 0, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN3, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 0, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 1 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 1, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN1, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 1, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 2 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 2, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN1, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 2, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 3 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 3, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN3, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 3, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 4 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 4, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 4, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 5 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 5, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 5, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 6 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 6, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 6, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 7 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 7, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 7, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 8 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 8, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 8, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 9 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 9, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 9, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 10 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 10, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 10, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 11 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 11, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 11, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 12 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 12, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 12, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 13 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 13, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 13, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 14 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 14, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 14, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 15 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC4_BASE_ADDR, 15, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC4_BASE_ADDR, 15, ADC_INT_SOC_TRIGGER_NONE);

	/* Powers up the analog-to-digital converter core. */
	ADC_enableConverter(CONFIG_ADC4_BASE_ADDR);
    /* Delay for ADC to power up. */
    ClockP_usleep(500);
	/* Sets the timing of the end-of-conversion pulse */
	ADC_setInterruptPulseMode(CONFIG_ADC4_BASE_ADDR, ADC_PULSE_END_OF_CONV);


	/* ADC Interrupt 1 Configuration */
	/* Enables an ADC interrupt source. */
	ADC_enableInterrupt(CONFIG_ADC4_BASE_ADDR, 0);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC4_BASE_ADDR, 0, ADC_SOC_NUMBER5);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC4_BASE_ADDR, 0);

	/* ADC Interrupt 2 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC4_BASE_ADDR, 1);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC4_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC4_BASE_ADDR, 1);

	/* ADC Interrupt 3 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC4_BASE_ADDR, 2);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC4_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC4_BASE_ADDR, 2);

	/* ADC Interrupt 4 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC4_BASE_ADDR, 3);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC4_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC4_BASE_ADDR, 3);


	/* Post Processing Block 1 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC4_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC4_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC4_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC4_BASE_ADDR, 0, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC4_BASE_ADDR, 0, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC4_BASE_ADDR, 0);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC4_BASE_ADDR, 0, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC4_BASE_ADDR, 0);

	/* Post Processing Block 2 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC4_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC4_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC4_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC4_BASE_ADDR, 1, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC4_BASE_ADDR, 1, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC4_BASE_ADDR, 1);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC4_BASE_ADDR, 1, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC4_BASE_ADDR, 1);

	/* Post Processing Block 3 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC4_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC4_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC4_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC4_BASE_ADDR, 2, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC4_BASE_ADDR, 2, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC4_BASE_ADDR, 2);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC4_BASE_ADDR, 2, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC4_BASE_ADDR, 2);

	/* Post Processing Block 4 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC4_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC4_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC4_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC4_BASE_ADDR, 3, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC4_BASE_ADDR, 3, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC4_BASE_ADDR, 3);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC4_BASE_ADDR, 3, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC4_BASE_ADDR, 3);

	/* Set SOC burst mode. */
	ADC_setBurstModeConfig(CONFIG_ADC4_BASE_ADDR, ADC_TRIGGER_SW_ONLY, 1);
	/* Disables SOC burst mode. */
	ADC_disableBurstMode(CONFIG_ADC4_BASE_ADDR);
	/* CONFIG_ADC1 initialization */

	/* Configures the analog-to-digital converter module prescaler. */
	ADC_setPrescaler(CONFIG_ADC1_BASE_ADDR, ADC_CLK_DIV_3_0);
	/* Configures the analog-to-digital converter resolution and signal mode. */
	ADC_setMode(CONFIG_ADC1_BASE_ADDR, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	/* Sets the priority mode of the SOCs. */
	ADC_setSOCPriority(CONFIG_ADC1_BASE_ADDR, ADC_PRI_ALL_ROUND_ROBIN);

	/* Start of Conversion 0 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 0, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN2, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 0, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 1 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 1, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 2 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 2, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 3 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 3, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 4 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 4, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 4, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 5 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 5, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 5, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 6 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 6, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 6, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 7 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 7, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 7, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 8 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 8, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 8, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 9 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 9, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 9, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 10 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 10, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 10, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 11 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 11, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 11, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 12 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 12, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 12, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 13 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 13, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 13, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 14 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 14, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 14, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 15 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC1_BASE_ADDR, 15, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC1_BASE_ADDR, 15, ADC_INT_SOC_TRIGGER_NONE);

	/* Powers up the analog-to-digital converter core. */
	ADC_enableConverter(CONFIG_ADC1_BASE_ADDR);
    /* Delay for ADC to power up. */
    ClockP_usleep(500);
	/* Sets the timing of the end-of-conversion pulse */
	ADC_setInterruptPulseMode(CONFIG_ADC1_BASE_ADDR, ADC_PULSE_END_OF_ACQ_WIN);


	/* ADC Interrupt 1 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC1_BASE_ADDR, 0);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC1_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC1_BASE_ADDR, 0);

	/* ADC Interrupt 2 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC1_BASE_ADDR, 1);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC1_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC1_BASE_ADDR, 1);

	/* ADC Interrupt 3 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC1_BASE_ADDR, 2);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC1_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC1_BASE_ADDR, 2);

	/* ADC Interrupt 4 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC1_BASE_ADDR, 3);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC1_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC1_BASE_ADDR, 3);


	/* Post Processing Block 1 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC1_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC1_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC1_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC1_BASE_ADDR, 0, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC1_BASE_ADDR, 0, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC1_BASE_ADDR, 0);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC1_BASE_ADDR, 0, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC1_BASE_ADDR, 0);

	/* Post Processing Block 2 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC1_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC1_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC1_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC1_BASE_ADDR, 1, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC1_BASE_ADDR, 1, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC1_BASE_ADDR, 1);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC1_BASE_ADDR, 1, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC1_BASE_ADDR, 1);

	/* Post Processing Block 3 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC1_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC1_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC1_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC1_BASE_ADDR, 2, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC1_BASE_ADDR, 2, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC1_BASE_ADDR, 2);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC1_BASE_ADDR, 2, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC1_BASE_ADDR, 2);

	/* Post Processing Block 4 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC1_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC1_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC1_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC1_BASE_ADDR, 3, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC1_BASE_ADDR, 3, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC1_BASE_ADDR, 3);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC1_BASE_ADDR, 3, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC1_BASE_ADDR, 3);

	/* Set SOC burst mode. */
	ADC_setBurstModeConfig(CONFIG_ADC1_BASE_ADDR, ADC_TRIGGER_SW_ONLY, 1);
	/* Disables SOC burst mode. */
	ADC_disableBurstMode(CONFIG_ADC1_BASE_ADDR);
	/* CONFIG_ADC2 initialization */

	/* Configures the analog-to-digital converter module prescaler. */
	ADC_setPrescaler(CONFIG_ADC2_BASE_ADDR, ADC_CLK_DIV_3_0);
	/* Configures the analog-to-digital converter resolution and signal mode. */
	ADC_setMode(CONFIG_ADC2_BASE_ADDR, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	/* Sets the priority mode of the SOCs. */
	ADC_setSOCPriority(CONFIG_ADC2_BASE_ADDR, ADC_PRI_ALL_ROUND_ROBIN);

	/* Start of Conversion 0 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 0, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN2, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 0, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 1 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 1, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 1, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 2 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 2, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 3 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 3, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 4 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 4, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 4, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 5 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 5, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 5, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 6 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 6, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 6, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 7 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 7, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 7, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 8 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 8, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 8, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 9 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 9, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 9, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 10 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 10, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 10, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 11 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 11, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 11, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 12 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 12, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 12, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 13 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 13, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 13, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 14 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 14, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 14, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 15 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC2_BASE_ADDR, 15, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC2_BASE_ADDR, 15, ADC_INT_SOC_TRIGGER_NONE);

	/* Powers up the analog-to-digital converter core. */
	ADC_enableConverter(CONFIG_ADC2_BASE_ADDR);
    /* Delay for ADC to power up. */
    ClockP_usleep(500);
	/* Sets the timing of the end-of-conversion pulse */
	ADC_setInterruptPulseMode(CONFIG_ADC2_BASE_ADDR, ADC_PULSE_END_OF_ACQ_WIN);


	/* ADC Interrupt 1 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC2_BASE_ADDR, 0);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC2_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC2_BASE_ADDR, 0);

	/* ADC Interrupt 2 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC2_BASE_ADDR, 1);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC2_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC2_BASE_ADDR, 1);

	/* ADC Interrupt 3 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC2_BASE_ADDR, 2);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC2_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC2_BASE_ADDR, 2);

	/* ADC Interrupt 4 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC2_BASE_ADDR, 3);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC2_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC2_BASE_ADDR, 3);


	/* Post Processing Block 1 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC2_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC2_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC2_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC2_BASE_ADDR, 0, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC2_BASE_ADDR, 0, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC2_BASE_ADDR, 0);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC2_BASE_ADDR, 0, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC2_BASE_ADDR, 0);

	/* Post Processing Block 2 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC2_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC2_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC2_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC2_BASE_ADDR, 1, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC2_BASE_ADDR, 1, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC2_BASE_ADDR, 1);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC2_BASE_ADDR, 1, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC2_BASE_ADDR, 1);

	/* Post Processing Block 3 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC2_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC2_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC2_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC2_BASE_ADDR, 2, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC2_BASE_ADDR, 2, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC2_BASE_ADDR, 2);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC2_BASE_ADDR, 2, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC2_BASE_ADDR, 2);

	/* Post Processing Block 4 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC2_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC2_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC2_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC2_BASE_ADDR, 3, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC2_BASE_ADDR, 3, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC2_BASE_ADDR, 3);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC2_BASE_ADDR, 3, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC2_BASE_ADDR, 3);

	/* Set SOC burst mode. */
	ADC_setBurstModeConfig(CONFIG_ADC2_BASE_ADDR, ADC_TRIGGER_SW_ONLY, 1);
	/* Disables SOC burst mode. */
	ADC_disableBurstMode(CONFIG_ADC2_BASE_ADDR);
	/* CONFIG_ADC3 initialization */

	/* Configures the analog-to-digital converter module prescaler. */
	ADC_setPrescaler(CONFIG_ADC3_BASE_ADDR, ADC_CLK_DIV_3_0);
	/* Configures the analog-to-digital converter resolution and signal mode. */
	ADC_setMode(CONFIG_ADC3_BASE_ADDR, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	/* Sets the priority mode of the SOCs. */
	ADC_setSOCPriority(CONFIG_ADC3_BASE_ADDR, ADC_PRI_ALL_ROUND_ROBIN);

	/* Start of Conversion 0 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 0, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN0, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 0, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 1 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 1, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN3, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 1, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 2 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 2, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 3 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 3, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 4 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 4, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 4, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 5 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 5, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 5, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 6 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 6, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 6, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 7 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 7, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 7, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 8 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 8, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 8, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 9 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 9, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 9, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 10 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 10, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 10, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 11 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 11, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 11, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 12 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 12, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 12, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 13 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 13, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 13, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 14 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 14, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 14, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 15 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC3_BASE_ADDR, 15, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC3_BASE_ADDR, 15, ADC_INT_SOC_TRIGGER_NONE);

	/* Powers up the analog-to-digital converter core. */
	ADC_enableConverter(CONFIG_ADC3_BASE_ADDR);
    /* Delay for ADC to power up. */
    ClockP_usleep(500);
	/* Sets the timing of the end-of-conversion pulse */
	ADC_setInterruptPulseMode(CONFIG_ADC3_BASE_ADDR, ADC_PULSE_END_OF_ACQ_WIN);


	/* ADC Interrupt 1 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC3_BASE_ADDR, 0);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC3_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC3_BASE_ADDR, 0);

	/* ADC Interrupt 2 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC3_BASE_ADDR, 1);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC3_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC3_BASE_ADDR, 1);

	/* ADC Interrupt 3 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC3_BASE_ADDR, 2);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC3_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC3_BASE_ADDR, 2);

	/* ADC Interrupt 4 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC3_BASE_ADDR, 3);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC3_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC3_BASE_ADDR, 3);


	/* Post Processing Block 1 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC3_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC3_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC3_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC3_BASE_ADDR, 0, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC3_BASE_ADDR, 0, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC3_BASE_ADDR, 0);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC3_BASE_ADDR, 0, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC3_BASE_ADDR, 0);

	/* Post Processing Block 2 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC3_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC3_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC3_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC3_BASE_ADDR, 1, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC3_BASE_ADDR, 1, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC3_BASE_ADDR, 1);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC3_BASE_ADDR, 1, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC3_BASE_ADDR, 1);

	/* Post Processing Block 3 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC3_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC3_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC3_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC3_BASE_ADDR, 2, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC3_BASE_ADDR, 2, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC3_BASE_ADDR, 2);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC3_BASE_ADDR, 2, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC3_BASE_ADDR, 2);

	/* Post Processing Block 4 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC3_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC3_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC3_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC3_BASE_ADDR, 3, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC3_BASE_ADDR, 3, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC3_BASE_ADDR, 3);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC3_BASE_ADDR, 3, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC3_BASE_ADDR, 3);

	/* Set SOC burst mode. */
	ADC_setBurstModeConfig(CONFIG_ADC3_BASE_ADDR, ADC_TRIGGER_SW_ONLY, 1);
	/* Disables SOC burst mode. */
	ADC_disableBurstMode(CONFIG_ADC3_BASE_ADDR);
	/* CONFIG_ADC0 initialization */

	/* Configures the analog-to-digital converter module prescaler. */
	ADC_setPrescaler(CONFIG_ADC0_BASE_ADDR, ADC_CLK_DIV_3_0);
	/* Configures the analog-to-digital converter resolution and signal mode. */
	ADC_setMode(CONFIG_ADC0_BASE_ADDR, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
	/* Sets the priority mode of the SOCs. */
	ADC_setSOCPriority(CONFIG_ADC0_BASE_ADDR, ADC_PRI_ALL_ROUND_ROBIN);

	/* Start of Conversion 0 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 0, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN2, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 0, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 1 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 1, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN3, 15);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 1, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 2 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 2, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 2, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 3 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 3, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 3, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 4 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 4, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 4, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 5 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 5, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 5, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 6 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 6, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 6, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 7 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 7, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 7, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 8 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 8, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 8, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 9 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 9, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 9, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 10 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 10, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 10, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 11 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 11, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 11, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 12 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 12, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 12, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 13 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 13, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 13, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 14 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 14, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 14, ADC_INT_SOC_TRIGGER_NONE);

	/* Start of Conversion 15 Configuration */
	/* Configures a start-of-conversion (SOC) in the ADC. */
	ADC_setupSOC(CONFIG_ADC0_BASE_ADDR, 15, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 1);
	/* Configures the interrupt SOC trigger of an SOC. */
	ADC_setInterruptSOCTrigger(CONFIG_ADC0_BASE_ADDR, 15, ADC_INT_SOC_TRIGGER_NONE);

	/* Powers up the analog-to-digital converter core. */
	ADC_enableConverter(CONFIG_ADC0_BASE_ADDR);
    /* Delay for ADC to power up. */
    ClockP_usleep(500);
	/* Sets the timing of the end-of-conversion pulse */
	ADC_setInterruptPulseMode(CONFIG_ADC0_BASE_ADDR, ADC_PULSE_END_OF_ACQ_WIN);


	/* ADC Interrupt 1 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC0_BASE_ADDR, 0);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC0_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC0_BASE_ADDR, 0);

	/* ADC Interrupt 2 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC0_BASE_ADDR, 1);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC0_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC0_BASE_ADDR, 1);

	/* ADC Interrupt 3 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC0_BASE_ADDR, 2);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC0_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC0_BASE_ADDR, 2);

	/* ADC Interrupt 4 Configuration */
	/* Disables an ADC interrupt source. */
	ADC_disableInterrupt(CONFIG_ADC0_BASE_ADDR, 3);
	/* Sets the source EOC for an analog-to-digital converter interrupt. */
	ADC_setInterruptSource(CONFIG_ADC0_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables continuous mode for an ADC interrupt. */
	ADC_disableContinuousMode(CONFIG_ADC0_BASE_ADDR, 3);


	/* Post Processing Block 1 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC0_BASE_ADDR, 0, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC0_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC0_BASE_ADDR, 0, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC0_BASE_ADDR, 0, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC0_BASE_ADDR, 0, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC0_BASE_ADDR, 0);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC0_BASE_ADDR, 0, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC0_BASE_ADDR, 0);

	/* Post Processing Block 2 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC0_BASE_ADDR, 1, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC0_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC0_BASE_ADDR, 1, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC0_BASE_ADDR, 1, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC0_BASE_ADDR, 1, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC0_BASE_ADDR, 1);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC0_BASE_ADDR, 1, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC0_BASE_ADDR, 1);

	/* Post Processing Block 3 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC0_BASE_ADDR, 2, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC0_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC0_BASE_ADDR, 2, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC0_BASE_ADDR, 2, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC0_BASE_ADDR, 2, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC0_BASE_ADDR, 2);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC0_BASE_ADDR, 2, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC0_BASE_ADDR, 2);

	/* Post Processing Block 4 Configuration */
	/* Configures a post-processing block (PPB) in the ADC. */
	ADC_setupPPB(CONFIG_ADC0_BASE_ADDR, 3, ADC_SOC_NUMBER0);
	/* Disables individual ADC PPB event sources. */
	ADC_disablePPBEvent(CONFIG_ADC0_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Disables individual ADC PPB event interrupt sources. */
	ADC_disablePPBEventInterrupt(CONFIG_ADC0_BASE_ADDR, 3, (ADC_EVT_TRIPHI | ADC_EVT_TRIPLO | ADC_EVT_ZERO));
	/* Sets the post processing block offset correction. */
	ADC_setPPBCalibrationOffset(CONFIG_ADC0_BASE_ADDR, 3, 0);
	/* Sets the post processing block reference offset. */
	ADC_setPPBReferenceOffset(CONFIG_ADC0_BASE_ADDR, 3, 0);
	/* Disables two's complement capability in the PPB. */
	ADC_disablePPBTwosComplement(CONFIG_ADC0_BASE_ADDR, 3);
	/* Sets the windowed trip limits for a PPB. */
	ADC_setPPBTripLimits(CONFIG_ADC0_BASE_ADDR, 3, 0, 0);
    /* Disables cycle by cycle clear of ADC PPB event flags. */
    ADC_disablePPBEventCBCClear(CONFIG_ADC0_BASE_ADDR, 3);

	/* Set SOC burst mode. */
	ADC_setBurstModeConfig(CONFIG_ADC0_BASE_ADDR, ADC_TRIGGER_SW_ONLY, 1);
	/* Disables SOC burst mode. */
	ADC_disableBurstMode(CONFIG_ADC0_BASE_ADDR);
}

void Drivers_dacOpen()
{
    /* CONFIG_DAC0 initialization */
    /* Set DAC reference voltage. */
    DAC_setReferenceVoltage(CONFIG_DAC0_BASE_ADDR, DAC_REF_VREF);
    /* Set DAC load mode. */
    DAC_setLoadMode(CONFIG_DAC0_BASE_ADDR, DAC_LOAD_SYSCLK);
    /* Enable the DAC output */
    DAC_enableOutput(CONFIG_DAC0_BASE_ADDR);
    /* Set the DAC shadow output */
    DAC_setShadowValue(CONFIG_DAC0_BASE_ADDR, 0);
    /* Delay for buffered DAC to power up. */
    ClockP_usleep(10);
}

/*
 * EDMA
 */
/* EDMA Driver handles */
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

/* EDMA Driver Open Parameters */
EDMA_Params gEdmaParams[CONFIG_EDMA_NUM_INSTANCES] =
{
    {
        .intrEnable = TRUE,
    },
};

void Drivers_edmaOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        gEdmaHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        gEdmaHandle[instCnt] = EDMA_open(instCnt, &gEdmaParams[instCnt]);
        if(NULL == gEdmaHandle[instCnt])
        {
            DebugP_logError("EDMA open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_edmaClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_edmaClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        if(gEdmaHandle[instCnt] != NULL)
        {
            EDMA_close(gEdmaHandle[instCnt]);
            gEdmaHandle[instCnt] = NULL;
        }
    }

    return;
}

void Drivers_epwmOpen(void)
{
	/* CONFIG_EPWM7 initialization */

	/* Time Base */
	EPWM_setEmulationMode(CONFIG_EPWM7_BASE_ADDR, EPWM_EMULATION_FREE_RUN);
	EPWM_setClockPrescaler(CONFIG_EPWM7_BASE_ADDR, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(CONFIG_EPWM7_BASE_ADDR, 999);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM7_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
	EPWM_setTimeBaseCounter(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(CONFIG_EPWM7_BASE_ADDR, EPWM_COUNTER_MODE_UP);
	EPWM_setCountModeAfterSync(CONFIG_EPWM7_BASE_ADDR, EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
	EPWM_enablePhaseShiftLoad(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setPhaseShift(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_enableSyncOutPulseSource(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_setSyncInPulseSource(CONFIG_EPWM7_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);
	EPWM_setOneShotSyncOutTrigger(CONFIG_EPWM7_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	

	/* Counter Compare */
	EPWM_setCounterCompareValue(CONFIG_EPWM7_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 850);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM7_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM7_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM7_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM7_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM7_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM7_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM7_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM7_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM7_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM7_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
	EPWM_setTripZoneAction(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZB_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZB_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZA_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZA_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(CONFIG_EPWM7_BASE_ADDR);
	EPWM_enableTripZoneSignals(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZoneInterrupt(CONFIG_EPWM7_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setDigitalCompareBlankingEvent(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, EPWM_DC_WINDOW_START_TBCTR_PERIOD);
	EPWM_setDigitalCompareWindowOffset(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(CONFIG_EPWM7_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(CONFIG_EPWM7_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setValleyTriggerSource(CONFIG_EPWM7_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(CONFIG_EPWM7_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setValleySWDelayValue(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(CONFIG_EPWM7_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM7_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM7_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM7_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM7_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM7_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM7_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(CONFIG_EPWM7_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(CONFIG_EPWM7_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(CONFIG_EPWM7_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM7_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM7_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DB_RED, false);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DB_FED, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(CONFIG_EPWM7_BASE_ADDR, 0);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_setDeadBandCounterClock(CONFIG_EPWM7_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setChopperDutyCycle(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_setChopperFreq(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(CONFIG_EPWM7_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setInterruptSource(CONFIG_EPWM7_BASE_ADDR, EPWM_INT_TBCTR_ZERO, EPWM_INT_TBCTR_ZERO);
	EPWM_setInterruptEventCount(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(CONFIG_EPWM7_BASE_ADDR, 0);
	
	EPWM_enableADCTrigger(CONFIG_EPWM7_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerSource(CONFIG_EPWM7_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA, EPWM_SOC_TBCTR_U_CMPA);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM7_BASE_ADDR, EPWM_SOC_A, 1);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM7_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM7_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_enableADCTrigger(CONFIG_EPWM7_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerSource(CONFIG_EPWM7_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_TBCTR_ZERO, EPWM_SOC_TBCTR_ZERO);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM7_BASE_ADDR, EPWM_SOC_B, 1);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM7_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM7_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(CONFIG_EPWM7_BASE_ADDR);
    EPWM_disableSplitXCMP(CONFIG_EPWM7_BASE_ADDR);
	EPWM_allocAXCMP(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    EPWM_setXCMPLoadMode(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XMINMAX_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XMINMAX_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XMINMAX_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM7_BASE_ADDR, EPWM_XMINMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM7_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(CONFIG_EPWM7_BASE_ADDR);
    EPWM_setDiodeEmulationMode(CONFIG_EPWM7_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(CONFIG_EPWM7_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM7_BASE_ADDR, EPWM_DE_TRIPL, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM7_BASE_ADDR, EPWM_DE_TRIPH, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM7_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM7_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM7_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM7_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(CONFIG_EPWM7_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(CONFIG_EPWM7_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(CONFIG_EPWM7_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM7_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM7_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(CONFIG_EPWM7_BASE_ADDR);
    HRPWM_setMEPControlMode(CONFIG_EPWM7_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(CONFIG_EPWM7_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(CONFIG_EPWM7_BASE_ADDR, 0);
    HRPWM_setSyncPulseSource(CONFIG_EPWM7_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);
    HRPWM_disablePhaseShiftLoad(CONFIG_EPWM7_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM7_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM7_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM7_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM7_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM7_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM7_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(CONFIG_EPWM7_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(CONFIG_EPWM7_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(CONFIG_EPWM7_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(CONFIG_EPWM7_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(CONFIG_EPWM7_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(CONFIG_EPWM7_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(CONFIG_EPWM7_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(CONFIG_EPWM7_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(CONFIG_EPWM7_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

	/* Global Load */
	EPWM_disableGlobalLoad(CONFIG_EPWM7_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(CONFIG_EPWM7_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(CONFIG_EPWM7_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(CONFIG_EPWM7_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(CONFIG_EPWM7_BASE_ADDR, 0);
	/* CONFIG_EPWM0 initialization */

	/* Time Base */
	EPWM_setEmulationMode(CONFIG_EPWM0_BASE_ADDR, EPWM_EMULATION_FREE_RUN);
	EPWM_setClockPrescaler(CONFIG_EPWM0_BASE_ADDR, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(CONFIG_EPWM0_BASE_ADDR, 10000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
	EPWM_setTimeBaseCounter(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_MODE_UP_DOWN);
	EPWM_setCountModeAfterSync(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNT_MODE_UP_AFTER_SYNC);
	EPWM_enablePhaseShiftLoad(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setPhaseShift(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_enableSyncOutPulseSource(CONFIG_EPWM0_BASE_ADDR, EPWM_SYNC_OUT_PULSE_ON_CNTR_ZERO);
	EPWM_setSyncInPulseSource(CONFIG_EPWM0_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_DISABLE);
	EPWM_setOneShotSyncOutTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	

	/* Counter Compare */
	EPWM_setCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 5000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
	EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZB_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZB_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZA_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZA_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(CONFIG_EPWM0_BASE_ADDR);
	EPWM_enableTripZoneSignals(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZoneInterrupt(CONFIG_EPWM0_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setDigitalCompareBlankingEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, EPWM_DC_WINDOW_START_TBCTR_PERIOD);
	EPWM_setDigitalCompareWindowOffset(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(CONFIG_EPWM0_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(CONFIG_EPWM0_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setValleyTriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(CONFIG_EPWM0_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setValleySWDelayValue(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(CONFIG_EPWM0_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM0_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM0_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM0_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(CONFIG_EPWM0_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_RED, true);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_FED, true);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(CONFIG_EPWM0_BASE_ADDR, 200);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(CONFIG_EPWM0_BASE_ADDR, 200);
	EPWM_setDeadBandCounterClock(CONFIG_EPWM0_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setChopperDutyCycle(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_setChopperFreq(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(CONFIG_EPWM0_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setInterruptSource(CONFIG_EPWM0_BASE_ADDR, EPWM_INT_TBCTR_ZERO, EPWM_INT_TBCTR_ZERO);
	EPWM_setInterruptEventCount(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(CONFIG_EPWM0_BASE_ADDR, 0);
	
	EPWM_enableADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO, EPWM_SOC_TBCTR_ZERO);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A, 1);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerSource(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM0_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(CONFIG_EPWM0_BASE_ADDR);
    EPWM_disableSplitXCMP(CONFIG_EPWM0_BASE_ADDR);
	EPWM_allocAXCMP(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    EPWM_setXCMPLoadMode(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMINMAX_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMINMAX_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMINMAX_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM0_BASE_ADDR, EPWM_XMINMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM0_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(CONFIG_EPWM0_BASE_ADDR);
    EPWM_setDiodeEmulationMode(CONFIG_EPWM0_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(CONFIG_EPWM0_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_TRIPL, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_TRIPH, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(CONFIG_EPWM0_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(CONFIG_EPWM0_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(CONFIG_EPWM0_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM0_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(CONFIG_EPWM0_BASE_ADDR);
    HRPWM_setMEPControlMode(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(CONFIG_EPWM0_BASE_ADDR, 0);
    HRPWM_setSyncPulseSource(CONFIG_EPWM0_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);
    HRPWM_disablePhaseShiftLoad(CONFIG_EPWM0_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM0_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM0_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(CONFIG_EPWM0_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(CONFIG_EPWM0_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(CONFIG_EPWM0_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(CONFIG_EPWM0_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(CONFIG_EPWM0_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(CONFIG_EPWM0_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(CONFIG_EPWM0_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(CONFIG_EPWM0_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(CONFIG_EPWM0_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

	/* Global Load */
	EPWM_disableGlobalLoad(CONFIG_EPWM0_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(CONFIG_EPWM0_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(CONFIG_EPWM0_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(CONFIG_EPWM0_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(CONFIG_EPWM0_BASE_ADDR, 0);
	/* CONFIG_EPWM1 initialization */

	/* Time Base */
	EPWM_setEmulationMode(CONFIG_EPWM1_BASE_ADDR, EPWM_EMULATION_FREE_RUN);
	EPWM_setClockPrescaler(CONFIG_EPWM1_BASE_ADDR, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(CONFIG_EPWM1_BASE_ADDR, 10000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
	EPWM_setTimeBaseCounter(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_MODE_UP_DOWN);
	EPWM_setCountModeAfterSync(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNT_MODE_UP_AFTER_SYNC);
	EPWM_enablePhaseShiftLoad(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setPhaseShift(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_enableSyncOutPulseSource(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setSyncInPulseSource(CONFIG_EPWM1_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);
	EPWM_setOneShotSyncOutTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	

	/* Counter Compare */
	EPWM_setCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 5000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_SYNC_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
	EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZB_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZB_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZA_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZA_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(CONFIG_EPWM1_BASE_ADDR);
	EPWM_enableTripZoneSignals(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZoneInterrupt(CONFIG_EPWM1_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setDigitalCompareBlankingEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, EPWM_DC_WINDOW_START_TBCTR_PERIOD);
	EPWM_setDigitalCompareWindowOffset(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(CONFIG_EPWM1_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(CONFIG_EPWM1_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setValleyTriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(CONFIG_EPWM1_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setValleySWDelayValue(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(CONFIG_EPWM1_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM1_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM1_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM1_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(CONFIG_EPWM1_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_RED, true);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_FED, true);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(CONFIG_EPWM1_BASE_ADDR, 200);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(CONFIG_EPWM1_BASE_ADDR, 200);
	EPWM_setDeadBandCounterClock(CONFIG_EPWM1_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setChopperDutyCycle(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setChopperFreq(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(CONFIG_EPWM1_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setInterruptSource(CONFIG_EPWM1_BASE_ADDR, EPWM_INT_TBCTR_ZERO, EPWM_INT_TBCTR_ZERO);
	EPWM_setInterruptEventCount(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(CONFIG_EPWM1_BASE_ADDR, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_A, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerSource(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM1_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(CONFIG_EPWM1_BASE_ADDR);
    EPWM_disableSplitXCMP(CONFIG_EPWM1_BASE_ADDR);
	EPWM_allocAXCMP(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    EPWM_setXCMPLoadMode(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMINMAX_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMINMAX_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMINMAX_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM1_BASE_ADDR, EPWM_XMINMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM1_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(CONFIG_EPWM1_BASE_ADDR);
    EPWM_setDiodeEmulationMode(CONFIG_EPWM1_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(CONFIG_EPWM1_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_TRIPL, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_TRIPH, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(CONFIG_EPWM1_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(CONFIG_EPWM1_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(CONFIG_EPWM1_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM1_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(CONFIG_EPWM1_BASE_ADDR);
    HRPWM_setMEPControlMode(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(CONFIG_EPWM1_BASE_ADDR, 0);
    HRPWM_setSyncPulseSource(CONFIG_EPWM1_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);
    HRPWM_disablePhaseShiftLoad(CONFIG_EPWM1_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM1_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM1_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(CONFIG_EPWM1_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(CONFIG_EPWM1_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(CONFIG_EPWM1_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(CONFIG_EPWM1_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(CONFIG_EPWM1_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(CONFIG_EPWM1_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(CONFIG_EPWM1_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(CONFIG_EPWM1_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(CONFIG_EPWM1_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

	/* Global Load */
	EPWM_disableGlobalLoad(CONFIG_EPWM1_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(CONFIG_EPWM1_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(CONFIG_EPWM1_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(CONFIG_EPWM1_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(CONFIG_EPWM1_BASE_ADDR, 0);
	/* CONFIG_EPWM2 initialization */

	/* Time Base */
	EPWM_setEmulationMode(CONFIG_EPWM2_BASE_ADDR, EPWM_EMULATION_FREE_RUN);
	EPWM_setClockPrescaler(CONFIG_EPWM2_BASE_ADDR, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
	EPWM_setTimeBasePeriod(CONFIG_EPWM2_BASE_ADDR, 10000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_TBPRD_TBPRDHR);
	EPWM_setPeriodLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_PERIOD_SHADOW_LOAD);
	EPWM_setTimeBaseCounter(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setTimeBaseCounterMode(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_MODE_UP_DOWN);
	EPWM_setCountModeAfterSync(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNT_MODE_UP_AFTER_SYNC);
	EPWM_enablePhaseShiftLoad(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setPhaseShift(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_enableSyncOutPulseSource(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setSyncInPulseSource(CONFIG_EPWM2_BASE_ADDR, EPWM_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0);
	EPWM_setOneShotSyncOutTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_OSHT_SYNC_OUT_TRIG_SYNC);
	

	/* Counter Compare */
	EPWM_setCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_A, 5000);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_CMPA_CMPAHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_B, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_CMPB_CMPBHR);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_C, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_CMPC);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_C, EPWM_COMP_LOAD_ON_CNTR_ZERO);
	EPWM_setCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_D, 0);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_CMPD);
	
	EPWM_setCounterCompareShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_COUNTER_COMPARE_D, EPWM_COMP_LOAD_ON_CNTR_ZERO);

	/* Action Qualifier */
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_AQCSFRC);
	EPWM_setActionQualifierContSWForceShadowMode(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_AQCTLA_AQCTLA2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_ACTION_QUALIFIER_A);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_SW_DISABLED);
	EPWM_disableGlobalLoadRegisters(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_REGISTER_AQCTLB_AQCTLB2);
	EPWM_disableActionQualifierShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_ACTION_QUALIFIER_B);
	EPWM_setActionQualifierShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO);
	EPWM_setActionQualifierT1TriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierT2TriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_TRIGGER_EVENT_TRIG_DCA_1);
	EPWM_setActionQualifierSWAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE);
	EPWM_setActionQualifierContSWForceAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_DISABLED);

	/* Events */
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
	EPWM_setActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

	/* Trip Zone */
	EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT1, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCAEVT2, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT1, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ACTION_EVENT_DCBEVT2, EPWM_TZ_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZB_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZB_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZA_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvAction(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_TZA_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionA(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U, EPWM_TZ_ADV_ACTION_HIGH_Z);
	EPWM_setTripZoneAdvDigitalCompareActionB(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D, EPWM_TZ_ADV_ACTION_HIGH_Z);
    EPWM_disableTripZoneAdvAction(CONFIG_EPWM2_BASE_ADDR);
	EPWM_enableTripZoneSignals(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_enableTripZoneSignals(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_selectCycleByCycleTripZoneClearEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_CBC_PULSE_CLR_CNTR_ZERO);
	EPWM_enableTripZoneInterrupt(CONFIG_EPWM2_BASE_ADDR, 0);

	/* Digital Compare */
	EPWM_setDigitalCompareFilterInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_WINDOW_SOURCE_DCAEVT1);
	EPWM_disableDigitalCompareBlankingWindow(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setDigitalCompareBlankingEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_WINDOW_START_TBCTR_PERIOD, EPWM_DC_WINDOW_START_TBCTR_PERIOD);
	EPWM_setDigitalCompareWindowOffset(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setDigitalCompareWindowLength(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_disableDigitalCompareWindowInverseMode(CONFIG_EPWM2_BASE_ADDR);
	EPWM_disableDigitalCompareCounterCapture(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setDigitalCompareCounterShadowMode(CONFIG_EPWM2_BASE_ADDR, false);
	EPWM_disableDigitalCompareEdgeFilter(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setDigitalCompareEdgeFilterMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_EDGEFILT_MODE_RISING);
	EPWM_setDigitalCompareEdgeFilterEdgeCount(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_EDGEFILT_EDGECNT_0);
	EPWM_disableValleyCapture(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setValleyTriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_VALLEY_TRIGGER_EVENT_SOFTWARE);
	
	EPWM_setValleyTriggerEdgeCounts(CONFIG_EPWM2_BASE_ADDR, 0, 0);
	EPWM_disableValleyHWDelay(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setValleySWDelayValue(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setValleyDelayDivider(CONFIG_EPWM2_BASE_ADDR, EPWM_VALLEY_DELAY_MODE_SW_DELAY);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM2_BASE_ADDR, 0, EPWM_DC_TYPE_DCAH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM2_BASE_ADDR, 0, EPWM_DC_TYPE_DCAL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_DC_OUTPUT_A2, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM2_BASE_ADDR, 0, EPWM_DC_TYPE_DCBH);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
	EPWM_enableDigitalCompareTripCombinationInput(CONFIG_EPWM2_BASE_ADDR, 0, EPWM_DC_TYPE_DCBL);
    EPWM_selectDigitalCompareTripInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBL);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DC_DISABLED);
	EPWM_setTripZoneDigitalCompareEventCondition(CONFIG_EPWM2_BASE_ADDR, EPWM_TZ_DC_OUTPUT_B2, EPWM_TZ_EVENT_DC_DISABLED);

	EPWM_disableDigitalCompareADCTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B);
	
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
	EPWM_setDigitalCompareEventSyncMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_INPUT_SYNCED);
	EPWM_setDigitalCompareEventSource(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_A, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);
	EPWM_setDigitalCompareCBCLatchMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_DISABLED);
	EPWM_selectDigitalCompareCBCLatchClearEvent(CONFIG_EPWM2_BASE_ADDR, EPWM_DC_MODULE_B, EPWM_DC_EVENT_2, EPWM_DC_CBC_LATCH_CLR_CNTR_ZERO);

	/* Deadband */
    EPWM_disableDeadBandControlShadowLoadMode(CONFIG_EPWM2_BASE_ADDR);
    EPWM_setDeadBandControlShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_LOAD_ON_CNTR_ZERO);
	EPWM_setRisingEdgeDeadBandDelayInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setFallingEdgeDeadBandDelayInput(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_INPUT_EPWMA);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
	EPWM_setDeadBandDelayPolarity(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_RED, true);
	EPWM_setDeadBandDelayMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_FED, true);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_OUTPUT_A, false);
	EPWM_setDeadBandOutputSwapMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_OUTPUT_B, false);
	
	
	EPWM_disableRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setRisingEdgeDelayCountShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_RED_LOAD_ON_CNTR_ZERO);
    EPWM_setRisingEdgeDelayCount(CONFIG_EPWM2_BASE_ADDR, 200);
	
	EPWM_disableFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setFallingEdgeDelayCountShadowLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_FED_LOAD_ON_CNTR_ZERO);
    EPWM_setFallingEdgeDelayCount(CONFIG_EPWM2_BASE_ADDR, 200);
	EPWM_setDeadBandCounterClock(CONFIG_EPWM2_BASE_ADDR, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

	/* Chopper */
	EPWM_disableChopper(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setChopperDutyCycle(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setChopperFreq(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_setChopperFirstPulseWidth(CONFIG_EPWM2_BASE_ADDR, 0);

	/* Event Trigger */
	EPWM_disableInterrupt(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setInterruptSource(CONFIG_EPWM2_BASE_ADDR, EPWM_INT_TBCTR_ZERO, EPWM_INT_TBCTR_ZERO);
	EPWM_setInterruptEventCount(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_disableInterruptEventCountInit(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setInterruptEventCountInitValue(CONFIG_EPWM2_BASE_ADDR, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_A, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_A, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_A);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_A, 0);
	
	EPWM_disableADCTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerSource(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_B, EPWM_SOC_DCxEVT1, EPWM_SOC_DCxEVT1);
	EPWM_setADCTriggerEventPrescale(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_B, 0);
	EPWM_disableADCTriggerEventCountInit(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_B);
	EPWM_setADCTriggerEventCountInitValue(CONFIG_EPWM2_BASE_ADDR, EPWM_SOC_B, 0);
	

    /* XCMP Mode */
    EPWM_disableXCMPMode(CONFIG_EPWM2_BASE_ADDR);
    EPWM_disableSplitXCMP(CONFIG_EPWM2_BASE_ADDR);
	EPWM_allocAXCMP(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_NONE_CMPA);
    EPWM_setXCMPLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    EPWM_setXCMPLoadMode(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_XLOADCTL_LOADMODE_LOADONCE);
    EPWM_setXCMPShadowLevel(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWLEVEL_0);
    EPWM_setXCMPShadowBufPtrLoadOnce(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, 0);
    EPWM_setXCMPShadowRepeatBufxCount(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, 0);

    /* Write values to Reg */
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP1_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP2_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP3_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP4_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP5_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP6_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP7_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP8_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP1_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP2_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP3_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP4_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP5_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP6_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP7_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP8_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP1_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP2_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP3_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP4_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP5_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP6_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP7_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP8_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP1_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP2_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP3_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP4_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP5_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP6_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP7_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP8_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XTBPRD_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XTBPRD_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XTBPRD_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XTBPRD_SHADOW3, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMINMAX_ACTIVE, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMINMAX_SHADOW1, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMINMAX_SHADOW2, 0);
    EPWM_setXCMPRegValue(CONFIG_EPWM2_BASE_ADDR, EPWM_XMINMAX_SHADOW3, 0);

    /* Events */

    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_ACTIVE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW2, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP1);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP2);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP3);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP4);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP5);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP6);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP7);
    EPWM_setXCMPActionQualifierAction(CONFIG_EPWM2_BASE_ADDR, EPWM_XCMP_SHADOW3, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_XCMP8);

    /* Diode Emulation */
    EPWM_disableDiodeEmulationMode(CONFIG_EPWM2_BASE_ADDR);
    EPWM_setDiodeEmulationMode(CONFIG_EPWM2_BASE_ADDR, EPWM_DIODE_EMULATION_CBC);
    EPWM_setDiodeEmulationReentryDelay(CONFIG_EPWM2_BASE_ADDR, 0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_TRIPL, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0);
    EPWM_configureDiodeEmulationTripSources(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_TRIPH, EPWM_DE_TRIP_SRC_INPUTXBAR_OUT0);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_CHANNEL_A, EPWM_DE_TRIPL);
    EPWM_selectDiodeEmulationPWMsignal(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_SYNC_TRIPHorL);
    EPWM_selectDiodeEmulationTripSignal(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_CHANNEL_B, EPWM_DE_TRIPL);
    EPWM_nobypassDiodeEmulationLogic(CONFIG_EPWM2_BASE_ADDR);
    
    EPWM_disableDiodeEmulationMonitorModeControl(CONFIG_EPWM2_BASE_ADDR);
    EPWM_setDiodeEmulationMonitorCounterThreshold(CONFIG_EPWM2_BASE_ADDR, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_COUNT_DOWN, 0);
    EPWM_setDiodeEmulationMonitorModeStep(CONFIG_EPWM2_BASE_ADDR, EPWM_DE_COUNT_UP, 0);

    /* HRPWM */
    HRPWM_disableAutoConversion(CONFIG_EPWM2_BASE_ADDR);
    HRPWM_setMEPControlMode(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setMEPControlMode(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);
    HRPWM_setHiResPhaseShift(CONFIG_EPWM2_BASE_ADDR, 0);
    HRPWM_setSyncPulseSource(CONFIG_EPWM2_BASE_ADDR, HRPWM_PWMSYNC_SOURCE_PERIOD);
    HRPWM_disablePhaseShiftLoad(CONFIG_EPWM2_BASE_ADDR);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setMEPEdgeSelect(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_DISABLE);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, HRPWM_COUNTER_COMPARE_A, 1);
    HRPWM_setHiResCounterCompareValue(CONFIG_EPWM2_BASE_ADDR, HRPWM_COUNTER_COMPARE_B, 1);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_A, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setCounterCompareShadowLoadEvent(CONFIG_EPWM2_BASE_ADDR, HRPWM_CHANNEL_B, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_disablePeriodControl(CONFIG_EPWM2_BASE_ADDR);
    HRPWM_setHiResTimeBasePeriod(CONFIG_EPWM2_BASE_ADDR, 0);
    HRPWM_setDeadbandMEPEdgeSelect(CONFIG_EPWM2_BASE_ADDR, HRPWM_DB_MEP_CTRL_DISABLE);
    HRPWM_setHiResRisingEdgeDelay(CONFIG_EPWM2_BASE_ADDR, 0);
    HRPWM_setRisingEdgeDelayLoadMode(CONFIG_EPWM2_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setHiResFallingEdgeDelayOnly(CONFIG_EPWM2_BASE_ADDR, 0);
    HRPWM_setFallingEdgeDelayLoadMode(CONFIG_EPWM2_BASE_ADDR, HRPWM_LOAD_ON_CNTR_ZERO);
    HRPWM_setOutputSwapMode(CONFIG_EPWM2_BASE_ADDR, false);
    HRPWM_setChannelBOutputPath(CONFIG_EPWM2_BASE_ADDR, HRPWM_OUTPUT_ON_B_NORMAL);

	/* Global Load */
	EPWM_disableGlobalLoad(CONFIG_EPWM2_BASE_ADDR);
	EPWM_setGlobalLoadTrigger(CONFIG_EPWM2_BASE_ADDR, EPWM_GL_LOAD_PULSE_CNTR_ZERO);
	EPWM_setGlobalLoadEventPrescale(CONFIG_EPWM2_BASE_ADDR, 0);
	EPWM_disableGlobalLoadOneShotMode(CONFIG_EPWM2_BASE_ADDR);
	
	

	/* EPWM Module */
	EPWM_lockRegisters(CONFIG_EPWM2_BASE_ADDR, 0);
}

/*
 * MCSPI
 */
/* MCSPI Driver handles */
MCSPI_Handle gMcspiHandle[CONFIG_MCSPI_NUM_INSTANCES];
/* MCSPI Driver Open Parameters */
MCSPI_OpenParams gMcspiOpenParams[CONFIG_MCSPI_NUM_INSTANCES] =
{
    {
        .transferMode           = MCSPI_TRANSFER_MODE_BLOCKING,
        .transferTimeout        = SystemP_WAIT_FOREVER,
        .transferCallbackFxn    = NULL,
        .msMode                 = MCSPI_MS_MODE_MASTER,
        .mcspiDmaIndex = -1,
    },
};
/* MCSPI Driver Channel Configurations */
MCSPI_ChConfig gConfigMcspiUccChCfg[CONFIG_MCSPI_UCC_NUM_CH] =
{
    {
        .chNum              = MCSPI_CHANNEL_0,
        .frameFormat        = MCSPI_FF_POL0_PHA1,
        .bitRate            = 4000000,
        .csPolarity         = MCSPI_CS_POL_LOW,
        .trMode             = MCSPI_TR_MODE_TX_RX,
        .inputSelect        = MCSPI_IS_D1,
        .dpe0               = MCSPI_DPE_ENABLE,
        .dpe1               = MCSPI_DPE_DISABLE,
        .slvCsSelect        = MCSPI_SLV_CS_SELECT_0,
        .startBitEnable     = FALSE,
        .startBitPolarity   = MCSPI_SB_POL_LOW,
        .csIdleTime         = MCSPI_TCS0_0_CLK,
        .defaultTxData      = 0x0U,
    },
};

#include <drivers/mcspi/v0/dma/mcspi_dma.h>



void Drivers_mcspiOpen(void)
{
    uint32_t instCnt, chCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_MCSPI_NUM_INSTANCES; instCnt++)
    {
        gMcspiHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_MCSPI_NUM_INSTANCES; instCnt++)
    {
        gMcspiHandle[instCnt] = MCSPI_open(instCnt, &gMcspiOpenParams[instCnt]);
        if(NULL == gMcspiHandle[instCnt])
        {
            DebugP_logError("MCSPI open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    /* Channel configuration */
    for(chCnt = 0U; chCnt < CONFIG_MCSPI_UCC_NUM_CH; chCnt++)
    {
        status = MCSPI_chConfig(
                     gMcspiHandle[CONFIG_MCSPI_UCC],
                     &gConfigMcspiUccChCfg[chCnt]);
        if(status != SystemP_SUCCESS)
        {
            DebugP_logError("CONFIG_MCSPI_UCC channel %d config failed !!!\r\n", chCnt);
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_mcspiClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_mcspiClose(void)
{
    uint32_t instCnt;
    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_MCSPI_NUM_INSTANCES; instCnt++)
    {
        if(gMcspiHandle[instCnt] != NULL)
        {
            MCSPI_close(gMcspiHandle[instCnt]);
            gMcspiHandle[instCnt] = NULL;
        }
    }

    return;
}

/*
 * UART
 */

/* UART Driver handles */
UART_Handle gUartHandle[CONFIG_UART_NUM_INSTANCES];

#include <drivers/uart/v0/dma/uart_dma.h>

/* UART Driver Parameters */
UART_Params gUartParams[CONFIG_UART_NUM_INSTANCES] =
{
    {
        .baudRate           = 115200,
        .dataLength         = UART_LEN_8,
        .stopBits           = UART_STOPBITS_1,
        .parityType         = UART_PARITY_NONE,
        .readMode           = UART_TRANSFER_MODE_BLOCKING,
        .readReturnMode     = UART_READ_RETURN_MODE_FULL,
        .writeMode          = UART_TRANSFER_MODE_BLOCKING,
        .readCallbackFxn    = NULL,
        .writeCallbackFxn   = NULL,
        .hwFlowControl      = FALSE,
        .hwFlowControlThr   = UART_RXTRIGLVL_16,
        .transferMode       = UART_CONFIG_MODE_POLLED,
        .skipIntrReg        = FALSE,
        .uartDmaIndex = -1,
        .intrNum            = 38U,
        .intrPriority       = 4U,
        .operMode           = UART_OPER_MODE_16X,
        .rxTrigLvl          = UART_RXTRIGLVL_8,
        .txTrigLvl          = UART_TXTRIGLVL_32,
        .rxEvtNum           = 0U,
        .txEvtNum           = 0U,

    },
};

void Drivers_uartOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = UART_open(instCnt, &gUartParams[instCnt]);
        if(NULL == gUartHandle[instCnt])
        {
            DebugP_logError("UART open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_uartClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_uartClose(void)
{
    uint32_t instCnt;
    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        if(gUartHandle[instCnt] != NULL)
        {
            UART_close(gUartHandle[instCnt]);
            gUartHandle[instCnt] = NULL;
        }
    }

    return;
}

void Drivers_intXbarOpen()
{
    /* INT XBAR */
    SOC_xbarSelectInterruptXBarInputSource(CSL_CONTROLSS_INTXBAR_U_BASE, 0, 0, 0, ( INT_XBAR_ADC4_INT1 ), 0, 0, 0, 0);
}
void Drivers_dmaTrigXbarOpen()
{
    /*
    * DMA TRIGGER XBAR
    */
    SOC_xbarSelectEdmaTrigXbarInputSource(CSL_EDMA_TRIG_XBAR_U_BASE, DMA_TRIG_XBAR_EDMA_MODULE_0, DMA_TRIG_XBAR_EDMA_MODULE_0_INPUT);
}
void Drivers_dmaXbarOpen()
{
    /* DMA XBAR */
    SOC_xbarSelectDMAXBarInputSource(CSL_CONTROLSS_DMAXBAR_U_BASE, 0, 0, DMA_XBAR_EPWM7_SOCA, 0, 0, 0, 0, 0);
}
