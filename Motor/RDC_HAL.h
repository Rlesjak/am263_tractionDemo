/*
 * Copyright (C) 2021-2022 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RDC_HAL_H_
#define RDC_HAL_H_

#include <device.h>

#define adcSOC_num 16
#define resolverExecTable_size 60

extern uint16_t RDCexc_start(uint16_t *table, uint16_t table_size,
                        EDMA_Handle dma_handle, uint32_t dma_ch,
                        uint32_t dac_base);
extern uint16_t RDCexc_update(uint16_t *table, uint16_t table_size,
                        EDMA_Handle dma_handle, uint32_t dma_ch,
                        uint32_t dac_base);

extern uint16_t resolverExecTable[resolverExecTable_size];

extern uint16_t gReadExecTable[adcSOC_num][resolverExecTable_size];
extern uint16_t gReadExecTable_shadow[adcSOC_num][resolverExecTable_size];

extern uint16_t gReadExecTable_ave[resolverExecTable_size];

extern uint16_t gReadExecTable_active[resolverExecTable_size];
extern uint16_t gReadExecTable_active_shadow[resolverExecTable_size];

extern uint16_t gCmpTable_shadow[resolverExecTable_size];

#endif /* RDC_HAL_H_ */
