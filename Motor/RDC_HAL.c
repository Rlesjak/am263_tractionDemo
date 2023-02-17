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

#include <RDC_HAL.h>


uint16_t resolverExecTable[resolverExecTable_size] =
  { 3548U, 3475U, 3262U, 2930U, 2512U, 2048U, 1584U, 1166U, 834U, 621U,
    548U, 621U, 834U, 1166U, 1584U, 2048U, 2512U, 2930U, 3262U, 3475U,
    3548U, 3475U, 3262U, 2930U, 2512U, 2048U, 1584U, 1166U, 834U, 621U,
    548U, 621U, 834U, 1166U, 1584U, 2048U, 2512U, 2930U, 3262U, 3475U,
    3548U, 3475U, 3262U, 2930U, 2512U, 2048U, 1584U, 1166U, 834U, 621U,
    548U, 621U, 834U, 1166U, 1584U, 2048U, 2512U, 2930U, 3262U, 3475U,};

uint16_t gReadExecTable[adcSOC_num][resolverExecTable_size]={0};
uint16_t gReadExecTable_shadow[adcSOC_num][resolverExecTable_size]={0};

uint16_t gReadExecTable_ave[resolverExecTable_size] = {0};

uint16_t gReadExecTable_active[resolverExecTable_size]={0};
uint16_t gReadExecTable_active_shadow[resolverExecTable_size] = {0};

uint16_t gCmpTable_shadow[resolverExecTable_size] = {0};

uint32_t gRDC_dmabaseAddr, gRDC_dmaregionId;
uint32_t gRDC_dmaCh, gRDC_tcc, gRDC_param0, gRDC_param1;

uint16_t RDCexc_start(uint16_t *table, uint16_t table_size,
                 EDMA_Handle dma_handle, uint32_t dma_ch,
                 uint32_t dac_base)
{

    EDMACCPaRAMEntry   edmaParam1,edmaParam2;
    int32_t             testStatus = SystemP_SUCCESS;

    gRDC_dmabaseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(gRDC_dmabaseAddr != 0);

    gRDC_dmaregionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(gRDC_dmaregionId < SOC_EDMA_NUM_REGIONS);

    gRDC_dmaCh = dma_ch;
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &gRDC_dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    gRDC_tcc = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &gRDC_tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    gRDC_param0 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &gRDC_param0);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    gRDC_param1 = EDMA_RESOURCE_ALLOC_ANY;
    testStatus = EDMA_allocParam(gEdmaHandle[0], &gRDC_param1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /* Request channel */
    EDMA_configureChannelRegion(gRDC_dmabaseAddr, gRDC_dmaregionId, EDMA_CHANNEL_TYPE_DMA,
                                gRDC_dmaCh, gRDC_tcc, gRDC_param0, 0);

    EDMA_disableDmaEvtRegion(gRDC_dmabaseAddr, gRDC_dmaregionId, gRDC_dmaCh);
    /* Disable the interrupt for the channel to transfer in polled mode */
    EDMA_disableEvtIntrRegion(gRDC_dmabaseAddr, gRDC_dmaregionId, gRDC_dmaCh);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam1);
    edmaParam1.srcAddr       = (uint32_t) SOC_virtToPhy((void *)table);
    edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy((void *)(dac_base+CSL_DAC_DACVALS));
    edmaParam1.aCnt          = (uint16_t) 2;
    edmaParam1.bCnt          = (uint16_t) table_size;
    edmaParam1.cCnt          = (uint16_t) 1;
    edmaParam1.bCntReload    = 0;
    edmaParam1.srcBIdx       = (int16_t) 2;
    edmaParam1.destBIdx      = (int16_t) 0;
    edmaParam1.srcCIdx       = (int16_t) 0;
    edmaParam1.destCIdx      = (int16_t) 0;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.opt           = (((((uint32_t)gRDC_tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_ccPaRAMEntry_init(&edmaParam2);
    edmaParam2.srcAddr       = (uint32_t) SOC_virtToPhy((void *)table);
    edmaParam2.destAddr      = (uint32_t) SOC_virtToPhy((void *)(dac_base+CSL_DAC_DACVALS));
    edmaParam2.aCnt          = (uint16_t) 2;
    edmaParam2.bCnt          = (uint16_t) table_size;
    edmaParam2.cCnt          = (uint16_t) 1;
    edmaParam2.bCntReload    = 0;
    edmaParam2.srcBIdx       = (int16_t) 2;
    edmaParam2.destBIdx      = (int16_t) 0;
    edmaParam2.srcCIdx       = (int16_t) 0;
    edmaParam2.destCIdx      = (int16_t) 0;
    edmaParam2.linkAddr      = 0xFFFFU;
    edmaParam2.opt           = (((((uint32_t)gRDC_tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(gRDC_dmabaseAddr, gRDC_param0, &edmaParam1);
    EDMA_setPaRAM(gRDC_dmabaseAddr, gRDC_param1, &edmaParam2);

    EDMA_linkChannel(gRDC_dmabaseAddr, gRDC_param0, gRDC_param1);
    EDMA_linkChannel(gRDC_dmabaseAddr, gRDC_param1, gRDC_param1);

    EDMA_enableTransferRegion(gRDC_dmabaseAddr, gRDC_dmaregionId, gRDC_dmaCh,
                              EDMA_TRIG_MODE_EVENT);

    return testStatus;
}

uint16_t RDCexc_update(uint16_t *table, uint16_t table_size,
                 EDMA_Handle dma_handle, uint32_t dma_ch,
                 uint32_t dac_base)
{
    EDMACCPaRAMEntry   edmaParam1,edmaParam2;
    int32_t             testStatus = SystemP_SUCCESS;

    EDMA_disableDmaEvtRegion(gRDC_dmabaseAddr, gRDC_dmaregionId, gRDC_dmaCh);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam1);
    edmaParam1.srcAddr       = (uint32_t) SOC_virtToPhy((void *)table);
    edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy((void *)(dac_base+CSL_DAC_DACVALS));
    edmaParam1.aCnt          = (uint16_t) 2;
    edmaParam1.bCnt          = (uint16_t) table_size;
    edmaParam1.cCnt          = (uint16_t) 1;
    edmaParam1.bCntReload    = 0;
    edmaParam1.srcBIdx       = (int16_t) 2;
    edmaParam1.destBIdx      = (int16_t) 0;
    edmaParam1.srcCIdx       = (int16_t) 0;
    edmaParam1.destCIdx      = (int16_t) 0;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.opt           = (((((uint32_t)gRDC_tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_ccPaRAMEntry_init(&edmaParam2);
    edmaParam2.srcAddr       = (uint32_t) SOC_virtToPhy((void *)table);
    edmaParam2.destAddr      = (uint32_t) SOC_virtToPhy((void *)(dac_base+CSL_DAC_DACVALS));
    edmaParam2.aCnt          = (uint16_t) 2;
    edmaParam2.bCnt          = (uint16_t) table_size;
    edmaParam2.cCnt          = (uint16_t) 1;
    edmaParam2.bCntReload    = 0;
    edmaParam2.srcBIdx       = (int16_t) 2;
    edmaParam2.destBIdx      = (int16_t) 0;
    edmaParam2.srcCIdx       = (int16_t) 0;
    edmaParam2.destCIdx      = (int16_t) 0;
    edmaParam2.linkAddr      = 0xFFFFU;
    edmaParam2.opt           = (((((uint32_t)gRDC_tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));

    EDMA_setPaRAM(gRDC_dmabaseAddr, gRDC_param0, &edmaParam1);
    EDMA_setPaRAM(gRDC_dmabaseAddr, gRDC_param1, &edmaParam2);

    EDMA_linkChannel(gRDC_dmabaseAddr, gRDC_param0, gRDC_param1);
    EDMA_linkChannel(gRDC_dmabaseAddr, gRDC_param1, gRDC_param1);

    EDMA_enableTransferRegion(gRDC_dmabaseAddr, gRDC_dmaregionId, gRDC_dmaCh,
                              EDMA_TRIG_MODE_EVENT);

    return testStatus;
}
