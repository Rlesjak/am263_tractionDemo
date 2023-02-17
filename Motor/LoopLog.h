//#############################################################################
//
// FILE:  looplog.h
//
// TITLE: header file for loop log
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef MOTOR_LOOPLOG_H_
#define MOTOR_LOOPLOG_H_

volatile uint32_t gLogScaler = 50;
__attribute__ ((section(".tcmb_data"))) uint32_t gLogTicker = 0;
__attribute__ ((section(".tcmb_data"))) uint32_t gLogSkiper = 0;

#define     LOG_CH_NUM  16
__attribute__ ((section(".tcmb_data"))) uint32_t LogChCnt;
__attribute__ ((section(".tcmb_data"))) float32_t *gLog_CH_ptr[LOG_CH_NUM] = {0};
__attribute__ ((section(".tcmb_data"))) float32_t *gLog_ptr[LOG_CH_NUM] = {0};

volatile uint32_t gLogSize = 200;
__attribute__ ((section(".tcmb_data"))) float32_t gLog_CH[LOG_CH_NUM][200] = {0};

static inline void LoopLog_init(void)
{
    for (LogChCnt = 0; LogChCnt<LOG_CH_NUM; LogChCnt++)
    {
        gLog_CH_ptr[LogChCnt] = &gLog_CH[LogChCnt][0];
    }
}
static inline void LoopLog_run(void)
{
    gLogSkiper+=1;
    if (gLogSkiper >= gLogScaler)
    {
        gLogSkiper = 0;

        gLogTicker+=1;

        for (LogChCnt = 0; LogChCnt<LOG_CH_NUM; LogChCnt++)
        {
            *gLog_CH_ptr[LogChCnt] = *gLog_ptr[LogChCnt];
            gLog_CH_ptr[LogChCnt]++;
        }

        if(gLogTicker >= gLogSize)
        {

            for (LogChCnt = 0; LogChCnt<LOG_CH_NUM; LogChCnt++)
            {
                gLog_CH_ptr[LogChCnt] = &gLog_CH[LogChCnt][0];
            }

            gLogTicker = 0;
        }
    }
}

#endif /* MOTOR_LOOPLOG_H_ */
