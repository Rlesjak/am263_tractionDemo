//#############################################################################
//
// FILE:  UCC5870.h
//
// TITLE: header file for UCC5870 interface modules
//
//#############################################################################
//
// Copyright (C) 2021-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
//
//#############################################################################

#ifndef UCC5870_H
#define UCC5870_H

#include "device.h"
#include "ucc5870_regs.h"

//
// enumerated variables
//
typedef enum {
    ALL_GOOD      = 0,
    INIT_FAULT    = 1,
    STATUS_FAULT  = 2,
    PRI_RDY_FAULT = 3,
    SEC_RDY_FAULT = 4
} UCC5870_Status_e;

//
// PWM trip status
//
typedef union {
    uint32_t  all;
    struct {
        UCC5870_Status_e UCC5870_status;
        uint16_t  nFLT1L : 1;  //  0    low gate driver fault  - pri side
        uint16_t  nFLT2L : 1;  //  1    low gate driver fault  - sec side
        uint16_t  nFLT1H : 1;  //  2    high gate driver fault - pri side
        uint16_t  nFLT2H : 1;  //  3    high gate driver fault - sec side
        uint16_t  OCPAP  : 1;  //  4    CMPSS --> OCP phase A+
        uint16_t  OCPAN  : 1;  //  5    CMPSS --> OCP phase A-
        uint16_t  OCPBP  : 1;  //  6    CMPSS --> OCP phase B+
        uint16_t  OCPBN  : 1;  //  7    CMPSS --> OCP phase B-
        uint16_t  OCPCP  : 1;  //  8    CMPSS --> OCP phase C+
        uint16_t  OCPCN  : 1;  //  9    CMPSS --> OCP phase C-
        uint16_t  OVP    : 1;  //  10   CMPSS --> OVP DC bus
        uint16_t  UVP    : 1;  //  11   CMPSS --> UVP DC bus

        uint16_t  OVP1   : 1;  //  12   OVP from hot side sense
        uint16_t  OVP2   : 1;  //  13   OVP from cold side sense
        uint16_t  OCP    : 1;  //  14   back up
        uint16_t  OTP    : 1;  //  15   not active
    } fault;
} TripFlagDMC_t;

#define gPWM_NUM 3

extern uint32_t gPWM_base[gPWM_NUM];

//
// function prototypes
//
extern void             Init_UCC5870_Regs(void);
extern UCC5870_Status_e Init_UCC5870(void);
extern uint16_t         diagnose_UCC5870(uint16_t i);
extern UCC5870_Status_e inverterDiagnostics(void);
extern void             clearFaultsUCC5870(void);


#endif
