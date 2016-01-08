/**
 ****************************************************************************************
 *
 * @file da14580_config.h
 *
 *
 * @brief BLE JPlus project source code .
 *
 * Copyright (C) 2014. DGC Technology Co., Ltd. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef DA14580_CONFIG_H_
#define DA14580_CONFIG_H_

#include "da14580_stack_config.h"

/////////////////////////////////////////////////////////////
/*FullEmbedded - FullHosted*/
#define CFG_APP  
/////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////
/*Profiles*/

#define CFG_PRF_BASS            1
#define CFG_PRF_DISS            1
#define CFG_PRF_PXPR            1
//#define CFG_PRF_FMPL            1
#define CFG_PRF_HRPS						1//gsx
#define CFG_PRF_ACCEL						1//gsx
#define CFG_PRF_SAMPLE128       1 
/////////////////////////////////////////////////////////////

/*Misc*/

/* Application Definition. Project's configuration */
#define CFG_APP_JPLUS

/* NVDS structure */
#define CFG_NVDS 

/* BLE Security  */
#define CFG_APP_SEC

/* Coarse calibration */
#define CFG_LUT_PATCH

/*Watchdog*/
#define CFG_WDOG 

/*Sleep modes*/
#undef CFG_EXT_SLEEP  
//#define  CFG_DEEP_SLEEP  

/*Maximum user connections*/
#define BLE_CONNECTION_MAX_USER 1

/*Build for OTP or JTAG*/
#define DEVELOPMENT__NO_OTP     1       //0: code at OTP, 1: code via JTAG

/*Low power clock selection*/
#define CFG_LP_CLK              0xaa    //0x00: XTAL32, 0xAA: RCX20, 0xFF: Select from OTP Header

/*Fab Calibration - Must be defined for calibrated devices*/
#undef CFG_CALIBRATED_AT_FAB  

/*
 * Scatterfile: Memory maps
 */
#if defined(CFG_EXT_SLEEP) || !defined(CFG_DEEP_SLEEP)
#define REINIT_DESCRIPT_BUF     0       //0: keep in RetRAM, 1: re-init is required (set to 0 when Extended Sleep is used)
#define USE_MEMORY_MAP          EXT_SLEEP_SETUP

#else
#define REINIT_DESCRIPT_BUF     0       //0: keep in RetRAM, 1: re-init is required (set to 0 when Extended Sleep is used)
#define USE_MEMORY_MAP          DEEP_SLEEP_SETUP
#define DB_HEAP_SZ              1024
#define ENV_HEAP_SZ             328
#define MSG_HEAP_SZ             1312
#define NON_RET_HEAP_SZ         1024
#endif

#endif // DA14580_CONFIG_H_
