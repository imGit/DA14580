/**
 ****************************************************************************************
 *
 * @file app_sample128.h
 *
 * Copyright (C) 2014. DGC Technology Co., Ltd. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef APP_SAMPLE128_H_
#define APP_SAMPLE128_H_


#include "rwble_config.h"
#include "gpio.h"

#if BLE_SAMPLE128
#include "sample128.h"



/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */



/**
 ****************************************************************************************
 *
 * Proximity Application Functions
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable the proximity profile
 *
 ****************************************************************************************
 */
void app_sample128_enable(void);


/**
 ****************************************************************************************
 * @brief Create proximity reporter Database
 *
 ****************************************************************************************
 */

void app_sample128_create_db_send(void);


#endif //BLE_SAMPLE

/// @} APP

#endif // APP_H_

