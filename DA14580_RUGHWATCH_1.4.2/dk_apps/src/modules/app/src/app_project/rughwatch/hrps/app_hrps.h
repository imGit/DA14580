/**
 ****************************************************************************************
 *
 * @file app_hrps.h
 *
 * @brief Accelerometer Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

#ifndef APP_HRPS_H_
#define APP_HRPS_H_

#if (BLE_ACCEL)
/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Accelerometer Application entry point.
 *
 * @{
 ****************************************************************************************
 */

void app_hrps_create_db(void);

void app_hrps_enable(void);

void app_heart_rate_set_value(uint8_t heart_rate_val);

#endif //(BLE_ACCEL)
#endif
