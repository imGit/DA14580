/**
 ****************************************************************************************
 *
 * @file app_hrps.c
 *
 * @brief Heart Rate Profile server application.
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if BLE_HR_SENSOR
#include "app_task.h"                // application task definitions
#include "hrps_task.h"
//#include "app_batt.h"
#include "gpio.h"
#include "hrps.h" 

#include "periph_setup.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Create Heart Rate Profile Service's Database.
 *
 * @return void
 ****************************************************************************************
 */

void app_hrps_create_db(void)
{
    // Add BASS in the database
    struct hrps_create_db_req * req = KE_MSG_ALLOC(HRPS_CREATE_DB_REQ, TASK_HRPS,
                                                   TASK_APP, hrps_create_db_req);
		
		req->features = 1;

    // Send the message
	//puts("Send HRPS_CREATE_DB_REQ\r\n");
    ke_msg_send(req);
}


/**
 ****************************************************************************************
 * Heart Rate Profile Application Functions
 ****************************************************************************************
 */

void app_hrps_enable(void)
{
    // Allocate the message
    struct hrps_enable_req * req = KE_MSG_ALLOC(HRPS_ENABLE_REQ, TASK_HRPS, TASK_APP,
                                                 hrps_enable_req);
		//结构体填充部分需要后续改善
    // Fill in the parameter structure
    ///Connection handle
    req->conhdl = app_env.conhdl;
    /// security level: b0= nothing, b1=unauthenticated, b2=authenticated, b3=authorized;
    /// b1 or b2 and b3 can go together
    req->sec_lvl = PERM(SVC, ENABLE);
    ///Type of connection - will someday depend on button press length; can be CFG or DISCOVERY
    req->con_type = PRF_CON_NORMAL;// PRF_CON_DISCOVERY;

    /// Heart Rate Notification configuration
    req->hr_meas_ntf_en = 1;//PRF_CLI_START_NTF;//PRF_CLI_STOP_NTFIND

    ///Body Sensor Location
    req->body_sensor_loc = 1;

    // Send the message
    ke_msg_send(req);
}

/**
 ****************************************************************************************
 * @brief Sends Heart Rate measurement value to Service.
 *
 * @param[in] heart_rate_val     heart rate value to send
 *
 * @return void
 ****************************************************************************************
 */

void app_heart_rate_set_value(uint8_t heart_rate_val)
{	
	// Allocate the message
	struct hrps_meas_send_req * req = KE_MSG_ALLOC(HRPS_MEAS_SEND_REQ, TASK_HRPS, TASK_APP,
                                                 hrps_meas_send_req);
		
    // Fill in the parameter structure
    req->conhdl = app_env.conhdl;
	
		//struct hrs_hr_meas meas_val;
	  /// Flag
    req->meas_val.flags = HRS_FLAG_HR_8BITS_VALUE;
    /// RR-Interval numbers (max 4)
    req->meas_val.nb_rr_interval = 1;
    /// RR-Intervals
    req->meas_val.rr_intervals[0] = 0;
    /// Heart Rate Measurement Value
    req->meas_val.heart_rate = heart_rate_val;
    /// Energy Expended
    req->meas_val.energy_expended = HRS_HR_CNTL_POINT_CODE;
	
  // Send the message
	//puts("Send BASS_BATT_LEVEL_UPD_REQ");
	ke_msg_send(req);
		
}




#endif //BLE_HR_SENSOR
/// @} APP
