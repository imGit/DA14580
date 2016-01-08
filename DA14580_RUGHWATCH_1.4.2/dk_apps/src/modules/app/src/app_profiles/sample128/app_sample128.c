/**
****************************************************************************************
*
* @file app_sample.c
*
* @brief Proximity Reporter application.
*
* Copyright (C) 2014. DGC Technology Co., Ltd. All Rights Reserved.
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

#include "rwble_config.h"

#if (BLE_APP_PRESENT)
#if (BLE_PROX_REPORTER)

#include "app_api.h"                // application task definitions
#include "app_sample128_task.h"              // proximity functions
#include "app_sample128.h"
#include "llc_task.h"
#include "gpio.h"
#include "wkupct_quadec.h"
#include "pwm.h"

//application allert state structrure
//app_alert_state alert_state __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/
 
 /**
 ****************************************************************************************
 * Proximity Reporter Application Functions
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Inialize applacition and enable proximity profile.
 *
 * @param[in] type      Alert type. Link Loss or Imediate Alert
 * @param[in] level     Alert level. Mild or High
 *
 * @return void.
 ****************************************************************************************
 */

void app_sample128_enable(void)
{
		
    // Allocate the message
    struct sample128_enable_req * req = KE_MSG_ALLOC(SAMPLE128_ENABLE_REQ, TASK_SAMPLE128, TASK_APP,
                                                 sample128_enable_req);

  	// init application alert state
	//	app_proxr_alert_stop();
	
    // Fill in the parameter structure
    req->conhdl = app_env.conhdl;
		req->sec_lvl = PERM(SVC, ENABLE);
		req->sample128_1_val = 1;  
		req->sample128_2_val = 2; 
	
    // Send the message
    ke_msg_send(req);

}
#if 0
//JPlus Send Message to PC. ---gsx
void app_jplus_send_value(uint8_t value1,uint8_t value2)
{
	attmdb_att_set_value(sample128_env.sample128_shdl + SAMPLE128_1_IDX_VAL,
                             sizeof(uint8_t), (uint8_t *)&value1);
	attmdb_att_set_value(sample128_env.sample128_shdl + SAMPLE128_2_IDX_VAL,
                             sizeof(uint8_t), (uint8_t *)&value2);
}
#endif

/*
struct sample128_enable_req
{
    /// Connection Handle
    uint16_t conhdl;   
    uint8_t sec_lvl;                     /// Security level
    uint8_t sample128_1_val;     /// characteristic 1 value  
    uint8_t sample128_2_val;    /// characteristic 2 value
    /// char 2 Ntf property status
    uint8_t feature;
};

*/

/**
 ****************************************************************************************
 * @brief Create sample128 profile database.
 *
 * @return void.
 ****************************************************************************************
 */

void app_sample128_create_db_send(void)
{
    // Add HTS in the database
    struct sample128_create_db_req *req = KE_MSG_ALLOC(SAMPLE128_CREATE_DB_REQ,
                                                  TASK_SAMPLE128, TASK_APP,
                                                  sample128_create_db_req);

    req->features = SAMPLE128_IAS_TXPS_NOT_SUP;

    ke_msg_send(req);
}



#endif //BLE_PROXR
#endif //BLE_APP_PRESENT

/// @} APP
