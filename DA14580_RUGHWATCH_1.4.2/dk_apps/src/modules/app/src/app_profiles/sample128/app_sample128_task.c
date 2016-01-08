/**
****************************************************************************************
*
* @file app_sample128_task.c
*
* Copyright (C) 2014. DGC Technology Co., Ltd. All Rights Reserved.
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup APPTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwble_config.h"              // SW configuration

#if (BLE_APP_PRESENT)

#if (BLE_SAMPLE128)

#include "app_sample128.h"
#include "app_sample128_task.h"
#include "app_task.h"                  // Application Task API
#include "gpio.h"
#include "pwm.h"    
/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles Proximity reporter's profile database creation confirmation.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance .
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int sample128_create_db_cfm_handler(ke_msg_id_t const msgid,
                                      struct sample128_create_db_cfm const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // If state is not idle, ignore the message
    if (ke_state_get(dest_id) == APP_DB_INIT)
    {				
        // Inform the Application Manager
        struct app_module_init_cmp_evt *cfm = KE_MSG_ALLOC(APP_MODULE_INIT_CMP_EVT,
                                                           TASK_APP, TASK_APP,
                                                           app_module_init_cmp_evt);
        cfm->status = param->status;
        ke_msg_send(cfm);			
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles disable indication from the Proximity Reporter profile.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int sample128_disable_ind_handler(ke_msg_id_t const msgid,
                                      struct sample128_disable_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{    	   
    return (KE_MSG_CONSUMED);
}



/**
 ****************************************************************************************
 * @brief Handles Alert indication from proximity reporter profile
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
//modified by gsx
int sample128_val_ind_handler(ke_msg_id_t const msgid,
                                      struct sample128_val_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{     

	attmdb_att_set_value(sample128_env.sample128_shdl+ SAMPLE128_1_IDX_VAL,
                     sizeof(uint8_t), (uint8_t *)&param->val1);
	attmdb_att_set_value(sample128_env.sample128_shdl + SAMPLE128_2_IDX_VAL,
                             sizeof(uint8_t), (uint8_t *)&param->val2);//gsx
 //prf_server_send_event((prf_env_struct *)&sample128_env, false, (sample128_env.sample128_shdl+ SAMPLE128_1_IDX_VAL));		

    return (KE_MSG_CONSUMED);
}
/*
static int spotar_status_update_req_handler (ke_msg_id_t const msgid,
                                         struct spotar_status_upadet_req const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    att_size_t len;
    uint16_t* status_en;
    uint8_t* status_val;
    uint8     status;

    // Read status value from database and compare with new one. 
    // If diferent and notification is enebled, then send notification        
    attmdb_att_get_value(spotar_env.spota_shdl + SPOTA_IDX_PATCH_STATUS_VAL,
                         &len, (uint8_t**)&(status_val));
    attmdb_att_get_value(spotar_env.spota_shdl + SPOTA_IDX_PATCH_STATUS_NTF_CFG,
                         &len, (uint8_t**)&(status_en));        

    status = *status_val;
    // Update the value in the attribute database
    attmdb_att_set_value(spotar_env.spota_shdl + SPOTA_IDX_PATCH_STATUS_VAL,
                     sizeof(uint8_t), (uint8_t *)&param->status);
    
    if ((*status_en) && (param->status != status))
    {
        // Send notification
        prf_server_send_event((prf_env_struct *)&spotar_env, false, (spotar_env.spota_shdl + SPOTA_IDX_PATCH_STATUS_VAL));
    }
          
    return (KE_MSG_CONSUMED);
}


*/


#endif //BLE_PROX_REPORTER

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
