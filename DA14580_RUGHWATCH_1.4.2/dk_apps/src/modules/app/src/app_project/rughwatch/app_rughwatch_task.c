/**
****************************************************************************************
*
* @file app_rughwatch_task.c
*
* @brief Battery server application task.
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
 * @addtogroup APPTASK
 * @{
 ****************************************************************************************
 */
 

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_rughwatch_task.h"
#include "rwip_config.h"              
#include "periph_setup.h"

#if (BLE_APP_PRESENT)

#if (BLE_BATT_SERVER)

#include "app_task.h"                  // Application Task API
//#include "bass_task.h"                      
//#include "app_batt.h"
#include "gpio.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#if 0 //gsx 1
/**
 ****************************************************************************************
 * @brief Handles the Create DB confirmation message for the Device Information Service.
 *        Set Advertising Mode
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int batt_create_db_cfm_handler(ke_msg_id_t const msgid,
                                      struct bass_create_db_cfm const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    
    // Inform the Application Manager
    struct app_module_init_cmp_evt *cfm = KE_MSG_ALLOC(APP_MODULE_INIT_CMP_EVT,
                                                           TASK_APP, TASK_APP,
                                                           app_module_init_cmp_evt);

    cfm->status = param->status;

    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

int batt_level_upd_cfm_handler(ke_msg_id_t const msgid,
                                      struct bass_batt_level_upd_cfm const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
	return (KE_MSG_CONSUMED);
}

int batt_level_ntf_cfg_ind_handler(ke_msg_id_t const msgid,
                                      struct bass_batt_level_ntf_cfg_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{

	return (KE_MSG_CONSUMED);
}
#endif //gsx 1

/**
 ****************************************************************************************
 * @brief Handles Battery Level polling timer
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int custom_delay_func(ke_msg_id_t const msgid,
                                   void const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
	static uint8_t led_flag = 0;
////	app_timer_set(APP_BATT_TIMER, dest_id, bat_poll_timeout);
	
	GPIO_ConfigurePin(  LED_B_PORT,  LED_B_PIN, OUTPUT, PID_GPIO, false);
//	if(led_flag==0)
//	{
//		GPIO_ConfigurePin(  LED_B_PORT,  LED_B_PIN, OUTPUT, PID_GPIO, false);
//		led_flag=1;
//	}
//	else
//	{
//		GPIO_ConfigurePin(  LED_B_PORT,  LED_B_PIN, OUTPUT, PID_GPIO, true);
//		led_flag=0;
//	}
//	app_timer_set(APP_CUSTOM_DELAY, TASK_APP, 100);//100=1S 连续触发
}


#if 0//gsx 2
/**
 ****************************************************************************************
 * @brief Handles Battery Alert timer
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int app_batt_alert_timer_handler(ke_msg_id_t const msgid,
                                        void const *param,
										ke_task_id_t const dest_id,
										ke_task_id_t const src_id)
{
		
	
		//Read LED GPIO state
    if(bat_lvl_alert_used)
    {		
#if 0
			if (bat_led_state)
			{
				if(GPIO_GetPinStatus(GPIO_ALERT_PWR_PORT,GPIO_ALERT_PWR_PIN)==false)
				{
					//正在充电
				}
				else
				{
					GPIO_SetActive( bat_led_port, bat_led_pin);  //V3.2版本后将黄灯逻辑电平反向
					bat_led_state = 0;
				}
				app_timer_set(APP_BATT_ALERT_TIMER, dest_id, 50);//20,电池电量低,报警灯闪烁速度,,delay延时
			}
			else
			{
				GPIO_SetInactive( bat_led_port, bat_led_pin);//V3.2版本后将黄灯逻辑电平反向
				bat_led_state = 1;
				app_timer_set(APP_BATT_ALERT_TIMER, dest_id, 50);//5,电池电量低,报警灯闪烁速度,delay延时
			}
#endif
    }
		
    return (KE_MSG_CONSUMED);
}
#endif //gsx 2
#endif //(BLE_BATT_SERVER)

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
