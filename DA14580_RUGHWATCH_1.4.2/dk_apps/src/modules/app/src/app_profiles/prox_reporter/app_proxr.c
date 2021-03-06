/**
****************************************************************************************
*
* @file app_proxr.c
*
* @brief Proximity Reporter application.
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

#include "rwble_config.h"

#if (BLE_APP_PRESENT)
#if (BLE_PROX_REPORTER)

#include "app_api.h"                // application task definitions
#include "proxr_task.h"              // proximity functions
#include "app_proxr.h"
#include "llc_task.h"
#include "gpio.h"
#include "wkupct_quadec.h"
#include "pwm.h"
//application allert state structrure
app_alert_state alert_state __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

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
 * @brief Initialize Proximity Apllication. Ports and interrupts.
 *
 * @return void.
 ****************************************************************************************
 */

void app_proxr_init(GPIO_PORT port, GPIO_PIN pin)
{	
    
    alert_state.port = port;
    alert_state.pin = pin;	
}

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

void app_proxr_enable(void)
{
		
    // Allocate the message
    struct proxr_enable_req * req = KE_MSG_ALLOC(PROXR_ENABLE_REQ, TASK_PROXR, TASK_APP,
                                                 proxr_enable_req);

  	// init application alert state
		app_proxr_alert_stop();
	
    // Fill in the parameter structure
        req->conhdl = app_env.conhdl;
		req->sec_lvl = PERM(SVC, ENABLE);
		req->lls_alert_lvl = (uint8_t) alert_state.ll_alert_lvl;  
		req->txp_lvl = alert_state.txp_lvl; 
	
    // Send the message
    ke_msg_send(req);

}

/**
 ****************************************************************************************
 * @brief Starts proximity apllication alert.
 *
 * @param[in] lvl     Alert level. Mild or High
 *
 * @return void.
 ****************************************************************************************
 */
uint8_t ledb_sw	=	0;
void app_proxr_alert_start(uint8_t lvl)
{
#if 0
	alert_state.lvl = lvl;
	
	if (alert_state.lvl == PROXR_ALERT_MILD)
		alert_state.blink_timeout = 150;
	else
		alert_state.blink_timeout = 50;
	
	alert_state.blink_toggle = 1;
//   GPIO_SetActive( alert_state.port, alert_state.pin);
//	  timer2_enable(TRIPLE_PWM_ENABLED);   
    ke_timer_set(APP_PXP_TIMER, TASK_APP, alert_state.blink_timeout);	

#else
//	timer2_enable(TRIPLE_PWM_ENABLED);//
	switch(lvl){
		case 0:
//			TurnOff_All_Led();
			ledb_sw=0;
		break;
		case 0x01:
			ledb_sw=1;
		break;
		case 0x02:
			ledb_sw=0;
		break;
		case 0x03:

		break;
		case 0x04:

		break;
		case 0x05:

		break;
		case 0x06:

		break;
		case 0x07:

		break;
		case 0x08:

		break;
		case 0x09:

		break;
		case 0x0A:

		break;
		case 0x0B:
//			TurnOn_All_Led();
		break;		
		case 0x0C:
//			TurnOff_All_Led();
		break;
		default:
			break;
	}
	/*
		#ifdef GPIO_ALERT_LED1_PORT
		if (lvl==1)	//大于0时控制灯亮
		GPIO_SetActive( GPIO_ALERT_LED1_PORT, GPIO_ALERT_LED1_PIN);
		#endif
		#ifdef USE_MOTOR
		if (lvl==10)	//大于0时控制灯亮
			 GPIO_SetInactive(GPIO_ALERT_MOTOR_PORT, GPIO_ALERT_MOTOR_PIN);
		#endif
	*/
#endif
}

/**
 ****************************************************************************************
 * @brief Stops proximity apllication alert.
 *
 * @return void.
 ****************************************************************************************
 */

void app_proxr_alert_stop(void)
{

	alert_state.lvl = PROXR_ALERT_NONE; //level;
	
	alert_state.blink_timeout = 0;
	alert_state.blink_toggle = 0;
	
 //  GPIO_SetInactive( alert_state.port, alert_state.pin);
//	  timer2_stop();   
    ke_timer_clear(APP_PXP_TIMER, TASK_APP);
}


/**
 ****************************************************************************************
 * @brief Read Tx Power Level.
 *
 * @return void.
 ****************************************************************************************
 */

void app_proxr_rd_tx_pwr(void)
{
		
    // Allocate the message
    struct llc_rd_tx_pw_lvl_cmd * req = KE_MSG_ALLOC(LLC_RD_TX_PW_LVL_CMP_EVT, TASK_LLC, TASK_APP,
                                                 llc_rd_tx_pw_lvl_cmd);
		
    req->conhdl = app_env.conhdl;
    req->type = TX_LVL_CURRENT;

    // Send the message
    ke_msg_send(req);

}

/**
 ****************************************************************************************
 * @brief Create Proximity Reporter profile database.
 *
 * @return void.
 ****************************************************************************************
 */

void app_proxr_create_db_send(void)
{
    // Add HTS in the database
    struct proxr_create_db_req *req = KE_MSG_ALLOC(PROXR_CREATE_DB_REQ,
                                                  TASK_PROXR, TASK_APP,
                                                  proxr_create_db_req);

    req->features = PROXR_IAS_TXPS_SUP;

    ke_msg_send(req);
}

/**
 ****************************************************************************************
 * @brief Reinit of proximity reporter LED pins and push button. Called by periph_init().
 *
 * @return void.
 ****************************************************************************************
 */

void app_proxr_port_reinit(GPIO_PORT port, GPIO_PIN pin)
{
    app_proxr_init(port, pin);
#if 0
	if(alert_state.blink_toggle == 1){
        GPIO_SetActive( alert_state.port, alert_state.pin);
//		    timer2_enable(TRIPLE_PWM_ENABLED); 
    }
	else{
        GPIO_SetInactive( alert_state.port, alert_state.pin);
//		    timer2_stop();
	}
#endif	
}

#endif //BLE_PROXR
#endif //BLE_APP_PRESENT

/// @} APP
