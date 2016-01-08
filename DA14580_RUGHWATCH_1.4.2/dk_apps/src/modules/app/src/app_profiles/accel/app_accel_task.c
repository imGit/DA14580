/**
 ****************************************************************************************
 *
 * @file app_task.c
 *
 * @brief RW APP Task implementation
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
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

#include "rwip_config.h"               // SW configuration

#if (BLE_APP_PRESENT) && (BLE_ACCEL)

#include "app_task.h"                  // Application Task API
#include "app.h"                       // Application Definition
#include "gapc_task.h"                 // GAP Controller Task API
#include "gapm_task.h"                 // GAP Manager Task API
#include "gap.h"                       // GAP Definitions
#include "co_error.h"                  // Error Codes Definition
#include "arch.h"                      // Platform Definitions

#if (BLE_APP_HT)
#include "app_ht.h"                    // Application Heath Thermometer Definition
#endif //(BLE_APP_HT)

#if (BLE_APP_DIS)
#include "app_dis.h"                   // Application Device Information Service Definition
#endif //(BLE_APP_DIS)

#if (BLE_ACCEL)
#include "app_accel.h"                 // Application Accelerometer Definition
#include "accel_task.h"
#include "app_accel_task.h"
#endif //(BLE_APP_ACCEL)

#if (BLE_APP_NEB)
#include "app_neb.h"                   // Application Nebulizer Definition
#endif //(BLE_APP_NEB)


#include "app_sec.h"

#include "ke_env.h"
#include "l2cm.h"

#if (BLE_ACCEL)
uint8_t accel_adv_count __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY
uint16_t accel_adv_interval __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY
int8_t update_conn_params __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

uint8_t accel_threshold __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY
uint8_t accel_adv_interval1 __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY
uint8_t accel_adv_interval2 __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY
uint8_t accel_adv_interval3 __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

uint8_t accel_con_interval __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

uint8_t accel_mode __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY
uint8_t accel_latency __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY
uint8_t accel_window __attribute__((section("retention_mem_area0"),zero_init)); //@RETENTION MEMORY

#endif

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if BLE_ACCEL
void app_accel_create_db_send(void)
{
    // Add HTS in the database
    struct accel_create_db_req *req = KE_MSG_ALLOC(ACCEL_CREATE_DB_REQ,
                                                  TASK_ACCEL, TASK_APP,
                                                  accel_create_db_req);

    //req->features = PROXR_IAS_TXPS_SUP;

    ke_msg_send(req);
}

#endif

/**
 ****************************************************************************************
 * @brief Handles start indication from the Proximity Reporter profile.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int accel_create_db_cfm_handler(ke_msg_id_t const msgid,
                                      struct accel_create_db_cfm const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    // If state is not idle, ignore the message
    if (ke_state_get(dest_id) == APP_DB_INIT)
    {
							
//				app_accel_init();
        
        // Inform the Application Manager
        struct app_module_init_cmp_evt *cfm = KE_MSG_ALLOC(APP_MODULE_INIT_CMP_EVT,
                                                           TASK_APP, TASK_APP,
                                                           app_module_init_cmp_evt);

        cfm->status = CO_ERROR_NO_ERROR;    //param->status;

        ke_msg_send(cfm);
			
    }

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles accelerometer start advertising request from the Wakeup ISR.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int accel_msg_handler(ke_msg_id_t const msgid,
									struct accel_create_db_cfm const *param,
									ke_task_id_t const dest_id,
									ke_task_id_t const src_id)
{
//	// If state is not idle, ignore the message
//	if (ke_state_get(dest_id) == APP_CONNECTABLE)
//		app_adv_start();

	return (KE_MSG_CONSUMED);
}

//ACCEL Enable描述符中断接收函数
int accel_start_ind_handler(ke_msg_id_t const msgid,
                                   struct accel_start_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
		//ACCEL Enable
		extern uint8_t ledb_sw;
		ledb_sw=1;
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles accelerometer stop indication from the Accelerometer profile.
 *        No axis of the accelerometer has been enabled through ATT, so upt it in stand by.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int accel_stop_ind_handler(ke_msg_id_t const msgid,
                                  void const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    // Stop the accelerometer
//vm	
//    acc_stop();

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles accelerometer start indication from the Accelerometer profile.
 *        At least one axis of the accelerometer has been enabled through ATT.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int accel_write_line_ind_handler(ke_msg_id_t const msgid,
                                        struct accel_write_line_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    #if PLF_DISPLAY
    // Display the line
    lcd_printf(param->line, param->text);
    #endif //PLF_DISPLAY

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles accelerometer timer
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_accel_timer_handler(ke_msg_id_t const msgid,
                                   void const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
//vm actual accel hardware used	
		
		updateData_test();	//这里是测试函数,上传的内容是虚拟的,如果上传实际数据则调用updateData(data),data是三轴的实际数据
		//updateData(); 
    ke_timer_set(APP_ACCEL_TIMER, TASK_APP, 10);

    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles accelerometer timer for controlling advertising interval
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance .
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

int app_accel_adv_timer_handler(ke_msg_id_t const msgid,
                                   void const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief  Update accelerometer 3-axis data
 *					
 * @param[in] None
 * @param[in] None
 * @param[in] None
 * @param[in] None
 *
 * @return None.
 ****************************************************************************************
 */


void updateData(void)
{
	static AxesRaw_t data={0,0,0};
	//data=   //全局数据
	if (1)//(response & 8) 
	{

//		//response = LIS3DH_GetAccAxesRaw(&data);
//		data.AXIS_X = data.AXIS_X/256;//acc_read_x();
//		data.AXIS_Y = data.AXIS_Y/256;//acc_read_y();
//		data.AXIS_Z = data.AXIS_Z/256;//acc_read_z();
//		
//		if((data.AXIS_X)>50)
//			data.AXIS_X = 50; 
//		if((data.AXIS_X)<-50)
//			data.AXIS_X = -50; 

//		if((data.AXIS_Y)>50)
//			data.AXIS_Y = 50; 
//		if((data.AXIS_Y)<-50)
//			data.AXIS_Y = -50; 

//		if((data.AXIS_Z)>50)
//			data.AXIS_Z = 50; 
//		if((data.AXIS_Z)<-50)
//			data.AXIS_Z = -50; 
		
//		x_val_new = data.AXIS_X;
//		y_val_new = data.AXIS_Y;
//		z_val_new = data.AXIS_Z;

		
    struct accel_value_req *req = KE_MSG_ALLOC(ACCEL_VALUE_REQ, TASK_ACCEL,
                                                TASK_APP, accel_value_req);

		req->accel[0] = data.AXIS_X;//data.AXIS_X/256;//acc_read_x();
    req->accel[1] = data.AXIS_Y;// data.AXIS_Y/256;//acc_read_y();
    req->accel[2] = data.AXIS_Z;// data.AXIS_Z/256;//acc_read_z();

// 		x_val = x_val_new;
// 		y_val = y_val_new;
// 		z_val = z_val_new;
		
    ke_msg_send(req);
	
}
//vm simulated accel reading

}

void updateData_test(void)
{
		static AxesRaw_t axis_value={0x1234,0x0123,0x0012};
    struct accel_value_req *req = KE_MSG_ALLOC(ACCEL_VALUE_REQ, TASK_ACCEL,
                                                TASK_APP, accel_value_req);
		
		req->accel[0] = axis_value.AXIS_X;
    req->accel[1] = axis_value.AXIS_Y;
    req->accel[2] = axis_value.AXIS_Z;
		ke_msg_send(req);
		
		axis_value.AXIS_X--;
		axis_value.AXIS_Y--;
		axis_value.AXIS_Z--;
}


#endif //(BLE_APP_PRESENT)

///// @} APPTASK
