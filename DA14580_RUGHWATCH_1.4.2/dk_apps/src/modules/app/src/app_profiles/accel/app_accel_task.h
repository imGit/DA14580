/**
 ****************************************************************************************
 *
 * @file app_accel_task.h
 *
 * @brief Header file - APPACCELTASK.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

#ifndef APP_ACCEL_TASK_H_
#define APP_ACCEL_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APPACCELTASK Task
 * @ingroup APPACCEL
 * @brief Acceleromter Application Task
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwble_config.h"
#include "accel_task.h"

#if BLE_ACCEL

//#define APP_DFLT_DEVICE_NAME    "DA14580-DICE5"


typedef struct {
  short int AXIS_X;
  short int AXIS_Y;
  short int AXIS_Z;
} AxesRaw_t;


void app_accel_create_db_send(void);

int accel_create_db_cfm_handler(ke_msg_id_t const msgid,
                                      struct accel_create_db_cfm const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

int accel_msg_handler(ke_msg_id_t const msgid,
									struct accel_create_db_cfm const *param,
									ke_task_id_t const dest_id,
									ke_task_id_t const src_id);

int accel_start_ind_handler(ke_msg_id_t const msgid,
                                   struct accel_start_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
 
int accel_stop_ind_handler(ke_msg_id_t const msgid,
                                  void const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

int accel_write_line_ind_handler(ke_msg_id_t const msgid,
                                        struct accel_write_line_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id);
                                        
int app_accel_timer_handler(ke_msg_id_t const msgid,
                                   void const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

int app_accel_adv_timer_handler(ke_msg_id_t const msgid,
                                   void const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
                                   
																	 
bool app_db_init_func(void);

void updateData(void);
void updateData_test(void);
																	 
#endif
/// @} APPACCELTASK

#endif //APP_ACCEL_TASK_H_
