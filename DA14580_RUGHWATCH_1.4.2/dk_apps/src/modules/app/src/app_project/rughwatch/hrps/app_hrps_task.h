/**
 ****************************************************************************************
 *
 * @file app_hrps_task.h
 *
 * @brief Heart Rate Senser  Service Task header. 
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef APP_HRPS_TASK_H_
#define APP_HRPS_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup RICOW
 *
 * @brief Heart Rate Service Application.
 *
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwble_config.h"

#if (BLE_HR_SENSOR)

//#include "bass_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

int hrps_create_db_cfm_handler(ke_msg_id_t const msgid,
                                      struct bass_create_db_cfm const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

#endif //BLE_HR_SENSOR

/// @} APP

#endif // APP_HR_TASK_H_
