#ifndef _APP_RUGH_
#define _APP_RUGH_
#include "rwble_config.h"
#include "ke_task.h"

extern int custom_delay_func(ke_msg_id_t const msgid,
                                   void const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);


#endif