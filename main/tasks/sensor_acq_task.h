
#ifndef __SENSOR_ACQ_TASK_H__
#define __SENSOR_ACQ_TASK_H__

#include "config/app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t sensor_acq_task_create(void);
void sensor_acq_task_notify_wakeup(void);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_ACQ_TASK_H__ */
