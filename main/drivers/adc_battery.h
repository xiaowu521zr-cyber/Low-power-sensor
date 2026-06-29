
#ifndef __ADC_BATTERY_H__
#define __ADC_BATTERY_H__

#include "config/app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t battery_adc_init(void);
esp_err_t battery_read_voltage(uint32_t *voltage_mv);
uint8_t battery_get_percentage(uint32_t voltage_mv);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_BATTERY_H__ */
