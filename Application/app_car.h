#ifndef APP_CAR_H
#define APP_CAR_H

#include <stdint.h>

/*
 * Vehicle application entry points.
 * The interrupt entry calls app_car_on_1ms_tick() only to schedule work;
 * app_car_run_scheduler() runs control/background tasks from the main loop.
 */
void app_car_init(void);
void app_car_run_10ms(void);
void app_car_run_background(void);
void app_car_run_scheduler(void);
void app_car_on_1ms_tick(void);
uint32_t app_car_get_control_overrun(void);

#endif
