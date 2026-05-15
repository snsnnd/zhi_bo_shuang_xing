#ifndef APP_CAR_H
#define APP_CAR_H

/*
 * Vehicle application entry points.
 * app_car_init() prepares all control modules, and app_car_run_10ms()
 * executes one fixed-period control cycle.
 */
void app_car_init(void);
void app_car_run_10ms(void);

#endif
