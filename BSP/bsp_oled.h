#ifndef BSP_OLED_H
#define BSP_OLED_H

#include "../Common/car_types.h"

// Aggregated runtime telemetry shown on the OLED debug pages.
typedef struct {
    uint8_t page;
    line_raw_t line_raw;
    line_bin_t line_bin;
    float line_err;
    float left_pwm;
    float right_pwm;
    float left_speed;
    float right_speed;
    float distance_m;
    float yaw_deg;
    uint16_t segment_id;
    segment_type_t segment_type;
    float dist_to_next_event;
    float current_limit_speed;
} oled_runtime_t;

/* OLED debug display abstraction; hardware refresh is internally throttled. */
void bsp_oled_set_page(uint8_t page);
void bsp_oled_show_runtime(car_mode_t mode, const oled_runtime_t *rt);

#endif
