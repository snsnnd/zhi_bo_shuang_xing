#include "bsp_oled.h"
#include <stdio.h>

static uint8_t g_page;

void bsp_oled_set_page(uint8_t page) { g_page = (uint8_t)(page % 3U); }

void bsp_oled_show_runtime(car_mode_t mode, const oled_runtime_t *rt) {
    if (!rt) return;
    if (g_page == 0U) {
        printf("P1 M:%d Lraw %.2f %.2f %.2f %.2f Bin %d%d%d%d Err %.2f\n", (int)mode,
               rt->line_raw.l2, rt->line_raw.l1, rt->line_raw.r1, rt->line_raw.r2,
               rt->line_bin.l2, rt->line_bin.l1, rt->line_bin.r1, rt->line_bin.r2, rt->line_err);
    } else if (g_page == 1U) {
        printf("P2 PWM %.1f %.1f Spd %.2f %.2f Dist %.2f Yaw %.1f\n", rt->left_pwm, rt->right_pwm,
               rt->left_speed, rt->right_speed, rt->distance_m, rt->yaw_deg);
    } else {
        printf("P3 Seg %u Type %d Next %.2f Vlim %.2f\n", rt->segment_id, (int)rt->segment_type,
               rt->dist_to_next_event, rt->current_limit_speed);
    }
}
