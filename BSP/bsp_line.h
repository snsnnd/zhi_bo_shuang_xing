#ifndef BSP_LINE_H
#define BSP_LINE_H

#include "../Common/car_types.h"

/*
 * Four-channel line sensor abstraction.
 * The setter accepts normalized raw values, while getters expose filtered
 * analog values, binary states, and weighted line error for PID control.
 */
void bsp_line_set_raw(line_raw_t raw);
line_raw_t bsp_line_get_raw(void);
line_bin_t bsp_line_get_bin(void);
float bsp_line_get_error(void);

#endif
