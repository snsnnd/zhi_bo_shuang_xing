#ifndef BSP_LINE_H
#define BSP_LINE_H

#include "../Common/car_types.h"

void bsp_line_set_raw(line_raw_t raw);
line_raw_t bsp_line_get_raw(void);
line_bin_t bsp_line_get_bin(void);
float bsp_line_get_error(void);

#endif
