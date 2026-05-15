#include "bsp_motor.h"

static motor_cmd_t g_cmd;

void bsp_motor_set(motor_cmd_t cmd) { g_cmd = cmd; }
motor_cmd_t bsp_motor_get_last(void) { return g_cmd; }
