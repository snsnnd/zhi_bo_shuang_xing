#include "bsp_motor.h"

// Last commanded motor output, kept for diagnostics and desktop simulation.
static motor_cmd_t g_cmd;

// Store the latest differential-drive PWM and direction command.
void bsp_motor_set(motor_cmd_t cmd) { g_cmd = cmd; }

// Return the last command for debug display or unit-level simulation checks.
motor_cmd_t bsp_motor_get_last(void) { return g_cmd; }
