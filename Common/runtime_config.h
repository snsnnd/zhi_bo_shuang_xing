#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RC_FEATURE_LINE_SENSOR = 0,
    RC_FEATURE_IMU,
    RC_FEATURE_ENCODER,
    RC_FEATURE_OLED,
    RC_FEATURE_MOTOR_OUTPUT,
    RC_FEATURE_SPEED_PID,
    RC_FEATURE_TRACK_MAP_LEARNING,
    RC_FEATURE_MAP_PREDICTION,
    RC_FEATURE_LOST_PROTECTION,
    RC_FEATURE_PID_SCOPE,
    RC_FEATURE_HC05,
    RC_FEATURE_PID_SCOPE_OVER_HC05,
    RC_FEATURE_REMOTE_CONTROL,
    RC_FEATURE_HC05_OLED_TEST,
    RC_FEATURE_HC05_RX_ECHO_TEST,
    RC_FEATURE_COUNT
} runtime_feature_t;

typedef struct {
    bool feature[RC_FEATURE_COUNT];
    uint32_t armed_until_ms[RC_FEATURE_COUNT];
} runtime_config_t;

void runtime_config_init(void);
const runtime_config_t *runtime_config_get(void);
bool runtime_feature_enabled(runtime_feature_t feature);
bool runtime_feature_set(runtime_feature_t feature, bool enabled, uint32_t now_ms);
bool runtime_feature_arm(runtime_feature_t feature, uint32_t now_ms);
bool runtime_feature_from_name(const char *name, runtime_feature_t *feature);
const char *runtime_feature_name(runtime_feature_t feature);
bool runtime_feature_needs_arm(runtime_feature_t feature);

#ifdef __cplusplus
}
#endif

#endif
