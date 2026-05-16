#include "runtime_config.h"

#include "car_config.h"
#include <string.h>

#define RUNTIME_ARM_WINDOW_MS (5000U)

static runtime_config_t g_runtime_config;

static bool str_eq(const char *a, const char *b) { return strcmp(a, b) == 0; }

static bool runtime_feature_compiled(runtime_feature_t feature) {
    switch (feature) {
    case RC_FEATURE_LINE_SENSOR: return (CAR_ENABLE_LINE_SENSOR != 0U);
    case RC_FEATURE_IMU: return (CAR_ENABLE_IMU != 0U);
    case RC_FEATURE_ENCODER: return (CAR_ENABLE_ENCODER != 0U);
    case RC_FEATURE_OLED: return (CAR_ENABLE_OLED != 0U);
    case RC_FEATURE_MOTOR_OUTPUT: return (CAR_ENABLE_MOTOR_OUTPUT != 0U);
    case RC_FEATURE_SPEED_PID: return (CAR_ENABLE_SPEED_PID != 0U);
    case RC_FEATURE_TRACK_MAP_LEARNING: return (CAR_ENABLE_TRACK_MAP_LEARNING != 0U);
    case RC_FEATURE_MAP_PREDICTION: return (CAR_ENABLE_MAP_PREDICTION != 0U);
    case RC_FEATURE_LOST_PROTECTION: return (CAR_ENABLE_LOST_PROTECTION != 0U);
    case RC_FEATURE_PID_SCOPE: return (CAR_ENABLE_PID_SCOPE != 0U);
    case RC_FEATURE_HC05: return (CAR_ENABLE_HC05 != 0U);
    case RC_FEATURE_PID_SCOPE_OVER_HC05: return (CAR_ENABLE_PID_SCOPE_OVER_HC05 != 0U);
    case RC_FEATURE_REMOTE_CONTROL: return (CAR_ENABLE_REMOTE_CONTROL != 0U);
    case RC_FEATURE_HC05_OLED_TEST: return (CAR_ENABLE_HC05_OLED_TEST != 0U);
    case RC_FEATURE_HC05_RX_ECHO_TEST: return (CAR_ENABLE_HC05_RX_ECHO_TEST != 0U);
    default: return false;
    }
}

void runtime_config_init(void) {
    for (uint8_t i = 0U; i < (uint8_t)RC_FEATURE_COUNT; ++i) {
        g_runtime_config.feature[i] = false;
        g_runtime_config.armed_until_ms[i] = 0U;
    }

    g_runtime_config.feature[RC_FEATURE_LINE_SENSOR] = (CAR_ENABLE_LINE_SENSOR != 0U);
    g_runtime_config.feature[RC_FEATURE_IMU] = (CAR_ENABLE_IMU != 0U);
    g_runtime_config.feature[RC_FEATURE_ENCODER] = (CAR_ENABLE_ENCODER != 0U);
    g_runtime_config.feature[RC_FEATURE_OLED] = (CAR_ENABLE_OLED != 0U);
    g_runtime_config.feature[RC_FEATURE_MOTOR_OUTPUT] = false;
    g_runtime_config.feature[RC_FEATURE_SPEED_PID] = false;
    g_runtime_config.feature[RC_FEATURE_TRACK_MAP_LEARNING] = false;
    g_runtime_config.feature[RC_FEATURE_MAP_PREDICTION] = false;
    g_runtime_config.feature[RC_FEATURE_LOST_PROTECTION] = false;
    g_runtime_config.feature[RC_FEATURE_PID_SCOPE] = false;
    g_runtime_config.feature[RC_FEATURE_HC05] = (CAR_ENABLE_HC05 != 0U);
    g_runtime_config.feature[RC_FEATURE_PID_SCOPE_OVER_HC05] = false;
    g_runtime_config.feature[RC_FEATURE_REMOTE_CONTROL] = (CAR_ENABLE_REMOTE_CONTROL != 0U);
    g_runtime_config.feature[RC_FEATURE_HC05_OLED_TEST] = false;
    g_runtime_config.feature[RC_FEATURE_HC05_RX_ECHO_TEST] = false;
}

const runtime_config_t *runtime_config_get(void) { return &g_runtime_config; }

bool runtime_feature_enabled(runtime_feature_t feature) {
    if (feature >= RC_FEATURE_COUNT) return false;
    return g_runtime_config.feature[feature];
}

bool runtime_feature_needs_arm(runtime_feature_t feature) {
    return feature == RC_FEATURE_MOTOR_OUTPUT || feature == RC_FEATURE_SPEED_PID || feature == RC_FEATURE_TRACK_MAP_LEARNING;
}

bool runtime_feature_arm(runtime_feature_t feature, uint32_t now_ms) {
    if (feature >= RC_FEATURE_COUNT || !runtime_feature_needs_arm(feature)) return false;
    if (!runtime_feature_compiled(feature)) return false;
    g_runtime_config.armed_until_ms[feature] = now_ms + RUNTIME_ARM_WINDOW_MS;
    return true;
}

bool runtime_feature_set(runtime_feature_t feature, bool enabled, uint32_t now_ms) {
    if (feature >= RC_FEATURE_COUNT) return false;
    if (enabled && !runtime_feature_compiled(feature)) return false;
    if (enabled && runtime_feature_needs_arm(feature) && g_runtime_config.armed_until_ms[feature] < now_ms) return false;
    g_runtime_config.feature[feature] = enabled;
    if (!enabled) g_runtime_config.armed_until_ms[feature] = 0U;
    return true;
}

const char *runtime_feature_name(runtime_feature_t feature) {
    static const char *names[RC_FEATURE_COUNT] = {
        "LINE_SENSOR", "IMU", "ENCODER", "OLED", "MOTOR_OUTPUT", "SPEED_PID",
        "TRACK_MAP_LEARNING", "MAP_PREDICTION", "LOST_PROTECTION", "PID_SCOPE",
        "HC05", "PID_SCOPE_OVER_HC05", "REMOTE_CONTROL", "HC05_OLED_TEST", "HC05_RX_ECHO_TEST"
    };
    return feature < RC_FEATURE_COUNT ? names[feature] : "UNKNOWN";
}

bool runtime_feature_from_name(const char *name, runtime_feature_t *feature) {
    if (!name || !feature) return false;
    for (uint8_t i = 0U; i < (uint8_t)RC_FEATURE_COUNT; ++i) {
        if (str_eq(name, runtime_feature_name((runtime_feature_t)i))) {
            *feature = (runtime_feature_t)i;
            return true;
        }
    }
    return false;
}
