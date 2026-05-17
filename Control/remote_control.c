#include "remote_control.h"

#include "../Common/car_config.h"
#include "../Common/runtime_config.h"
#if CAR_ENABLE_PID_SCOPE
#include "../Debug/PIDScope/pid_scope_adapter.h"
#endif
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RC_LINE_MAX 96U

static remote_control_port_t g_port;
static remote_control_state_t g_state;
static char g_line[RC_LINE_MAX];
static uint8_t g_len;
static bool g_collecting;

static uint32_t rc_time_ms(void) { return g_port.get_time_ms ? g_port.get_time_ms() : HAL_GetTick(); }

static void rc_reply(const char *text) {
    if (g_port.write_text && text) (void)g_port.write_text(text);
}

static void mark_cmd_seen(void) { g_state.last_cmd_ms = rc_time_ms(); }

static char upper_char(char c) {
    return (c >= 'a' && c <= 'z') ? (char)(c - ('a' - 'A')) : c;
}

static bool is_cmd_start(uint8_t byte) {
    return byte == '?' || (byte >= 'A' && byte <= 'Z') || (byte >= 'a' && byte <= 'z');
}

static bool is_printable_ascii(uint8_t byte) { return byte >= 32U && byte <= 126U; }

static void upper_in_place(char *s) {
    while (*s) {
        *s = upper_char(*s);
        s++;
    }
}

static bool parse_on_off(const char *s) {
    return (strcmp(s, "1") == 0 || strcmp(s, "ON") == 0 || strcmp(s, "START") == 0 || strcmp(s, "ENABLE") == 0);
}

static bool parse_mode(const char *s, car_mode_t *mode) {
    if (strcmp(s, "IDLE") == 0) *mode = CAR_MODE_IDLE;
    else if (strcmp(s, "CALIB") == 0 || strcmp(s, "CALIBRATION") == 0) *mode = CAR_MODE_CALIBRATION;
    else if (strcmp(s, "LEARN") == 0 || strcmp(s, "LEARNING") == 0) *mode = CAR_MODE_LEARNING;
    else if (strcmp(s, "TRACK") == 0 || strcmp(s, "TRACKING") == 0) *mode = CAR_MODE_TRACKING;
    else if (strcmp(s, "PREDICT") == 0 || strcmp(s, "PREDICTION") == 0) *mode = CAR_MODE_PREDICTION;
    else if (strcmp(s, "HAIRPIN") == 0) *mode = CAR_MODE_HAIRPIN;
    else if (strcmp(s, "LOST") == 0) *mode = CAR_MODE_LOST;
    else if (strcmp(s, "PROTECT") == 0) *mode = CAR_MODE_PROTECT;
    else return false;
    return true;
}

static remote_pid_target_t parse_pid_target(const char *s) {
    if (strcmp(s, "LINE") == 0) return REMOTE_PID_LINE;
    if (strcmp(s, "LEFT") == 0 || strcmp(s, "LSPD") == 0 || strcmp(s, "SPEED_LEFT") == 0) return REMOTE_PID_SPEED_LEFT;
    if (strcmp(s, "RIGHT") == 0 || strcmp(s, "RSPD") == 0 || strcmp(s, "SPEED_RIGHT") == 0) return REMOTE_PID_SPEED_RIGHT;
    if (strcmp(s, "SPEED") == 0 || strcmp(s, "BOTH") == 0 || strcmp(s, "SPEED_BOTH") == 0) return REMOTE_PID_SPEED_BOTH;
    return REMOTE_PID_NONE;
}

static bool parse_motor_test_mode(const char *s, remote_motor_test_mode_t *mode) {
    if (!s || !mode) return false;
    if (strcmp(s, "LEFT") == 0 || strcmp(s, "L") == 0) *mode = REMOTE_MOTOR_TEST_LEFT;
    else if (strcmp(s, "RIGHT") == 0 || strcmp(s, "R") == 0) *mode = REMOTE_MOTOR_TEST_RIGHT;
    else if (strcmp(s, "BOTH") == 0 || strcmp(s, "ALL") == 0) *mode = REMOTE_MOTOR_TEST_BOTH;
    else if (strcmp(s, "STOP") == 0 || strcmp(s, "OFF") == 0 || strcmp(s, "0") == 0) *mode = REMOTE_MOTOR_TEST_NONE;
    else return false;
    return true;
}

static bool parse_speed_tune_mode(const char *s, remote_speed_tune_mode_t *mode) {
    if (!s || !mode) return false;
    if (strcmp(s, "LEFT") == 0 || strcmp(s, "L") == 0) *mode = REMOTE_SPEED_TUNE_LEFT;
    else if (strcmp(s, "RIGHT") == 0 || strcmp(s, "R") == 0) *mode = REMOTE_SPEED_TUNE_RIGHT;
    else if (strcmp(s, "BOTH") == 0 || strcmp(s, "SPEED") == 0 || strcmp(s, "ALL") == 0) *mode = REMOTE_SPEED_TUNE_BOTH;
    else if (strcmp(s, "STOP") == 0 || strcmp(s, "OFF") == 0 || strcmp(s, "0") == 0) *mode = REMOTE_SPEED_TUNE_NONE;
    else return false;
    return true;
}

static const char *speed_tune_mode_name(remote_speed_tune_mode_t mode) {
    if (mode == REMOTE_SPEED_TUNE_LEFT) return "LEFT";
    if (mode == REMOTE_SPEED_TUNE_RIGHT) return "RIGHT";
    if (mode == REMOTE_SPEED_TUNE_BOTH) return "BOTH";
    return "OFF";
}

static bool is_delim(char c) { return c == ' ' || c == ',' || c == '\t'; }

static char *next_token(char **ctx) {
    char *s = *ctx;
    while (*s && is_delim(*s)) s++;
    if (*s == '\0') {
        *ctx = s;
        return 0;
    }
    char *token = s;
    while (*s && !is_delim(*s)) s++;
    if (*s) {
        *s = '\0';
        s++;
    }
    *ctx = s;
    return token;
}

static void reply_status(void) {
    char buf[128];
    snprintf(buf, sizeof(buf), "$STATUS,MOTOR,%u,STOP,%u,SPEED,%.3f,LIMIT,%.3f,MODE,%u,TIMEOUT,%lu\r\n",
             g_state.motor_enable ? 1U : 0U,
             g_state.force_stop ? 1U : 0U,
             (double)g_state.target_speed_mps,
             (double)g_state.speed_limit_mps,
             (unsigned)g_state.mode,
             (unsigned long)g_state.timeout_ms);
    rc_reply(buf);
}

static void reply_help(void) {
    rc_reply("$HELP PING|START|STOP|AUTO|MOTOR 0/1|MOTORTEST LEFT/RIGHT/BOTH <pwm>|TUNE LEFT/RIGHT/BOTH <mps>|ENC?|SCOPE <dev> <ch>|SCOPE SPEED|SCOPE PERIOD <ms>|SCOPE RATE <hz>|MODE <name>|SPEED <mps>|LIMIT <mps>|CFG ?|CFG LIST|PID <target> <kp> <ki> <kd>|STATUS\r\n");
}

static void reply_config(void) {
    char buf[220];
    snprintf(buf, sizeof(buf),
             "$CFG,HC05,%u,REMOTE,%u,MOTOR_OUT,%u,SPEED_PID,%u,PID_SCOPE,%u,PID_HC05,%u,OLED,%u,LINE,%u,IMU,%u,ENC,%u,TIMEOUT,%lu,LIMIT,%.3f,SPEED,%.3f\r\n",
             (unsigned)runtime_feature_enabled(RC_FEATURE_HC05),
             (unsigned)runtime_feature_enabled(RC_FEATURE_REMOTE_CONTROL),
             (unsigned)runtime_feature_enabled(RC_FEATURE_MOTOR_OUTPUT),
             (unsigned)runtime_feature_enabled(RC_FEATURE_SPEED_PID),
             (unsigned)runtime_feature_enabled(RC_FEATURE_PID_SCOPE),
             (unsigned)runtime_feature_enabled(RC_FEATURE_PID_SCOPE_OVER_HC05),
             (unsigned)runtime_feature_enabled(RC_FEATURE_OLED),
             (unsigned)runtime_feature_enabled(RC_FEATURE_LINE_SENSOR),
             (unsigned)runtime_feature_enabled(RC_FEATURE_IMU),
             (unsigned)runtime_feature_enabled(RC_FEATURE_ENCODER),
             (unsigned long)g_state.timeout_ms,
             (double)g_state.speed_limit_mps,
             (double)g_state.target_speed_mps);
    rc_reply(buf);
}

static void reply_feature(runtime_feature_t feature) {
    char buf[80];
    snprintf(buf, sizeof(buf), "$CFG,FEATURE,%s,%u,ARM,%u\r\n",
             runtime_feature_name(feature),
             runtime_feature_enabled(feature) ? 1U : 0U,
             runtime_feature_needs_arm(feature) ? 1U : 0U);
    rc_reply(buf);
}

static void reply_feature_list(void) {
    uint8_t i;
    for (i = 0U; i < (uint8_t)RC_FEATURE_COUNT; ++i) {
        reply_feature((runtime_feature_t)i);
    }
    rc_reply("$OK,CFG,LIST\r\n");
}

static void set_speed_limit(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > SPEED_HIGH_MPS) v = SPEED_HIGH_MPS;
    g_state.speed_limit_mps = v;
    g_state.speed_limit_override = true;
    if (g_state.target_speed_mps > v) g_state.target_speed_mps = v;
}

static void set_target_speed(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > g_state.speed_limit_mps) v = g_state.speed_limit_mps;
    g_state.target_speed_mps = v;
    g_state.speed_override = true;
    g_state.force_stop = false;
}

static void parse_config(char **ctx) {
    char *name = next_token(ctx);
    if (!name || strcmp(name, "?") == 0 || strcmp(name, "ALL") == 0) {
        reply_config();
        return;
    }

    if (strcmp(name, "LIST") == 0 || strcmp(name, "FEATURES") == 0) {
        reply_feature_list();
    } else if (strcmp(name, "TIMEOUT") == 0) {
        char *arg = next_token(ctx);
        if (!arg) {
            rc_reply("$ERR,CFG_TIMEOUT_ARG\r\n");
            return;
        }
        uint32_t timeout = (uint32_t)atoi(arg);
        if (timeout < 100U) timeout = 100U;
        if (timeout > 10000U) timeout = 10000U;
        g_state.timeout_ms = timeout;
        rc_reply("$OK,CFG,TIMEOUT\r\n");
    } else if (strcmp(name, "LIMIT") == 0 || strcmp(name, "MAXSPEED") == 0) {
        char *arg = next_token(ctx);
        if (!arg) {
            rc_reply("$ERR,CFG_LIMIT_ARG\r\n");
            return;
        }
        set_speed_limit((float)atof(arg));
        rc_reply("$OK,CFG,LIMIT\r\n");
    } else if (strcmp(name, "SPEED") == 0) {
        char *arg = next_token(ctx);
        if (!arg) {
            rc_reply("$ERR,CFG_SPEED_ARG\r\n");
            return;
        }
        set_target_speed((float)atof(arg));
        rc_reply("$OK,CFG,SPEED\r\n");
    } else if (strcmp(name, "ARM") == 0) {
        char *feature_name = next_token(ctx);
        runtime_feature_t feature;
        if (!runtime_feature_from_name(feature_name, &feature) || !runtime_feature_arm(feature, rc_time_ms())) {
            rc_reply("$ERR,CFG_ARM_ARG\r\n");
            return;
        }
        rc_reply("$OK,CFG,ARM\r\n");
    } else if (strcmp(name, "SET") == 0) {
        char *feature_name = next_token(ctx);
        char *value = next_token(ctx);
        runtime_feature_t feature;
        if (!runtime_feature_from_name(feature_name, &feature) || !value) {
            rc_reply("$ERR,CFG_SET_ARG\r\n");
            return;
        }
        if (!runtime_feature_set(feature, parse_on_off(value), rc_time_ms())) {
            rc_reply("$ERR,CFG_NEED_ARM\r\n");
            return;
        }
        rc_reply("$OK,CFG,SET\r\n");
    } else if (strcmp(name, "GET") == 0) {
        char *feature_name = next_token(ctx);
        runtime_feature_t feature;
        if (!runtime_feature_from_name(feature_name, &feature)) {
            rc_reply("$ERR,CFG_GET_ARG\r\n");
            return;
        }
        reply_feature(feature);
    } else {
        rc_reply("$ERR,CFG_NAME\r\n");
    }
}

static void parse_line(char *line) {
    mark_cmd_seen();

    if (line[0] == '$') line++;
    upper_in_place(line);

    char *ctx = line;
    char *cmd = next_token(&ctx);
    if (!cmd) return;

    if (strcmp(cmd, "PING") == 0) {
        rc_reply("$OK,PONG\r\n");
    } else if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
        reply_help();
    } else if (strcmp(cmd, "START") == 0 || strcmp(cmd, "RUN") == 0) {
        g_state.motor_enable = true;
        g_state.force_stop = false;
        rc_reply("$OK,START\r\n");
    } else if (strcmp(cmd, "STOP") == 0) {
        g_state.force_stop = true;
        g_state.motor_enable = false;
        g_state.motor_test_mode = REMOTE_MOTOR_TEST_NONE;
        g_state.speed_tune_mode = REMOTE_SPEED_TUNE_NONE;
        g_state.motor_test_pwm = 0.0f;
        rc_reply("$OK,STOP\r\n");
    } else if (strcmp(cmd, "AUTO") == 0) {
        g_state.force_stop = false;
        g_state.speed_override = false;
        g_state.mode_override = false;
        g_state.speed_tune_mode = REMOTE_SPEED_TUNE_NONE;
        rc_reply("$OK,AUTO\r\n");
    } else if (strcmp(cmd, "MOTOR") == 0) {
        char *arg = next_token(&ctx);
        if (!arg) {
            rc_reply("$ERR,MOTOR_ARG\r\n");
            return;
        }
        g_state.motor_enable = parse_on_off(arg);
        if (g_state.motor_enable) g_state.force_stop = false;
        rc_reply(g_state.motor_enable ? "$OK,MOTOR,1\r\n" : "$OK,MOTOR,0\r\n");
    } else if (strcmp(cmd, "SPEED") == 0) {
        char *arg = next_token(&ctx);
        if (!arg) {
            rc_reply("$ERR,SPEED_ARG\r\n");
            return;
        }
        set_target_speed((float)atof(arg));
        g_state.speed_tune_mode = REMOTE_SPEED_TUNE_NONE;
        rc_reply("$OK,SPEED\r\n");
    } else if (strcmp(cmd, "LIMIT") == 0 || strcmp(cmd, "MAXSPEED") == 0) {
        char *arg = next_token(&ctx);
        if (!arg) {
            rc_reply("$ERR,LIMIT_ARG\r\n");
            return;
        }
        set_speed_limit((float)atof(arg));
        rc_reply("$OK,LIMIT\r\n");
    } else if (strcmp(cmd, "MODE") == 0) {
        char *arg = next_token(&ctx);
        car_mode_t mode;
        if (!arg || !parse_mode(arg, &mode)) {
            rc_reply("$ERR,MODE_ARG\r\n");
            return;
        }
        g_state.mode = mode;
        g_state.mode_override = true;
        g_state.force_stop = (mode == CAR_MODE_PROTECT);
        rc_reply("$OK,MODE\r\n");
    } else if (strcmp(cmd, "PID") == 0) {
        char *target = next_token(&ctx);
        char *kp = next_token(&ctx);
        char *ki = next_token(&ctx);
        char *kd = next_token(&ctx);
        remote_pid_target_t pid_target = target ? parse_pid_target(target) : REMOTE_PID_NONE;
        if (pid_target == REMOTE_PID_NONE || !kp || !ki || !kd) {
            rc_reply("$ERR,PID_ARG\r\n");
            return;
        }
        g_state.pid_update.target = pid_target;
        g_state.pid_update.kp = (float)atof(kp);
        g_state.pid_update.ki = (float)atof(ki);
        g_state.pid_update.kd = (float)atof(kd);
        g_state.pid_update.pending = true;
        rc_reply("$OK,PID\r\n");
    } else if (strcmp(cmd, "MOTORTEST") == 0 || strcmp(cmd, "MTEST") == 0) {
        char *mode_arg = next_token(&ctx);
        char *pwm_arg = next_token(&ctx);
        remote_motor_test_mode_t mode;
        float pwm;
        if (!mode_arg) {
            rc_reply("$ERR,MOTORTEST_ARG\r\n");
            return;
        }
        if (!parse_motor_test_mode(mode_arg, &mode)) {
            rc_reply("$ERR,MOTORTEST_ARG\r\n");
            return;
        }
        if (mode == REMOTE_MOTOR_TEST_NONE) {
            g_state.motor_test_mode = REMOTE_MOTOR_TEST_NONE;
            g_state.speed_tune_mode = REMOTE_SPEED_TUNE_NONE;
            g_state.motor_test_pwm = 0.0f;
            g_state.motor_enable = false;
            g_state.force_stop = true;
            rc_reply("$OK,MOTORTEST,STOP\r\n");
            return;
        }
        if (!pwm_arg) {
            rc_reply("$ERR,MOTORTEST_PWM\r\n");
            return;
        }
        pwm = (float)atof(pwm_arg);
        if (pwm < 0.0f) pwm = 0.0f;
        if (pwm > CAR_MAX_PWM) pwm = CAR_MAX_PWM;
        g_state.motor_test_mode = mode;
        g_state.speed_tune_mode = REMOTE_SPEED_TUNE_NONE;
        g_state.motor_test_pwm = pwm;
        g_state.motor_enable = true;
        g_state.force_stop = false;
        rc_reply("$OK,MOTORTEST\r\n");
    } else if (strcmp(cmd, "TUNE") == 0 || strcmp(cmd, "SPDTUNE") == 0) {
        char *mode_arg = next_token(&ctx);
        char *speed_arg = next_token(&ctx);
        remote_speed_tune_mode_t tune_mode;
        char buf[80];
        if (!mode_arg || !parse_speed_tune_mode(mode_arg, &tune_mode)) {
            rc_reply("$ERR,TUNE_ARG\r\n");
            return;
        }
        if (tune_mode == REMOTE_SPEED_TUNE_NONE) {
            g_state.speed_tune_mode = REMOTE_SPEED_TUNE_NONE;
            g_state.speed_override = false;
            rc_reply("$OK,TUNE,STOP\r\n");
            return;
        }
        if (!speed_arg) {
            rc_reply("$ERR,TUNE_SPEED\r\n");
            return;
        }
        set_target_speed((float)atof(speed_arg));
        g_state.speed_tune_mode = tune_mode;
        g_state.motor_test_mode = REMOTE_MOTOR_TEST_NONE;
        g_state.motor_enable = true;
        g_state.force_stop = false;
        snprintf(buf, sizeof(buf), "$OK,TUNE,%s,SPEED,%.3f\r\n", speed_tune_mode_name(g_state.speed_tune_mode), (double)g_state.target_speed_mps);
        rc_reply(buf);
    } else if (strcmp(cmd, "ENC?") == 0 || strcmp(cmd, "ENC") == 0) {
        g_state.encoder_query_pending = true;
        rc_reply("$OK,ENC,QUERY\r\n");
    } else if (strcmp(cmd, "SCOPE") == 0 || strcmp(cmd, "PSCOPE") == 0) {
        char *dev_arg = next_token(&ctx);
        char *ch_arg = next_token(&ctx);
        int dev;
        int ch;
        char buf[80];
#if CAR_ENABLE_PID_SCOPE
        if (dev_arg && strcmp(dev_arg, "PERIOD") == 0) {
            uint32_t period_ms;
            if (!ch_arg) {
                rc_reply("$ERR,SCOPE_PERIOD_ARG\r\n");
                return;
            }
            period_ms = (uint32_t)atoi(ch_arg);
            pid_scope_set_send_period_ms(period_ms);
            snprintf(buf, sizeof(buf), "$OK,SCOPE,PERIOD,%lu\r\n", (unsigned long)pid_scope_get_send_period_ms());
            rc_reply(buf);
            return;
        }
        if (dev_arg && strcmp(dev_arg, "RATE") == 0) {
            uint32_t hz;
            if (!ch_arg) {
                rc_reply("$ERR,SCOPE_RATE_ARG\r\n");
                return;
            }
            hz = (uint32_t)atoi(ch_arg);
            if (hz == 0U) {
                rc_reply("$ERR,SCOPE_RATE_RANGE\r\n");
                return;
            }
            pid_scope_set_send_period_ms(1000U / hz);
            snprintf(buf, sizeof(buf), "$OK,SCOPE,RATE,%lu,PERIOD,%lu\r\n", (unsigned long)hz, (unsigned long)pid_scope_get_send_period_ms());
            rc_reply(buf);
            return;
        }
        if (dev_arg && (strcmp(dev_arg, "SPEED") == 0 || strcmp(dev_arg, "BOTH") == 0 || strcmp(dev_arg, "ALL") == 0)) {
            g_state.scope_device_id = PID_SCOPE_DEVICE_ID;
            g_state.scope_channel_id = PID_SCOPE_CH_SPEED_BOTH;
            rc_reply("$OK,SCOPE,SPEED,DEV,1,CH,254\r\n");
            return;
        }
#endif
        if (!dev_arg || !ch_arg) {
            rc_reply("$ERR,SCOPE_ARG\r\n");
            return;
        }
        dev = atoi(dev_arg);
        ch = atoi(ch_arg);
        if (dev < 0 || dev > 255 || ch < 0 || ch > 255) {
            rc_reply("$ERR,SCOPE_RANGE\r\n");
            return;
        }
        g_state.scope_device_id = (uint8_t)dev;
        g_state.scope_channel_id = (uint8_t)ch;
        rc_reply("$OK,SCOPE\r\n");
    } else if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "STATUS?") == 0) {
        reply_status();
    } else if (strcmp(cmd, "CFG") == 0 || strcmp(cmd, "CONFIG") == 0) {
        parse_config(&ctx);
    } else {
        rc_reply("$ERR,UNKNOWN\r\n");
    }
}

void remote_control_init(const remote_control_port_t *port) {
    memset(&g_port, 0, sizeof(g_port));
    if (port) g_port = *port;
    memset(&g_state, 0, sizeof(g_state));
    g_state.last_cmd_ms = rc_time_ms();
    g_state.speed_limit_mps = SPEED_HIGH_MPS;
    g_state.mode = CAR_MODE_TRACKING;
    g_state.timeout_ms = REMOTE_CONTROL_TIMEOUT_MS;
    g_state.scope_device_id = PID_SCOPE_DEVICE_ID;
    g_state.scope_channel_id = PID_SCOPE_CH_SPEED_LEFT;
    g_len = 0U;
    g_collecting = false;
}

void remote_control_rx_byte(uint8_t byte) {
    if (byte == '$') {
        g_collecting = true;
        g_len = 0U;
        return;
    }
    if (!g_collecting) {
        if (!is_cmd_start(byte)) return;
        g_collecting = true;
        g_len = 0U;
    }
    if (byte == '\r' || byte == '\n') {
        if (g_len == 0U) {
            g_collecting = false;
            return;
        }
        g_line[g_len] = '\0';
        g_collecting = false;
        parse_line(g_line);
        return;
    }
    if (!is_printable_ascii(byte)) {
        g_collecting = false;
        g_len = 0U;
        return;
    }
    if (g_len < (RC_LINE_MAX - 1U)) {
        g_line[g_len++] = (char)byte;
    } else {
        g_collecting = false;
        g_len = 0U;
    }
}

void remote_control_poll(void) {
    if ((rc_time_ms() - g_state.last_cmd_ms) > g_state.timeout_ms) {
        g_state.motor_enable = false;
        g_state.force_stop = true;
    }
}

remote_control_state_t remote_control_get_state(void) { return g_state; }

void remote_control_clear_pid_update(void) { g_state.pid_update.pending = false; }

void remote_control_reply_encoder(float left_speed_mps, float right_speed_mps, float distance_m) {
    char buf[96];
    g_state.encoder_query_pending = false;
    snprintf(buf, sizeof(buf), "$ENC,L,%.4f,R,%.4f,D,%.4f\r\n", (double)left_speed_mps, (double)right_speed_mps, (double)distance_m);
    rc_reply(buf);
}
