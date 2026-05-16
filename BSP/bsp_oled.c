#include "bsp_oled.h"
#include "../Common/car_config.h"
#include "i2c.h"
#include <string.h>

// Active debug page: 0=line, 1=motion, 2=map/strategy.
static uint8_t g_page;
static bool g_ready;
static uint32_t g_last_refresh_ms;
static uint8_t g_buf[128U * 8U];
static uint32_t g_last_hc05_rx_count;
static uint32_t g_last_hc05_tx_count;
static uint32_t g_last_hc05_rx_ms;
static uint32_t g_last_hc05_tx_ms;

static bool oled_write(uint8_t ctrl, const uint8_t *data, uint16_t len) {
    uint8_t tmp[17];
    if (len > 16U) return false;
    tmp[0] = ctrl;
    memcpy(&tmp[1], data, len);
    return HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, tmp, (uint16_t)(len + 1U), 20U) == HAL_OK;
}

static bool oled_cmd1(uint8_t cmd) { return oled_write(0x00U, &cmd, 1U); }

static void oled_init_once(void) {
    if (g_ready) return;
    if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_I2C_ADDR, 2U, 20U) != HAL_OK) return;

    const uint8_t init[] = {
        0xAEU, 0x20U, 0x00U, 0xB0U, 0xC8U, 0x00U, 0x10U, 0x40U,
        0x81U, 0x7FU, 0xA1U, 0xA6U, 0xA8U, 0x3FU, 0xA4U, 0xD3U,
        0x00U, 0xD5U, 0x80U, 0xD9U, 0xF1U, 0xDAU, 0x12U, 0xDBU,
        0x40U, 0x8DU, 0x14U, 0xAFU
    };
    for (uint16_t i = 0U; i < sizeof(init); ++i) {
        if (!oled_cmd1(init[i])) return;
    }
    g_ready = true;
}

static float absf(float x) { return x < 0.0f ? -x : x; }
static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void draw_vbar(uint8_t x0, uint8_t width, float value, float max_value) {
    uint8_t h = (uint8_t)((clampf(value, 0.0f, max_value) / max_value) * 63.0f);
    for (uint8_t x = x0; x < (uint8_t)(x0 + width) && x < 128U; ++x) {
        for (uint8_t y = 0U; y < h; ++y) {
            uint8_t row = (uint8_t)(63U - y);
            g_buf[(row / 8U) * 128U + x] |= (uint8_t)(1U << (row % 8U));
        }
    }
}

static void draw_center_mark(float signed_value, float max_abs) {
    uint8_t center = 64U;
    uint8_t x = (uint8_t)(64.0f + (clampf(signed_value, -max_abs, max_abs) / max_abs) * 60.0f);
    for (uint8_t y = 0U; y < 64U; ++y) g_buf[(y / 8U) * 128U + center] |= (uint8_t)(1U << (y % 8U));
    for (uint8_t dx = 0U; dx < 3U; ++dx) {
        for (uint8_t y = 24U; y < 40U; ++y) g_buf[(y / 8U) * 128U + (uint8_t)(x + dx)] |= (uint8_t)(1U << (y % 8U));
    }
}

static void fill_rect(uint8_t x0, uint8_t y0, uint8_t width, uint8_t height) {
    for (uint8_t x = x0; x < (uint8_t)(x0 + width) && x < 128U; ++x) {
        for (uint8_t y = y0; y < (uint8_t)(y0 + height) && y < 64U; ++y) {
            g_buf[(y / 8U) * 128U + x] |= (uint8_t)(1U << (y % 8U));
        }
    }
}

static void oled_flush(void) {
    if (!g_ready) return;
    (void)oled_cmd1(0x21U);
    (void)oled_cmd1(0U);
    (void)oled_cmd1(127U);
    (void)oled_cmd1(0x22U);
    (void)oled_cmd1(0U);
    (void)oled_cmd1(7U);
    for (uint16_t i = 0U; i < sizeof(g_buf); i += 16U) {
        (void)oled_write(0x40U, &g_buf[i], 16U);
    }
}

// Select the runtime debug page, wrapping unsupported page numbers.
void bsp_oled_set_page(uint8_t page) { g_page = (uint8_t)(page % 3U); }

// Render one page of runtime data as simple SSD1306 bar/marker graphics.
void bsp_oled_show_runtime(car_mode_t mode, const oled_runtime_t *rt) {
    if (!rt) return;
    (void)mode;
    uint32_t now = HAL_GetTick();
    if ((now - g_last_refresh_ms) < 100U) return;
    g_last_refresh_ms = now;

    oled_init_once();
    if (!g_ready) return;

    memset(g_buf, 0, sizeof(g_buf));
    if (g_page == 0U) {
        // OLED 安装方向与车头视角相反，这里只镜像显示，不影响控制中的 L/R 数据。
        draw_vbar(4U, 16U, rt->line_raw.r2, 1.0f);
        draw_vbar(28U, 16U, rt->line_raw.r1, 1.0f);
        draw_vbar(84U, 16U, rt->line_raw.l1, 1.0f);
        draw_vbar(108U, 16U, rt->line_raw.l2, 1.0f);
        draw_center_mark(-rt->line_err, 3.0f);
    } else if (g_page == 1U) {
        draw_vbar(12U, 24U, rt->left_pwm, CAR_MAX_PWM);
        draw_vbar(92U, 24U, rt->right_pwm, CAR_MAX_PWM);
        draw_center_mark(rt->yaw_deg, 90.0f);
    } else {
        draw_vbar(12U, 24U, rt->current_limit_speed, SPEED_HIGH_MPS);
        draw_vbar(52U, 24U, rt->dist_to_next_event < 0.0f ? 0.0f : rt->dist_to_next_event, 1.0f);
        draw_vbar(92U, 24U, absf((float)rt->segment_type), (float)SEG_RISK);
    }
    oled_flush();
}

void bsp_oled_show_hc05_activity(uint32_t rx_count, uint32_t tx_count) {
    uint32_t now = HAL_GetTick();
    if ((now - g_last_refresh_ms) < 100U) return;
    g_last_refresh_ms = now;

    oled_init_once();
    if (!g_ready) return;

    if (rx_count != g_last_hc05_rx_count) {
        g_last_hc05_rx_count = rx_count;
        g_last_hc05_rx_ms = now;
    }
    if (tx_count != g_last_hc05_tx_count) {
        g_last_hc05_tx_count = tx_count;
        g_last_hc05_tx_ms = now;
    }

    bool rx_active = (rx_count > 0U) && ((now - g_last_hc05_rx_ms) < HC05_OLED_ACTIVITY_HOLD_MS);
    bool tx_active = (tx_count > 0U) && ((now - g_last_hc05_tx_ms) < HC05_OLED_ACTIVITY_HOLD_MS);

    memset(g_buf, 0, sizeof(g_buf));
    fill_rect(0U, 31U, 128U, 2U); // 中线：上半区 RX，下半区 TX
    fill_rect(0U, 0U, 2U, 64U);
    fill_rect(126U, 0U, 2U, 64U);

    if (rx_active) {
        fill_rect(8U, 6U, 112U, 18U); // 上半区亮起表示收到蓝牙数据
    }
    if (tx_active) {
        fill_rect(8U, 40U, 112U, 18U); // 下半区亮起表示正在/刚刚发送数据
    }
    oled_flush();
}
