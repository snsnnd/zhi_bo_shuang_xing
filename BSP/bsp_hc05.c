#include "bsp_hc05.h"

#include "../Common/car_config.h"
#include "../Common/runtime_config.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    uint8_t data[HC05_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} hc05_ring_t;

typedef struct {
    uint8_t data[HC05_TX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} hc05_tx_ring_t;

static hc05_port_t g_port;
static hc05_ring_t g_rx;
static hc05_tx_ring_t g_tx;
static bool g_ready;
static volatile bool g_tx_busy;
static volatile uint32_t g_rx_count;
static volatile uint32_t g_tx_count;
static volatile bool g_rx_echo_pending;
static uint32_t g_tx_start_ms;
static uint8_t g_uart2_rx_byte;
static uint8_t g_tx_chunk[HC05_TX_CHUNK_SIZE];

static void hc05_uart2_start_rx(void) {
    (void)HAL_UART_Receive_DMA(&huart2, &g_uart2_rx_byte, 1U);
}

int bsp_hc05_port_write(const uint8_t *data, uint16_t len) {
    if (!data || len == 0U) return 0;
    return (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)data, len) == HAL_OK) ? (int)len : 0;
}

__weak uint32_t bsp_hc05_port_time_ms(void) { return HAL_GetTick(); }

static uint16_t rb_next(uint16_t v) { return (uint16_t)((v + 1U) % HC05_RX_BUFFER_SIZE); }
static uint16_t tx_next(uint16_t v) { return (uint16_t)((v + 1U) % HC05_TX_BUFFER_SIZE); }

static void rb_push(uint8_t byte) {
    uint16_t next = rb_next(g_rx.head);
    if (next == g_rx.tail) {
        g_rx.tail = rb_next(g_rx.tail);
    }
    g_rx.data[g_rx.head] = byte;
    g_rx.head = next;
}

static uint16_t tx_available(void) {
    return (uint16_t)((g_tx.head + HC05_TX_BUFFER_SIZE - g_tx.tail) % HC05_TX_BUFFER_SIZE);
}

static uint16_t tx_free(void) { return (uint16_t)(HC05_TX_BUFFER_SIZE - 1U - tx_available()); }

static void tx_drop(uint16_t len) {
    while (len-- > 0U && g_tx.tail != g_tx.head) g_tx.tail = tx_next(g_tx.tail);
}

static int tx_push(const uint8_t *data, uint16_t len) {
    uint16_t writable = tx_free();
    if (len > writable) len = writable;
    for (uint16_t i = 0U; i < len; ++i) {
        g_tx.data[g_tx.head] = data[i];
        g_tx.head = tx_next(g_tx.head);
    }
    return (int)len;
}

void bsp_hc05_init(const hc05_port_t *port) {
    memset(&g_rx, 0, sizeof(g_rx));
    memset(&g_tx, 0, sizeof(g_tx));
    memset(&g_port, 0, sizeof(g_port));
    if (port) {
        g_port = *port;
    } else {
        g_port.write = bsp_hc05_port_write;
        g_port.get_time_ms = bsp_hc05_port_time_ms;
    }
    g_ready = (g_port.write != 0);
    g_tx_busy = false;
    g_rx_count = 0U;
    g_tx_count = 0U;
    g_rx_echo_pending = false;
    g_tx_start_ms = 0U;
    hc05_uart2_start_rx();
}

bool bsp_hc05_is_ready(void) { return g_ready; }

int bsp_hc05_send(const uint8_t *data, uint16_t len) {
    if (!g_ready || !data || len == 0U) return 0;
    return tx_push(data, len);
}

int bsp_hc05_send_text(const char *text) {
    if (!text) return 0;
    return bsp_hc05_send((const uint8_t *)text, (uint16_t)strlen(text));
}

void bsp_hc05_poll(void) {
    if (g_tx_busy && (bsp_hc05_port_time_ms() - g_tx_start_ms) > HC05_TX_TIMEOUT_MS) {
        (void)HAL_UART_AbortTransmit(&huart2);
        g_tx_busy = false;
    }

    if (runtime_feature_enabled(RC_FEATURE_HC05_RX_ECHO_TEST) && g_ready && g_rx_echo_pending) {
        g_rx_echo_pending = false;
        (void)bsp_hc05_send_text("$RX\r\n");
    }

    if (!g_ready || g_tx_busy || g_tx.head == g_tx.tail) return;

    uint16_t n = 0U;
    uint16_t pos = g_tx.tail;
    while (pos != g_tx.head && n < HC05_TX_CHUNK_SIZE) {
        g_tx_chunk[n++] = g_tx.data[pos];
        pos = tx_next(pos);
    }

    g_tx_busy = true;
    g_tx_start_ms = bsp_hc05_port_time_ms();
    int accepted = g_port.write(g_tx_chunk, n);
    if (accepted > 0) {
        tx_drop((uint16_t)accepted);
        g_tx_count += (uint32_t)accepted;
    } else {
        g_tx_busy = false;
    }
}

void bsp_hc05_tx_done(void) { g_tx_busy = false; }

void bsp_hc05_rx_byte(uint8_t byte) {
    g_rx_count++;
    rb_push(byte);
    if (runtime_feature_enabled(RC_FEATURE_HC05_RX_ECHO_TEST)) g_rx_echo_pending = true;
    if (g_port.on_rx_byte) g_port.on_rx_byte(byte);
}

uint32_t bsp_hc05_rx_count(void) { return g_rx_count; }
uint32_t bsp_hc05_tx_count(void) { return g_tx_count; }

uint16_t bsp_hc05_available(void) {
    return (uint16_t)((g_rx.head + HC05_RX_BUFFER_SIZE - g_rx.tail) % HC05_RX_BUFFER_SIZE);
}

bool bsp_hc05_read_byte(uint8_t *byte) {
    if (!byte || g_rx.head == g_rx.tail) return false;
    *byte = g_rx.data[g_rx.tail];
    g_rx.tail = rb_next(g_rx.tail);
    return true;
}

int bsp_hc05_at_command(const char *cmd) {
    int sent = bsp_hc05_send_text(cmd);
    sent += bsp_hc05_send_text("\r\n");
    return sent;
}

int bsp_hc05_at_set_name(const char *name) {
    char buf[40];
    if (!name) return 0;
    int n = snprintf(buf, sizeof(buf), "AT+NAME=%s", name);
    if (n <= 0 || n >= (int)sizeof(buf)) return 0;
    return bsp_hc05_at_command(buf);
}

int bsp_hc05_at_set_uart(uint32_t baud, uint8_t stop_bits, uint8_t parity) {
    char buf[40];
    int n = snprintf(buf, sizeof(buf), "AT+UART=%lu,%u,%u", (unsigned long)baud, stop_bits, parity);
    if (n <= 0 || n >= (int)sizeof(buf)) return 0;
    return bsp_hc05_at_command(buf);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) bsp_hc05_tx_done();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        bsp_hc05_rx_byte(g_uart2_rx_byte);
        hc05_uart2_start_rx();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        bsp_hc05_tx_done();
        hc05_uart2_start_rx();
    }
}
