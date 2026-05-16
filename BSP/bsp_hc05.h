#ifndef BSP_HC05_H
#define BSP_HC05_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*hc05_write_fn)(const uint8_t *data, uint16_t len);
typedef uint32_t (*hc05_time_fn)(void);
typedef void (*hc05_rx_cb)(uint8_t byte);

typedef struct {
    hc05_write_fn write;
    hc05_time_fn get_time_ms;
    hc05_rx_cb on_rx_byte;
} hc05_port_t;

void bsp_hc05_init(const hc05_port_t *port);
bool bsp_hc05_is_ready(void);
int bsp_hc05_send(const uint8_t *data, uint16_t len);
int bsp_hc05_send_text(const char *text);
void bsp_hc05_poll(void);
void bsp_hc05_tx_done(void);
void bsp_hc05_rx_byte(uint8_t byte);
uint16_t bsp_hc05_available(void);
bool bsp_hc05_read_byte(uint8_t *byte);
uint32_t bsp_hc05_rx_count(void);
uint32_t bsp_hc05_tx_count(void);

/* AT helpers: only call while HC-05 KEY pin is in AT mode. */
int bsp_hc05_at_command(const char *cmd);
int bsp_hc05_at_set_name(const char *name);
int bsp_hc05_at_set_uart(uint32_t baud, uint8_t stop_bits, uint8_t parity);

/* Weak USART hooks. Override these in a board-specific UART file. */
int bsp_hc05_port_write(const uint8_t *data, uint16_t len);
uint32_t bsp_hc05_port_time_ms(void);

#ifdef __cplusplus
}
#endif

#endif
