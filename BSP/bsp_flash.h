#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Flash persistence abstraction used by the learned track map.
 * The implementation uses STM32F1 internal Flash half-word programming and a
 * reserved map area configured in Common/car_config.h.
 */
bool bsp_flash_read(uint32_t addr, void *buf, uint32_t len);
bool bsp_flash_write(uint32_t addr, const void *buf, uint32_t len);
bool bsp_flash_erase_page(uint32_t page_addr);

#endif
