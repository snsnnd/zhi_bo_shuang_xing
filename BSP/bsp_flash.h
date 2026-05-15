#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Flash persistence abstraction used by the learned track map.
 * The C file currently provides a RAM-backed mock; on hardware this should be
 * replaced with STM32 page erase/program/read operations at the same API level.
 */
bool bsp_flash_read(uint32_t addr, void *buf, uint32_t len);
bool bsp_flash_write(uint32_t addr, const void *buf, uint32_t len);
bool bsp_flash_erase_page(uint32_t page_addr);

#endif
