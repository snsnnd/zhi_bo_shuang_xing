#include "bsp_flash.h"
#include "../Common/car_config.h"
#include "main.h"
#include <string.h>

static bool in_map_area(uint32_t addr, uint32_t len) {
    return addr >= FLASH_MAP_ADDR && (addr + len) <= (FLASH_MAP_ADDR + FLASH_MAP_SIZE_BYTES);
}

// 从 MCU 内部 Flash 读取地图数据。
bool bsp_flash_read(uint32_t addr, void *buf, uint32_t len) {
    if (!buf || !in_map_area(addr, len)) return false;
    memcpy(buf, (const void *)addr, len);
    return true;
}

// STM32F1 Flash 按半字编程，调用方必须先擦除目标页。
bool bsp_flash_write(uint32_t addr, const void *buf, uint32_t len) {
    if (!buf || !in_map_area(addr, len)) return false;
    const uint8_t *p = (const uint8_t *)buf;
    bool ok = true;

    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < len; i += 2U) {
        uint16_t half = p[i];
        if ((i + 1U) < len) half |= (uint16_t)p[i + 1U] << 8;
        else half |= 0xFF00U;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, half) != HAL_OK) {
            ok = false;
            break;
        }
    }
    HAL_FLASH_Lock();
    return ok;
}

// 擦除地图存储区域；STM32F103C8 每页 1KB，这里擦除连续 2 页。
bool bsp_flash_erase_page(uint32_t page_addr) {
    if (page_addr != FLASH_MAP_ADDR) return false;

    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0U;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = FLASH_MAP_ADDR;
    erase.NbPages = FLASH_MAP_SIZE_BYTES / 1024U;

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();
    return st == HAL_OK;
}
