#include "bsp_flash.h"
#include <string.h>

#define MOCK_FLASH_BASE 0x0801F800u
#define MOCK_FLASH_SIZE 2048u

static uint8_t g_mock_flash[MOCK_FLASH_SIZE];
static bool g_init;

// 初始化 mock Flash：默认擦除态 0xFF
static void ensure_init(void) {
    if (!g_init) {
        memset(g_mock_flash, 0xFF, sizeof(g_mock_flash));
        g_init = true;
    }
}

// 模拟 Flash 读取
bool bsp_flash_read(uint32_t addr, void *buf, uint32_t len) {
    ensure_init();
    if (addr < MOCK_FLASH_BASE) return false;
    uint32_t off = addr - MOCK_FLASH_BASE;
    if ((off + len) > MOCK_FLASH_SIZE) return false;
    memcpy(buf, &g_mock_flash[off], len);
    return true;
}

// 模拟 Flash 写入（真实硬件需先擦后写）
bool bsp_flash_write(uint32_t addr, const void *buf, uint32_t len) {
    ensure_init();
    if (addr < MOCK_FLASH_BASE) return false;
    uint32_t off = addr - MOCK_FLASH_BASE;
    if ((off + len) > MOCK_FLASH_SIZE) return false;
    memcpy(&g_mock_flash[off], buf, len);
    return true;
}

// 模拟页擦除
bool bsp_flash_erase_page(uint32_t page_addr) {
    ensure_init();
    if (page_addr != MOCK_FLASH_BASE) return false;
    memset(g_mock_flash, 0xFF, sizeof(g_mock_flash));
    return true;
}
