#include "flash.h"

#include <string.h>

#include "stm32f4xx.h"

#define FLASH_KEY1 0x45670123U
#define FLASH_KEY2 0xCDEF89ABU

#define FLASH_ERRORS (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_SOP)

// Secteur contenant addr, ou -1 hors de la flash.
static int sector_of(uint32_t addr, uint32_t *start, uint32_t *size)
{
    uint32_t base = FLASH_BASE;
    for (int n = 0; n < 12; n++) {
        uint32_t len = (n < 4) ? 16U * 1024 : (n == 4) ? 64U * 1024 : 128U * 1024;
        if (addr >= base && addr < base + len) {
            *start = base;
            *size = len;
            return n;
        }
        base += len;
    }
    return -1;
}

static void wait_ready(void)
{
    while (FLASH->SR & FLASH_SR_BSY) {
    }
}

static void unlock(void)
{
    wait_ready();
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
    FLASH->SR = FLASH_ERRORS | FLASH_SR_EOP;
}

static void lock(void)
{
    FLASH->CR = FLASH_CR_LOCK;
}

// Le cache de données peut garder l'ancien contenu d'une zone qu'on vient d'effacer ou d'écrire.
static void flush_data_cache(void)
{
    FLASH->ACR &= ~FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;
}

bool flash_is_sector_start(uint32_t addr)
{
    uint32_t start, size;
    return sector_of(addr, &start, &size) >= 0 && start == addr;
}

uint32_t flash_erase_time_ms(uint32_t addr)
{
    uint32_t start, size;
    if (sector_of(addr, &start, &size) < 0) {
        return 0;
    }
    return (size <= 16U * 1024) ? 500 : (size <= 64U * 1024) ? 1100 : 2000;
}

bool flash_erase_sector(uint32_t addr)
{
    uint32_t start, size;
    int n = sector_of(addr, &start, &size);
    if (n < 0) {
        return false;
    }

    unlock();
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER | ((uint32_t)n << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    wait_ready();
    bool ok = !(FLASH->SR & FLASH_ERRORS);
    lock();
    flush_data_cache();
    return ok;
}

bool flash_program(uint32_t addr, const uint8_t *data, size_t len)
{
    if (addr & 3U) {
        return false;
    }

    unlock();
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
    bool ok = true;
    for (size_t i = 0; i < len && ok; i += 4) {
        uint32_t word = 0xFFFFFFFFU;
        memcpy(&word, data + i, (len - i < 4) ? len - i : 4);
        *(volatile uint32_t *)(addr + i) = word;
        __DSB();
        wait_ready();
        ok = !(FLASH->SR & FLASH_ERRORS);
    }
    lock();
    flush_data_cache();

    return ok && memcmp((const void *)addr, data, len) == 0;
}
