// Réception d'une image par DFU 1.1 et écriture dans la zone application.
//
// Le bloc 0 (table des vecteurs) est gardé en RAM et écrit en dernier, au manifeste : si le transfert
// est interrompu, la zone application reste invalide et le bootloader reste en DFU au lieu de lancer
// une image tronquée.

#include <string.h>

#include "boot.h"
#include "flash.h"
#include "tusb.h"

#define BLOCK_SIZE  CFG_TUD_DFU_XFER_BUFSIZE
#define WRITE_MS    10 // programmation d'un bloc de 1 Ko, large

static uint8_t first_block[BLOCK_SIZE];
static uint16_t first_block_len;
static uint32_t next_block;
static bool complete;

static uint32_t block_address(uint32_t block)
{
    return APP_START + block * BLOCK_SIZE;
}

bool dfu_done(void)
{
    return complete;
}

// Appelé juste avant tud_dfu_download_cb() : le PC attendra ce délai avant de redemander le statut.
uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state)
{
    (void)alt;
    if (state == DFU_DNBUSY) {
        uint32_t addr = block_address(next_block);
        return flash_is_sector_start(addr) ? flash_erase_time_ms(addr) + WRITE_MS : WRITE_MS;
    }
    return 0;
}

void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, const uint8_t *data, uint16_t length)
{
    (void)alt;
    if (block_num == 0) {
        next_block = 0; // nouveau transfert
        complete = false;
    }

    uint32_t addr = block_address(block_num);
    if (block_num != next_block || addr + length > APP_END) {
        tud_dfu_finish_flashing(DFU_STATUS_ERR_ADDRESS);
        return;
    }

    if (flash_is_sector_start(addr) && !flash_erase_sector(addr)) {
        tud_dfu_finish_flashing(DFU_STATUS_ERR_ERASE);
        return;
    }

    if (block_num == 0) {
        memcpy(first_block, data, length);
        first_block_len = length;
    } else if (!flash_program(addr, data, length)) {
        tud_dfu_finish_flashing(DFU_STATUS_ERR_VERIFY);
        return;
    }

    next_block++;
    tud_dfu_finish_flashing(DFU_STATUS_OK);
}

void tud_dfu_manifest_cb(uint8_t alt)
{
    (void)alt;
    if (next_block == 0) {
        tud_dfu_finish_flashing(DFU_STATUS_ERR_NOTDONE);
        return;
    }
    if (!flash_program(APP_START, first_block, first_block_len)) {
        tud_dfu_finish_flashing(DFU_STATUS_ERR_VERIFY);
        return;
    }
    if (!app_is_valid()) {
        tud_dfu_finish_flashing(DFU_STATUS_ERR_FIRMWARE);
        return;
    }

    complete = true;
    tud_dfu_finish_flashing(DFU_STATUS_OK);
}

void tud_dfu_abort_cb(uint8_t alt)
{
    (void)alt;
    next_block = 0;
}
