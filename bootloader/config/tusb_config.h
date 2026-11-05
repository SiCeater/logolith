#pragma once

// Configuration TinyUSB du bootloader : sans RTOS, une seule interface DFU 1.1.

#define CFG_TUSB_MCU OPT_MCU_STM32F4
#define CFG_TUSB_OS  OPT_OS_NONE

#define BOARD_TUD_RHPORT  0 // OTG_FS
#define CFG_TUD_ENABLED   1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))

#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_DFU              1
#define CFG_TUD_DFU_XFER_BUFSIZE 1024 // divise toutes les tailles de secteur : un bloc ne chevauche jamais deux secteurs
