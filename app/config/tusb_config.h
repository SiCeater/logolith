#pragma once

// Configuration TinyUSB : périphérique USB full speed sur l'OTG_FS du STM32F405 (PA11/PA12).

#define CFG_TUSB_MCU OPT_MCU_STM32F4
#define CFG_TUSB_OS  OPT_OS_FREERTOS

#define BOARD_TUD_RHPORT  0 // OTG_FS
#define CFG_TUD_ENABLED   1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))

#define CFG_TUD_ENDPOINT0_SIZE 64

// Un port série virtuel : console de debug, et déclencheur du passage en DFU (voir app/src/usb.c).
#define CFG_TUD_CDC            1
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256
#define CFG_TUD_CDC_EP_BUFSIZE 64
