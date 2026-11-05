/* Plan mémoire de la LithCore (STM32F405RG).
   Flash : secteurs 0-3 de 16 Ko, secteur 4 de 64 Ko, secteurs 5-11 de 128 Ko. */

MEMORY
{
    BOOTLOADER (rx)  : ORIGIN = 0x08000000, LENGTH = 32K   /* secteurs 0-1 */
    SETTINGS   (r)   : ORIGIN = 0x08008000, LENGTH = 32K   /* secteurs 2-3, réservés aux réglages */
    APP        (rx)  : ORIGIN = 0x08010000, LENGTH = 960K  /* secteurs 4-11 */

    RAM        (xrw) : ORIGIN = 0x20000000, LENGTH = 128K - 16
    BOOT_SHARED (rw) : ORIGIN = 0x2001FFF0, LENGTH = 16    /* jamais initialisé : survit au reset */
    CCMRAM     (rw)  : ORIGIN = 0x10000000, LENGTH = 64K   /* pas accessible par le DMA */
}

/* Utilisés par le code C (boot.h, board.c). */
__app_start = ORIGIN(APP);
__app_end = ORIGIN(APP) + LENGTH(APP);
__boot_shared = ORIGIN(BOOT_SHARED);
