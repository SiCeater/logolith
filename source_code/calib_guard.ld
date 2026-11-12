/* calib_guard.ld — script de link complémentaire (passé en entrée au linker par CMakeLists.txt).
 *
 * Le secteur 11 de la flash (0x080E0000 – 0x080FFFFF) est réservé au stockage persistant
 * (calibrations, voir App/Inc/flash_storage.h). Ce fichier fait ÉCHOUER le link si le
 * firmware (code + rodata + valeurs initiales de .data/.tdata) déborde dans cette zone.
 *
 * Il est séparé de STM32F405XX_FLASH.ld pour survivre aux régénérations CubeMX.
 */
ASSERT(LOADADDR(.tdata) + SIZEOF(.tdata) <= 0x080E0000,
       "Firmware trop gros : il deborde dans le secteur 11 reserve a la calibration (flash_storage.h)")
