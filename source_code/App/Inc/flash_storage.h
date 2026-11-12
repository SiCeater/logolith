/* flash_storage.h
 * Stockage persistant d'un bloc de config en flash interne STM32F405.
 *
 * Zone réservée : SECTEUR 11 (0x080E0000 – 0x080FFFFF, 128 Ko), et RIEN d'autre.
 *
 * Garanties :
 *   - Le firmware ne peut jamais déborder dans le secteur 11 : calib_guard.ld
 *     fait échouer le link si le code + données atteignent 0x080E0000.
 *   - Comme l'ELF ne contient rien dans le secteur 11, les outils de flash
 *     (CubeProgrammer en SWD ou DFU, ST-Link GDB server) n'effacent que les
 *     secteurs qu'ils écrivent : la config survit à un reflash.
 *     SEULE exception : un "full chip erase" / mass erase volontaire.
 *   - L'inverse : ce module n'efface que le secteur 11 et refuse toute écriture
 *     hors de [FLASH_STORE_BASE, FLASH_STORE_END[ (le code n'est jamais touché).
 *   - Chaque enregistrement porte un magic + une longueur + un CRC32. Une coupure
 *     d'alimentation pendant une écriture donne un enregistrement invalide qui
 *     est ignoré : le précédent reste utilisé.
 *
 * Format : journal (log) de slots de taille fixe, écrits les uns après les autres.
 * Le dernier slot valide fait foi. Le secteur n'est effacé que lorsqu'il est plein
 * (512 sauvegardes), ce qui limite l'usure (~10 000 cycles d'effacement garantis).
 */
#ifndef __FLASH_STORAGE_H__
#define __FLASH_STORAGE_H__

#include <stdint.h>
#include <stdbool.h>

#define FLASH_STORE_SECTOR      11U
#define FLASH_STORE_BASE        0x080E0000UL
#define FLASH_STORE_SIZE        (128UL * 1024UL)
#define FLASH_STORE_END         (FLASH_STORE_BASE + FLASH_STORE_SIZE)

#define FLASH_STORE_SLOT_SIZE   256U                                   /* octets par enregistrement */
#define FLASH_STORE_HDR_SIZE    8U                                     /* magic + len + seq */
#define FLASH_STORE_MAX_PAYLOAD (FLASH_STORE_SLOT_SIZE - FLASH_STORE_HDR_SIZE - 4U)  /* - CRC32 */

/* Charge le dernier enregistrement valide. Retourne false si aucun (secteur vierge,
 * effacé par un mass erase, ou taille différente). `data` n'est pas modifié dans ce cas. */
bool FlashStore_Load(void *data, uint16_t len);

/* Ajoute un enregistrement (efface le secteur 11 d'abord s'il est plein : ~1-2 s,
 * CPU et IRQ bloqués pendant l'effacement → à n'appeler que drone désarmé au sol).
 * Relit et vérifie l'enregistrement après écriture. */
bool FlashStore_Save(const void *data, uint16_t len);

#endif
