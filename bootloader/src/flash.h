#pragma once

// Écriture de la flash interne du STM32F4 (secteurs de 16, 64 et 128 Ko), par mots de 32 bits.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Vrai si addr est le premier octet d'un secteur.
bool flash_is_sector_start(uint32_t addr);

// Durée maximale d'effacement du secteur qui commence à addr (datasheet, 32 bits en parallèle).
uint32_t flash_erase_time_ms(uint32_t addr);

bool flash_erase_sector(uint32_t addr);

// Écrit len octets à partir de addr (aligné sur 4). La fin est complétée avec 0xFF. Relit pour vérifier.
bool flash_program(uint32_t addr, const uint8_t *data, size_t len);
