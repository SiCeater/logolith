#pragma once

#include <stdbool.h>
#include <stdint.h>

// Zone de l'application, définie par boards/<carte>/memory.ld.
extern uint32_t __app_start[];
extern uint32_t __app_end[];
#define APP_START ((uint32_t)__app_start)
#define APP_END   ((uint32_t)__app_end)

// Vrai si la zone application commence par une table de vecteurs plausible
// (pile en RAM, point d'entrée dans la zone application).
bool app_is_valid(void);

// DFU : vrai quand une image complète vient d'être écrite et que la carte doit redémarrer.
bool dfu_done(void);
