#pragma once

// Interface de la carte : tout ce que l'application sait du matériel passe par ici.

#include <stdbool.h>

typedef enum {
    BOARD_LED_RED,  // PB9, N_R_LED
    BOARD_LED_BLUE, // PC0, N_B_LED
} board_led_t;

// Horloges (168 MHz sur quartz, repli sur HSI si le quartz ne démarre pas) et GPIO.
void board_init(void);

void board_led_set(board_led_t led, bool on);
void board_led_toggle(board_led_t led);

// Vrai si le quartz n'a pas démarré et que la carte tourne sur l'oscillateur interne.
bool board_hse_failed(void);
