/**
 * led_driver.c
 * LEDs de la LithCore : rouge (N_R_LED, PB9) et bleue (N_B_LED, PC0).
 * Le préfixe "N_" des labels CubeMX signifie "actif à l'état bas" : la LED est
 * câblée entre le 3,3 V et la broche, donc broche à 0 = LED allumée.
 * D'où l'inversion On → Reset, Off → Set.
 *
 * Utilisation actuelle : accel_calib (pendant une calibration) et security (squelette).
 */
#include "led_driver.h"

void R_LED_On()
{
    // Écrire un RESET (0) pour allumer la LED
    LL_GPIO_ResetOutputPin(N_R_LED_GPIO_Port, N_R_LED_Pin);
}

void R_LED_Off()
{
    // Écrire un SET (1) pour éteindre la LED
    LL_GPIO_SetOutputPin(N_R_LED_GPIO_Port, N_R_LED_Pin);
}

void R_LED_Toggle()
{
    // Basculer l'état actuel de la broche (SET <-> RESET)
    LL_GPIO_TogglePin(N_R_LED_GPIO_Port, N_R_LED_Pin);
}


void B_LED_On()
{
    // Écrire un RESET (0) pour allumer la LED
    LL_GPIO_ResetOutputPin(N_B_LED_GPIO_Port, N_B_LED_Pin);
}

void B_LED_Off()
{
    // Écrire un SET (1) pour éteindre la LED
    LL_GPIO_SetOutputPin(N_B_LED_GPIO_Port, N_B_LED_Pin);
}

void B_LED_Toggle()
{
    // Basculer l'état actuel de la broche (SET <-> RESET)
    LL_GPIO_TogglePin(N_B_LED_GPIO_Port, N_B_LED_Pin);
}