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