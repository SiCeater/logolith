#ifndef __LED_DRIVER_H__
#define __LED_DRIVER_H__

#include "main.h"   /* labels CubeMX N_R_LED / N_B_LED (actives à l'état bas, voir led_driver.c) */

void R_LED_On();
void R_LED_Off();
void R_LED_Toggle();
void B_LED_On();
void B_LED_Off();
void B_LED_Toggle();

#endif