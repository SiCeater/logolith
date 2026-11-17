#pragma once

#include <stdbool.h>

// Passe le cœur de l'oscillateur interne (16 MHz) au quartz 8 MHz multiplié par la PLL :
// SYSCLK 168 MHz, AHB 168 MHz, APB1 42 MHz, APB2 84 MHz, USB 48 MHz.
// Renvoie false si le quartz ne démarre pas : la carte reste alors sur l'oscillateur interne.
bool clock_init(void);
