/**
 * global.c — définitions des variables globales (voir global.h pour leur rôle).
 */
#include "global.h"

/* Flags de debug : passer à true pour activer l'affichage correspondant sur UART5.
 * Les prints sont BLOQUANTS (attente UART). debug_imu imprime depuis l'ISR DMA de
 * l'ICM (décimé ~10 Hz) ; debug_rc depuis la boucle principale (5 Hz). À n'activer
 * que sur le banc. */
bool debug_init = true;
bool debug_rc   = true;
bool debug_imu  = true;
bool debug_warn = true;

/* Non utilisés pour l'instant (cf. global.h) */
int16_t AccData[3]  = {0};
int16_t MagData[3]  = {0};
int16_t GyroData[3] = {0};

/* Consignes moteurs, échelle 0 (arrêt) … 3000 (plein gaz).
 * DOIVENT démarrer à 0. Elles valaient 3000 auparavant (reste de l'époque où
 * l'unité était directement le compare du timer, 3000 = 125 µs = arrêt). Avec
 * l'échelle actuelle d'ESC_Set_Values, 3000 = PLEIN GAZ au premier appel de
 * ESC_Set_Global_Values(). */
uint16_t m1 = 0;
uint16_t m2 = 0;
uint16_t m3 = 0;
uint16_t m4 = 0;
uint16_t m5 = 0;
uint16_t m6 = 0;
uint16_t m7 = 0;
uint16_t m8 = 0;

/* Compteur milliseconde, incrémenté par SysTick_Handler (voir global.h). */
volatile uint32_t ms_counter = 0;

/* Lecture 32 bits alignée = atomique sur Cortex-M4 : pas besoin de couper les IRQ. */
uint32_t get_ms(void)
{
    return ms_counter;
}
