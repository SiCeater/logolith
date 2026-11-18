#ifndef __FAST_LOOP_H__
#define __FAST_LOOP_H__

#include <stdint.h>

/* Boucle rapide à ICM42688_ODR_HZ (1 kHz, squelette), voir fast_loop.c. */

/* dt réel de la dernière itération en secondes (~1,004 ms), mesuré au DWT. */
extern volatile float fast_loop_dt;

/* Active le compteur de cycles DWT. À appeler AVANT ICM42688_Init (qui lance
 * les lectures DMA, donc les appels à FastLoop_Run). */
void FastLoop_Init(void);

/* Valeur courante du compteur de cycles (168 MHz). */
uint32_t FastLoop_Cycles(void);

/* Appelée à la fin de l'IRQ DMA de l'ICM. t_entry = FastLoop_Cycles() lu à
 * l'entrée de cette IRQ (sert au dt et au temps d'exécution). */
void FastLoop_Run(uint32_t t_entry);

/* Affiche les stats (~1 Hz) : fréquence, période min/max, temps d'exécution,
 * charge CPU de la chaîne IMU + boucle. Hors ISR, si debug_loop. */
void FastLoop_Debug_Process(void);

#endif
