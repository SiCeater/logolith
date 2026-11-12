/**
 * esc_driver.h
 * Sortie moteurs OneShot125 sur TIM8 (4 moteurs du quad). Détails : esc_driver.c.
 *
 * Échelle commune à toutes les fonctions : 0 = arrêt (125 µs) … 3000 = plein gaz (250 µs).
 */
#ifndef __ESC_DRIVER_H__
#define __ESC_DRIVER_H__

#include "main.h"
#include "global.h"
#include "debug.h"
#include "led_driver.h"

#define ESC_CMD_MAX     3000U   /* plein gaz */
#define ESC_CCR_MIN     3000U   /* compare TIM8 pour 125 µs (24 MHz × 125 µs) */

/* Démarre TIM8 et sort l'impulsion d'arrêt (125 µs) sur les 4 moteurs. Sans risque. */
void ESC_Init(void);

/* /!\ DANGEREUX — HÉLICES DÉMONTÉES. Calibration de la plage des ESC (plein gaz
 * 2 s puis arrêt 2 s). Bloquant 4 s. À ne lancer que volontairement, jamais au boot. */
void ESC_Calibrate(void);

void ESC_Set_Values(uint16_t front_left_motor, uint16_t front_right_motor,
                    uint16_t rear_right_motor, uint16_t rear_left_motor);
void ESC_Set_Global_Values(void);   /* applique m1..m4 (global.h) */
void ESC_FL_MS(uint16_t x);
void ESC_FR_MS(uint16_t x);
void ESC_RR_MS(uint16_t x);
void ESC_RL_MS(uint16_t x);

/* /!\ HÉLICES DÉMONTÉES. Fait tourner chaque moteur l'un après l'autre (bloquant ~8 s). */
void ESC_Test(void);

#endif
