/**
 * esc_driver.h
 * Sortie moteurs OneShot125 sur TIM8 (4 moteurs du quad), one-pulse déclenché par
 * la fast loop. Les ESC_Set_* rangent la commande, ESC_Update l'émet. Détails : esc_driver.c.
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
#define ESC_CCR_MIN     3000U   /* 125 µs en ticks TIM8 (24 MHz × 125 µs) */
#define ESC_OPM_ARR     6000U   /* cycle one-pulse : 6001 ticks = 250 µs (voir esc_driver.c) */

/* Passe TIM8 en one-pulse, sorties armées à l'arrêt. N'émet rien. Sans risque. */
void ESC_Init(void);

/* Émet UNE impulsion avec les dernières commandes. Appelée par la fast loop à
 * chaque itération (seul point d'émission). */
void ESC_Update(void);

/* Calibration de plage des ESC : mode radio dans esc_calib.c. */

void ESC_Set_Values(uint16_t front_left_motor, uint16_t front_right_motor,
                    uint16_t rear_right_motor, uint16_t rear_left_motor);
void ESC_Set_Global_Values(void);   /* applique m1..m4 (global.h) */
void ESC_FL_MS(uint16_t x);
void ESC_FR_MS(uint16_t x);
void ESC_RR_MS(uint16_t x);
void ESC_RL_MS(uint16_t x);

/* Position des pads (vu de dessus, avant = caméra) : M1 = RL, M2 = FL, M3 = RR, M4 = FR. */
void ESC_Set_Pad(uint8_t pad, uint16_t cmd);   /* pad 1..4 = M1..M4 (sérigraphie) */
void ESC_Stop_All(void);                        /* commande d'arrêt sur les 4 sorties */

#endif
