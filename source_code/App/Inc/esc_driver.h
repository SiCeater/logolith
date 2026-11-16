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
#define ESC_CMD_START   450U    /* 15 % : seuil de démarrage moteur mesuré au banc */
#define ESC_CMD_SAT     2500U   /* ~83 % : au-delà, l'ESC (non calibré) ne va pas plus vite.
                                 * Ces deux valeurs sont à remesurer après la calibration ESC (esc_calib.c). */

/* Démarre TIM8 et sort l'impulsion d'arrêt (125 µs) sur les 4 moteurs. Sans risque. */
void ESC_Init(void);

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
void ESC_Stop_All(void);                        /* impulsion d'arrêt sur les 4 sorties */

#endif
