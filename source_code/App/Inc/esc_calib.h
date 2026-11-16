#ifndef __ESC_CALIB_H__
#define __ESC_CALIB_H__

#include <stdbool.h>

/* Calibration de plage des ESC (OneShot125, 125–250 µs), pilotée à la radio,
 * HÉLICES DÉMONTÉES. Détails et procédure : esc_calib.c.
 *
 * Entrée  : drone désarmé, gaz EN HAUT, SA + SD tenus 2 s.
 * Sortie  : SA + SD de nouveau appuyés (à tout moment), ou perte du lien radio. */

#define ESCCAL_TRIG_HOLD_MS     2000U   /* SA+SD tenus pour entrer */
#define ESCCAL_EXIT_HOLD_MS     200U    /* SA+SD tenus pour sortir (anti-parasite) */
#define ESCCAL_THROTTLE_HIGH    0.95f   /* gaz "en haut" pour entrer */
#define ESCCAL_THROTTLE_LOW     0.05f   /* gaz "en bas" pour passer en manche direct */
#define ESCCAL_BLINK_HALF_MS    500U    /* clignotement moteurs 1 Hz : 500 ms on / 500 ms off */

/* À appeler dans la slow loop, après Security_Process(). Non bloquant. */
void EscCal_Process(void);

/* true tant que le mode calibration ESC est actif (les sorties moteurs lui
 * appartiennent) : l'armement et la calibration accel sont alors refusés. */
bool EscCal_InProgress(void);

#endif
