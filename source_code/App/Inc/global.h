/**
 * global.h
 * Variables partagées par tout le firmware : flags de debug, consignes moteurs,
 * base de temps milliseconde.
 * Les variables propres à un module (RC, IMU, calibration…) sont dans leur
 * propre module : ne pas en rajouter ici sans raison.
 */
#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* ── Flags de debug (console UART5) ── voir global.c */
extern bool debug_init;   /* messages d'init (ICM, ESC, self-test, calibration gyro) */
extern bool debug_rc;     /* LQ, throttle et 12 canaux bruts, 5 Hz (depuis Remote_Process) */
extern bool debug_imu;    /* valeurs IMU ~10 Hz (dans l'ISR DMA de l'ICM) */
extern bool debug_warn;   /* avertissements (perte de lien RC…) */

/* ── Non utilisés pour l'instant (réservés : fusion IMU, magnéto BNO085) ── */
extern int16_t AccData[3];
extern int16_t MagData[3];
extern int16_t GyroData[3];

/* ── Consignes moteurs pour ESC_Set_Global_Values() ──
 * Unité : 0 = moteur arrêté (impulsion 125 µs) … 3000 = plein gaz (250 µs).
 * m1-m4 : les 4 moteurs du quad (ordre FL, FR, RR, RL, voir esc_driver.c).
 * m5-m8 : sorties ESC_M5..M8 de la LithCore, pas encore pilotées (timers initialisés
 *         par CubeMX mais sorties jamais activées) :
 *           ESC_M5/M6/M7 = TIM1 CH1/CH2/CH3 (PA8/PA9/PA10), même échelle que TIM8
 *             (APB2, 168 MHz / 7 = 24 MHz → 125 µs = 3000 ticks).
 *           ESC_M8 = TIM3 CH4 (PB1). /!\ TIM3 est sur APB1 : 84 MHz / 7 = 12 MHz,
 *             donc ÉCHELLE DIVISÉE PAR 2 : 125 µs = 1500 ticks, période 11999.
 *             (24 MHz est impossible depuis 84 MHz avec un prescaler entier.) */
extern uint16_t m1;
extern uint16_t m2;
extern uint16_t m3;
extern uint16_t m4;
extern uint16_t m5;
extern uint16_t m6;
extern uint16_t m7;
extern uint16_t m8;

/* ── Base de temps ──
 * ms_counter est incrémenté toutes les 1 ms par SysTick_Handler (stm32f4xx_it.c).
 * Le SysTick est configuré à 1 kHz par LL_Init1msTick() dans main.c.
 * On n'utilise pas HAL_GetTick() : le projet est en LL pur, il n'y a pas de tick HAL.
 * Déborde après 49,7 jours : toujours comparer des durées par soustraction
 * (now - t0 >= d), qui reste juste même au débordement, jamais par (now >= t0 + d). */
extern volatile uint32_t ms_counter;
uint32_t get_ms(void);

#endif
