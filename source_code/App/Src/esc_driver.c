/**
 * esc_driver.c
 * Commande des 4 ESC en OneShot125 via TIM8 (PWM, 4 canaux).
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * PROTOCOLE ONESHOT125
 * ═══════════════════════════════════════════════════════════════════════════
 * Une impulsion par période, dont la LARGEUR code les gaz :
 *     125 µs = arrêt    …    250 µs = plein gaz
 * (= le PWM servo classique 1000-2000 µs divisé par 8, donc plus réactif.)
 *
 * Config TIM8 (CubeMX, tim.c) : horloge timer 168 MHz, Prescaler = 6 → 168/7 = 24 MHz,
 * soit 1 tick = 41,67 ns. Autoreload = 23999 → période 1 ms → 1 kHz.
 *     125 µs = 3000 ticks (ESC_CCR_MIN)      250 µs = 6000 ticks
 * → compare = 3000 + commande, avec commande 0 … 3000 (ESC_CMD_MAX).
 *
 * NB fréquence : à 1 kHz, les moteurs ne reçoivent une nouvelle consigne qu'une
 * fois par ms, même si le PID tourne à 8 kHz. OneShot125 accepte jusqu'à ~3,9 kHz
 * (période > 250 µs). À revoir quand le PID sera branché (ou DShot, cf. README).
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * CORRESPONDANCE MOTEURS ↔ CANAUX
 * ═══════════════════════════════════════════════════════════════════════════
 * Sorties de la LithCore (sérigraphie M1..M8 = labels CubeMX ESC_Mx, confirmés) :
 *   M1 = PC6 TIM8_CH1   M2 = PC7 TIM8_CH2   M3 = PC8 TIM8_CH3   M4 = PC9 TIM8_CH4
 *   M5 = PA8 TIM1_CH1   M6 = PA9 TIM1_CH2   M7 = PA10 TIM1_CH3  M8 = PB1 TIM3_CH4
 *
 * Position et sens des moteurs, relevés au banc sur Aerolith (motor_test.c).
 * NB : un premier relevé avait été fait drone face à l'opérateur (caméra vers
 * lui), donc tourné de 180° ; corrigé ci-dessous (avant = côté caméra).
 *   M1 (CH1) = arrière gauche (RL), horaire       M2 (CH2) = avant gauche (FL), anti-horaire
 *   M3 (CH3) = arrière droit  (RR), anti-horaire  M4 (CH4) = avant droit  (FR), horaire
 * Vu de dessus, avant (caméra) en haut :
 *          M2 (FL) ↺   M4 (FR) ↻
 *                \     /
 *                /     \
 *          M1 (RL) ↻   M3 (RR) ↺
 * Les diagonales tournent dans le même sens (FR/RL horaires, FL/RR anti-horaires) :
 * Quad-X cohérent, en configuration "props out" (vu de dessus, à l'avant du
 * drone les pales s'écartent de l'axe : FL vers la gauche, FR vers la droite),
 * c.-à-d. le "motor direction reversed" de Betaflight.
 * Pour le mixeur yaw : le couple de réaction sur le châssis est OPPOSÉ au sens
 * de l'hélice. Accélérer les horaires (M1, M4) fait tourner le drone en
 * anti-horaire vu de dessus (nez à gauche) ; accélérer les anti-horaires (M2, M3)
 * le fait tourner en horaire (nez à droite).
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * PLAGE UTILE DES ESC : 15 % … 83 % (ESC NON CALIBRÉS)
 * ═══════════════════════════════════════════════════════════════════════════
 * Mesuré au banc : le moteur démarre vers 15 % et n'accélère plus au-delà de
 * ~83 %. Ces deux bornes tombent pile sur les réglages PAR DÉFAUT des ESC BLHeli
 * en entrée PWM : min throttle 1148 µs, max throttle 1832 µs. En OneShot125 la
 * durée est divisée par 8 : 143,5 µs et 229 µs, soit
 *     (143,5 − 125) / 125 = 14,8 %      (229 − 125) / 125 = 83,2 %
 * Ce n'est donc pas l'absence d'hélice (sans charge le moteur accélérerait
 * jusqu'à son max, sans plafond net), mais l'ESC qui sature : tout ce qui
 * dépasse 229 µs est lu comme plein gaz, tout ce qui est sous 143,5 µs comme arrêt.
 * Conséquence : on perd ~32 % de la résolution de commande. Remède : calibrer la
 * plage des ESC (mode radio esc_calib.c, ou min/max throttle 1000/2000 dans BLHeliSuite),
 * puis remesurer ESC_CMD_START. Le passage au DShot (numérique, sans calibration)
 * règlera le problème pour de bon.
 */
#include "esc_driver.h"
#include "global.h"
#include <stdint.h>

/* Sature la commande à [0, ESC_CMD_MAX]. Le type est non signé : pas de cas < 0. */
static inline uint16_t esc_clamp(uint16_t x)
{
    return (x > ESC_CMD_MAX) ? (uint16_t)ESC_CMD_MAX : x;
}

/**
 * @brief Démarre TIM8 et envoie l'impulsion d'ARRÊT (125 µs) aux 4 ESC.
 * @note  Les ESC s'arment ("bips") en recevant un signal d'arrêt valide pendant
 *        quelques secondes. Cette fonction ne fait jamais tourner un moteur.
 */
void ESC_Init(void)
{
    /* Compare initial = arrêt, AVANT d'activer les sorties : la toute première
     * impulsion émise est déjà une impulsion d'arrêt. */
    LL_TIM_OC_SetCompareCH1(TIM8, ESC_CCR_MIN);
    LL_TIM_OC_SetCompareCH2(TIM8, ESC_CCR_MIN);
    LL_TIM_OC_SetCompareCH3(TIM8, ESC_CCR_MIN);
    LL_TIM_OC_SetCompareCH4(TIM8, ESC_CCR_MIN);

    /* La preload des compares est activée (CubeMX, LL_TIM_OC_EnablePreload) : une
     * écriture de compare ne devient active qu'au prochain événement "update"
     * (fin de période). On force cet update maintenant pour charger 125 µs tout
     * de suite. MX_TIM8_Init met déjà 3000 (= 125 µs, arrêt, cf. Pulse dans le .ioc),
     * donc c'est une double sécurité : si quelqu'un change ces valeurs dans CubeMX,
     * la 1re impulsion reste quand même un arrêt. */
    LL_TIM_GenerateEvent_UPDATE(TIM8);
    LL_TIM_SetCounter(TIM8, 0);

    /* TIM8 est un timer "advanced" : en plus de l'activation de chaque canal, ses
     * sorties ne sortent sur les broches que si MOE (Main Output Enable) est à 1,
     * d'où LL_TIM_EnableAllOutputs. */
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableAllOutputs(TIM8);

    LL_TIM_EnableCounter(TIM8);

    if (debug_init)
        print_to_console("\n\rESCs : initialized (stop pulse 125us)", 38);
}

/**
 * @brief Consigne des 4 moteurs, chacun de 0 (arrêt) à 3000 (plein gaz). Saturé.
 * @note  Ordre des paramètres : FL, FR, RR, RL (sens horaire depuis l'avant gauche).
 */
void ESC_Set_Values(uint16_t front_left_motor, uint16_t front_right_motor,
                    uint16_t rear_right_motor, uint16_t rear_left_motor)
{
    ESC_FL_MS(front_left_motor);
    ESC_FR_MS(front_right_motor);
    ESC_RR_MS(rear_right_motor);
    ESC_RL_MS(rear_left_motor);
}

/* Applique les consignes globales m1..m4 (dans l'ordre FL, FR, RR, RL). */
void ESC_Set_Global_Values(void)
{
    ESC_Set_Values(m1, m2, m3, m4);
}

/* Un moteur : commande 0-3000 → compare 3000-6000 (125-250 µs).
 * Grâce à la preload, le nouveau compare est pris en compte à la période suivante :
 * jamais d'impulsion tronquée si on écrit en plein milieu d'une impulsion. */
void ESC_RL_MS(uint16_t x) { LL_TIM_OC_SetCompareCH1(TIM8, ESC_CCR_MIN + esc_clamp(x)); }  /* M1 */
void ESC_FL_MS(uint16_t x) { LL_TIM_OC_SetCompareCH2(TIM8, ESC_CCR_MIN + esc_clamp(x)); }  /* M2 */
void ESC_RR_MS(uint16_t x) { LL_TIM_OC_SetCompareCH3(TIM8, ESC_CCR_MIN + esc_clamp(x)); }  /* M3 */
void ESC_FR_MS(uint16_t x) { LL_TIM_OC_SetCompareCH4(TIM8, ESC_CCR_MIN + esc_clamp(x)); }  /* M4 */

/* Un pad par son NUMÉRO de sérigraphie (1..4 = M1..M4 = TIM8 CH1..CH4), sans
 * présumer de la position du moteur : utilisé par le test de correspondance
 * (motor_test.c). Commande 0-3000, saturée ; pad hors 1..4 ignoré. */
void ESC_Set_Pad(uint8_t pad, uint16_t cmd)
{
    uint32_t ccr = ESC_CCR_MIN + esc_clamp(cmd);
    switch (pad) {
    case 1: LL_TIM_OC_SetCompareCH1(TIM8, ccr); break;
    case 2: LL_TIM_OC_SetCompareCH2(TIM8, ccr); break;
    case 3: LL_TIM_OC_SetCompareCH3(TIM8, ccr); break;
    case 4: LL_TIM_OC_SetCompareCH4(TIM8, ccr); break;
    default: break;
    }
}

/* Impulsion d'arrêt (125 µs) sur les 4 sorties. */
void ESC_Stop_All(void)
{
    for (uint8_t p = 1; p <= 4U; p++) ESC_Set_Pad(p, 0);
}
