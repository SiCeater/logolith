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
 * CORRESPONDANCE MOTEURS ↔ CANAUX  /!\ À VÉRIFIER SUR LE DRONE /!\
 * ═══════════════════════════════════════════════════════════════════════════
 * Sorties de la LithCore (sérigraphie M1..M8 = labels CubeMX ESC_Mx, confirmés) :
 *   M1 = PC6 TIM8_CH1   M2 = PC7 TIM8_CH2   M3 = PC8 TIM8_CH3   M4 = PC9 TIM8_CH4
 *   M5 = PA8 TIM1_CH1   M6 = PA9 TIM1_CH2   M7 = PA10 TIM1_CH3  M8 = PB1 TIM3_CH4
 *
 * Ce que fait le code aujourd'hui (fonction → sortie) :
 *   ESC_RL_MS → CH1 → M1        ESC_FL_MS → CH2 → M2
 *   ESC_RR_MS → CH3 → M3        ESC_FR_MS → CH4 → M4
 * La sortie ↔ pad est sûre ; ce qui reste À VÉRIFIER, c'est la position du moteur
 * branché sur chaque pad (avant gauche, etc.). Avant le premier vol : ESC_Test()
 * (hélices démontées) pour vérifier que chaque fonction fait tourner le bon moteur,
 * ET le sens de rotation attendu par le mixer.
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
 * @brief /!\ CALIBRATION DE PLAGE DES ESC — HÉLICES DÉMONTÉES /!\
 *
 * Apprend aux ESC les bornes du signal : plein gaz (250 µs) pendant 2 s, puis
 * arrêt (125 µs) pendant 2 s. Procédure standard : alimenter les ESC (batterie)
 * PENDANT que le plein gaz est envoyé, ils entrent alors en mode calibration.
 *
 * DANGER : si les ESC sont déjà alimentés et armés quand cette fonction est
 * appelée (reboot du FC sur batterie, par exemple), les moteurs partent à plein
 * régime pendant 2 s. Elle était auparavant dans ESC_Init, donc exécutée à chaque
 * boot : elle en a été sortie. À lancer seulement volontairement, sur le banc,
 * une fois par jeu d'ESC (les ESC mémorisent la plage).
 */
void ESC_Calibrate(void)
{
    R_LED_On();
    ESC_Set_Values(ESC_CMD_MAX, ESC_CMD_MAX, ESC_CMD_MAX, ESC_CMD_MAX);  /* 250 µs = plein gaz */
    LL_mDelay(2000);
    ESC_Set_Values(0, 0, 0, 0);                                          /* 125 µs = arrêt */
    LL_mDelay(2000);
    R_LED_Off();

    if (debug_init)
        print_to_console("\n\rESCs : range calibrated", 25);
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
void ESC_RL_MS(uint16_t x) { LL_TIM_OC_SetCompareCH1(TIM8, ESC_CCR_MIN + esc_clamp(x)); }
void ESC_FL_MS(uint16_t x) { LL_TIM_OC_SetCompareCH2(TIM8, ESC_CCR_MIN + esc_clamp(x)); }
void ESC_RR_MS(uint16_t x) { LL_TIM_OC_SetCompareCH3(TIM8, ESC_CCR_MIN + esc_clamp(x)); }
void ESC_FR_MS(uint16_t x) { LL_TIM_OC_SetCompareCH4(TIM8, ESC_CCR_MIN + esc_clamp(x)); }

/**
 * @brief /!\ HÉLICES DÉMONTÉES /!\ Test moteur par moteur (bloquant ~8 s) :
 *        FL, FR, RR, RL, chacun 1 s à mi-gaz puis 1 s à plein gaz.
 *        Sert à vérifier la correspondance fonction ↔ moteur (tableau en tête).
 */
void ESC_Test(void)
{
    ESC_FL_MS(1500); LL_mDelay(1000); ESC_FL_MS(3000); LL_mDelay(1000); ESC_FL_MS(0);
    ESC_FR_MS(1500); LL_mDelay(1000); ESC_FR_MS(3000); LL_mDelay(1000); ESC_FR_MS(0);
    ESC_RR_MS(1500); LL_mDelay(1000); ESC_RR_MS(3000); LL_mDelay(1000); ESC_RR_MS(0);
    ESC_RL_MS(1500); LL_mDelay(1000); ESC_RL_MS(3000); LL_mDelay(1000); ESC_RL_MS(0);
}
