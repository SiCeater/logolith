/**
 * esc_driver.c
 * Commande des 4 ESC en OneShot125 via TIM8 (4 canaux), en mode ONE-PULSE
 * déclenché par la boucle rapide.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * PROTOCOLE ONESHOT125
 * ═══════════════════════════════════════════════════════════════════════════
 * Une impulsion par mise à jour, dont la LARGEUR code les gaz :
 *     125 µs = arrêt    …    250 µs = plein gaz
 * (= le PWM servo classique 1000-2000 µs divisé par 8.) Le nom « OneShot » vient
 * de là : l'impulsion est censée partir UNE fois, juste après chaque calcul du
 * contrôleur, et non sur une horloge libre.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * POURQUOI LE MODE ONE-PULSE (et plus le PWM libre à 1 kHz)
 * ═══════════════════════════════════════════════════════════════════════════
 * Avant : TIM8 en PWM libre à 1000,0 Hz (quartz du STM32), la boucle à ~996 Hz
 * (oscillateur interne de l'ICM, ±1,5 %). Deux horloges indépendantes qui
 * glissent l'une sur l'autre :
 *   - retard entre calcul et impulsion variable de 0 à 1 ms (dent de scie à
 *     ~4 Hz), soit jusqu'à une période d'échantillonnage de retard en plus,
 *     et variable : c'est de la gigue injectée dans la boucle ;
 *   - toutes les ~250 ms, une impulsion répète l'ancienne consigne (la boucle
 *     n'a pas fini sa période quand TIM8 repart).
 * Maintenant : chaque itération de la fast loop appelle ESC_Update(), qui
 * charge les 4 largeurs et lance UN cycle de TIM8. L'impulsion part quelques µs
 * après le calcul, avec un retard FIXE, et suit exactement le rythme de l'IMU.
 *
 * Bonus sécurité : si la fast loop s'arrête (IMU muette, DMA bloqué), plus
 * aucune impulsion ne sort et les ESC coupent les moteurs sur perte de signal,
 * au lieu de garder la dernière consigne pour toujours comme avec le PWM libre.
 * Corollaire : AUCUNE sortie tant que la fast loop ne tourne pas (pendant
 * l'init de l'ICM au boot) ; les ESC attendent simplement un signal.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * RÉGLAGE DE TIM8 (reconfiguré ici à l'exécution, par-dessus CubeMX)
 * ═══════════════════════════════════════════════════════════════════════════
 * Horloge timer 168 MHz, Prescaler = 6 (CubeMX) → 24 MHz, 1 tick = 41,67 ns.
 *     125 µs = 3000 ticks           250 µs = 6000 ticks
 * CubeMX génère un PWM1 libre (ARR 23999, preload des compares) : ESC_Init le
 * transforme en one-pulse. Le .ioc n'est pas modifié, MX_TIM8_Init ne démarre
 * pas le compteur et n'active pas les sorties, donc rien ne sort avant ESC_Init.
 *
 *   - OPM = 1 : à l'update (CNT = ARR → 0), le compteur s'arrête tout seul (CEN = 0).
 *   - ARR = ESC_OPM_ARR = 6000 : un cycle dure 6001 ticks = 250 µs.
 *   - PWM MODE 2 (sortie active quand CNT ≥ CCR), et non PWM1 : au repos le
 *     compteur est arrêté à 0. En PWM1 (active quand CNT < CCR), 0 < CCR mettrait
 *     les sorties à l'état HAUT en permanence entre deux impulsions ! En PWM2,
 *     0 < CCR → sortie basse au repos.
 *   - L'impulsion va donc de CNT = CCR à CNT = ARR inclus : largeur =
 *     ARR + 1 − CCR ticks. Pour une largeur w = 3000 + cmd :
 *         CCR = ESC_OPM_ARR + 1 − (3000 + cmd) = 3001 − cmd   ∈ [1, 3001]
 *     Les 4 impulsions FINISSENT ensemble (fin de cycle) et commencent à des
 *     instants différents ; l'ESC ne mesure que la largeur, ça lui est égal.
 *   - Preload des compares DÉSACTIVÉE : on écrit les CCR compteur arrêté, ils
 *     s'appliquent tout de suite. ESC_Update ne relance jamais un cycle en cours
 *     (test de CEN) : pas d'impulsion tronquée possible.
 *
 * Cadence résultante ≈ 996 Hz (celle de l'IMU), compatible OneShot125 (période
 * > 250 µs : max ~3,9 kHz).
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * QUI ÉCRIT LES COMMANDES
 * ═══════════════════════════════════════════════════════════════════════════
 * Les fonctions ESC_Set_* n'écrivent pas le timer : elles rangent la commande
 * dans esc_cmd[] (une case 16 bits par moteur, écriture atomique). ESC_Update
 * (fast loop, ISR) les envoie. Les modes banc (motor_test, esc_calib) écrivent
 * depuis la slow loop, le futur mixeur depuis la fast loop elle-même.
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
 * PLAGE DES ESC : CALIBRATION OBLIGATOIRE (esc_calib.c)
 * ═══════════════════════════════════════════════════════════════════════════
 * Mesuré au banc AVANT calibration : le moteur démarrait vers 15 % et n'accélère plus au-delà de
 * ~83 %. Ces deux bornes tombent pile sur les réglages PAR DÉFAUT des ESC BLHeli
 * en entrée PWM : min throttle 1148 µs, max throttle 1832 µs. En OneShot125 la
 * durée est divisée par 8 : 143,5 µs et 229 µs, soit
 *     (143,5 − 125) / 125 = 14,8 %      (229 − 125) / 125 = 83,2 %
 * Ce n'est donc pas l'absence d'hélice (sans charge le moteur accélérerait
 * jusqu'à son max, sans plafond net), mais l'ESC qui sature : tout ce qui
 * dépasse 229 µs est lu comme plein gaz, tout ce qui est sous 143,5 µs comme arrêt.
 * Conséquence : on perd ~32 % de la résolution de commande. Remède : calibrer la
 * plage des ESC (mode radio esc_calib.c, ou min/max throttle 1000/2000 dans BLHeliSuite),
 * Fait et validé au banc sur Aerolith : après calibration, les moteurs répondent
 * sur toute la plage 0–100 %. À refaire sur tout nouveau jeu d'ESC. Le passage au
 * DShot (numérique, sans calibration) supprimera cette étape.
 */
#include "esc_driver.h"
#include "global.h"
#include <stdint.h>

/* Commandes en attente d'émission, index 0..3 = pads M1..M4. 0 = arrêt. */
static volatile uint16_t esc_cmd[4] = { 0U, 0U, 0U, 0U };

/* Nb d'appels à ESC_Update arrivés alors qu'un cycle tournait encore
 * (n'arrive que si deux itérations sont à moins de 250 µs : jamais à 1 kHz). */
volatile uint32_t esc_update_skipped = 0;

/* Sature la commande à [0, ESC_CMD_MAX]. Le type est non signé : pas de cas < 0. */
static inline uint16_t esc_clamp(uint16_t x)
{
    return (x > ESC_CMD_MAX) ? (uint16_t)ESC_CMD_MAX : x;
}

/* Commande 0..3000 → compare PWM2 en one-pulse (largeur 3000 + cmd ticks). */
static inline uint32_t esc_ccr(uint16_t cmd)
{
    return (uint32_t)ESC_OPM_ARR + 1U - (ESC_CCR_MIN + (uint32_t)esc_clamp(cmd));
}

/**
 * @brief Passe TIM8 en one-pulse (voir en-tête) et arme les 4 sorties à l'arrêt.
 * @note  N'émet RIEN : la 1re impulsion part au 1er ESC_Update (fast loop). Les
 *        ESC s'arment ("bips") dès qu'ils reçoivent des impulsions d'arrêt.
 *        Cette fonction ne fait jamais tourner un moteur.
 */
void ESC_Init(void)
{
    LL_TIM_DisableCounter(TIM8);
    LL_TIM_SetOnePulseMode(TIM8, LL_TIM_ONEPULSEMODE_SINGLE);
    LL_TIM_DisableARRPreload(TIM8);
    LL_TIM_SetAutoReload(TIM8, ESC_OPM_ARR);

    static const uint32_t ch[4] = { LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2,
                                    LL_TIM_CHANNEL_CH3, LL_TIM_CHANNEL_CH4 };
    for (uint8_t i = 0; i < 4U; i++) {
        LL_TIM_OC_SetMode(TIM8, ch[i], LL_TIM_OCMODE_PWM2);
        LL_TIM_OC_DisablePreload(TIM8, ch[i]);
    }
    LL_TIM_OC_SetCompareCH1(TIM8, esc_ccr(0));
    LL_TIM_OC_SetCompareCH2(TIM8, esc_ccr(0));
    LL_TIM_OC_SetCompareCH3(TIM8, esc_ccr(0));
    LL_TIM_OC_SetCompareCH4(TIM8, esc_ccr(0));
    LL_TIM_SetCounter(TIM8, 0);   /* repos : CNT = 0 < CCR → sorties basses (PWM2) */

    /* TIM8 est un timer "advanced" : en plus de l'activation de chaque canal, ses
     * sorties ne sortent sur les broches que si MOE (Main Output Enable) est à 1,
     * d'où LL_TIM_EnableAllOutputs. Compteur NON démarré. */
    for (uint8_t i = 0; i < 4U; i++) LL_TIM_CC_EnableChannel(TIM8, ch[i]);
    LL_TIM_EnableAllOutputs(TIM8);

    if (debug_init)
        print_to_console("\n\rESCs : initialized (OneShot125 one-pulse, fired by fast loop)", 63);
}

/**
 * @brief Émet UNE impulsion OneShot125 sur les 4 sorties avec esc_cmd[].
 * @note  Appelée à chaque itération de la fast loop (ISR DMA de l'ICM). Durée
 *        ~1 µs. Si un cycle est encore en cours (CEN = 1), on ne touche à rien.
 */
void ESC_Update(void)
{
    if (LL_TIM_IsEnabledCounter(TIM8)) { esc_update_skipped++; return; }
    LL_TIM_OC_SetCompareCH1(TIM8, esc_ccr(esc_cmd[0]));
    LL_TIM_OC_SetCompareCH2(TIM8, esc_ccr(esc_cmd[1]));
    LL_TIM_OC_SetCompareCH3(TIM8, esc_ccr(esc_cmd[2]));
    LL_TIM_OC_SetCompareCH4(TIM8, esc_ccr(esc_cmd[3]));
    LL_TIM_EnableCounter(TIM8);   /* CNT part de 0, s'arrête seul à l'update */
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

/* Un moteur par sa position (commande 0-3000, émise au prochain ESC_Update). */
void ESC_RL_MS(uint16_t x) { esc_cmd[0] = esc_clamp(x); }  /* M1 */
void ESC_FL_MS(uint16_t x) { esc_cmd[1] = esc_clamp(x); }  /* M2 */
void ESC_RR_MS(uint16_t x) { esc_cmd[2] = esc_clamp(x); }  /* M3 */
void ESC_FR_MS(uint16_t x) { esc_cmd[3] = esc_clamp(x); }  /* M4 */

/* Un pad par son NUMÉRO de sérigraphie (1..4 = M1..M4 = TIM8 CH1..CH4), sans
 * présumer de la position du moteur : utilisé par motor_test et esc_calib.
 * Commande 0-3000, saturée ; pad hors 1..4 ignoré. */
void ESC_Set_Pad(uint8_t pad, uint16_t cmd)
{
    if (pad >= 1U && pad <= 4U) esc_cmd[pad - 1U] = esc_clamp(cmd);
}

/* Commande d'arrêt (125 µs) sur les 4 sorties, à partir de la prochaine impulsion. */
void ESC_Stop_All(void)
{
    for (uint8_t p = 1; p <= 4U; p++) ESC_Set_Pad(p, 0);
}
