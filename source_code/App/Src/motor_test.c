/**
 * motor_test.c
 * Test de correspondance moteurs, /!\ HÉLICES DÉMONTÉES /!\.
 *
 * But : savoir, pour chaque pad M1..M4, OÙ est le moteur (avant gauche…) et
 * dans quel SENS il tourne, pour écrire le mixeur Quad-X sans deviner.
 *
 * Déclenchement : drone ARMÉ (SF), gaz en bas, appui sur SD (CH8, poussoir).
 * Séquence : M1, M2, M3, M4, chacun MT_SPIN_MS à MT_CMD (20 % de gaz), avec
 * MT_PAUSE_MS d'arrêt entre deux. La console annonce chaque pad ; on note la
 * position et le sens de rotation du moteur qui tourne.
 *
 * Arrêt immédiat (impulsion d'arrêt sur les 4 sorties) si le drone est désarmé
 * (SF baissé, failsafe) ou si on rappuie sur SD.
 *
 * Pas de conflit avec la calibration accel (SA+SD) : elle exige un drone
 * DÉSARMÉ, ce test un drone ARMÉ.
 *
 * Résultat du test sur Aerolith (banc, vu de dessus, avant = caméra) :
 * M1 = ArG horaire, M2 = AvG anti-horaire, M3 = ArD anti-horaire, M4 = AvD horaire.
 * /!\ Se placer DERRIÈRE le drone (caméra dos à soi) pour le relevé : un premier
 * relevé fait face au drone était tourné de 180°.
 * Avant calibration des ESC : démarrage ~15 %, saturation ~83 % ; après
 * calibration (esc_calib.c), plage complète 0–100 %.
 *
 * Non bloquant : MotorTest_Process() est appelé dans la slow loop après
 * Security_Process(). Hors séquence, ce module n'écrit jamais dans les ESC.
 * /!\ À retirer ou à verrouiller quand le mixeur pilotera les ESC : les deux ne
 * doivent jamais écrire en même temps.
 */
#include "motor_test.h"
#include "esc_driver.h"
#include "security.h"
#include "remote_driver.h"
#include "global.h"
#include "debug.h"
#include <string.h>

#define MT_TRIG_CH       7U      /* index 7 = CH8 = SD (poussoir) */
#define MT_TRIG_HIGH     CRSF_CHANNEL_VALUE_MID
/* 600 / 3000 = 20 % → impulsion de 150 µs. 8 % (135 µs) ne suffisait pas : les
 * ESC BLHeli ont par défaut un "min throttle" vers 1148 µs en PWM, soit ~143 µs en
 * OneShot125 (÷8), sous lequel le moteur ne démarre pas. */
#define MT_CMD           600U
#define MT_SPIN_MS       2000U
#define MT_PAUSE_MS      1000U
#define MT_THROTTLE_MAX  0.05f
#define MT_NB_PADS       4U

/* Libellé de chaque pad (sérigraphie, broche, timer) pour la console */
static const char *const pad_name[MT_NB_PADS] = {
    "M1 (PC6, TIM8 CH1)", "M2 (PC7, TIM8 CH2)",
    "M3 (PC8, TIM8 CH3)", "M4 (PC9, TIM8 CH4)",
};

typedef enum { MT_IDLE, MT_SPIN, MT_PAUSE } mt_state_t;

static mt_state_t state    = MT_IDLE;
static uint8_t    pad      = 0;      /* index 0..3 du pad en cours */
static uint32_t   t_phase  = 0;      /* get_ms() au début de la phase */
static bool       trig_prev = true;  /* true au boot : SD tenu au démarrage ne lance rien */

static void mt_print(const char *s)
{
    print_to_console((char *)s, (uint16_t)strlen(s));
}

static void mt_stop(const char *why)
{
    ESC_Stop_All();
    state = MT_IDLE;
    mt_print("\n\rMOTEST : ");
    mt_print(why);
}

static void mt_spin_pad(uint32_t now)
{
    mt_print("\n\rMOTEST : ");
    mt_print(pad_name[pad]);
    mt_print(" tourne -> noter position et sens");
    ESC_Set_Pad((uint8_t)(pad + 1U), MT_CMD);
    t_phase = now;
    state   = MT_SPIN;
}

void MotorTest_Process(void)
{
    uint32_t now = get_ms();

    /* Front montant de SD (rc_ch est figé en failsafe, mais on est alors
     * désarmé, donc le test est de toute façon arrêté ci-dessous). */
    bool trig   = (rc_ch[MT_TRIG_CH] > MT_TRIG_HIGH);
    bool rising = trig && !trig_prev;
    trig_prev = trig;

    if (state != MT_IDLE) {
        if (!Security_IsArmed()) { mt_stop("arret (desarme)"); return; }
        if (rising)              { mt_stop("arret (SD)");      return; }
    }

    switch (state) {
    case MT_IDLE:
        if (rising && Security_IsArmed() && rc_axes.throttle < MT_THROTTLE_MAX) {
            mt_print("\n\rMOTEST : test moteurs, HELICES DEMONTEES. SD ou SF bas = arret.");
            pad = 0;
            mt_spin_pad(now);
        }
        break;

    case MT_SPIN:
        if ((now - t_phase) >= MT_SPIN_MS) {
            ESC_Stop_All();
            t_phase = now;
            state   = MT_PAUSE;
        }
        break;

    case MT_PAUSE:
        if ((now - t_phase) >= MT_PAUSE_MS) {
            if (++pad >= MT_NB_PADS) mt_stop("termine");
            else                     mt_spin_pad(now);
        }
        break;
    }
}
