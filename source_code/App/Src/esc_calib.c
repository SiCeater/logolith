/**
 * esc_calib.c
 * Calibration de plage des ESC à la radio, /!\ HÉLICES DÉMONTÉES /!\.
 *
 * ── Pourquoi ──────────────────────────────────────────────────────────────
 * Non calibrés, les ESC BLHeli gardent leur plage par défaut 1148–1832 µs (PWM),
 * soit 143,5–229 µs en OneShot125 : le moteur ne démarre qu'à ~15 % et sature à
 * ~83 % de notre commande (cf. esc_driver.c). Calibrer leur apprend la vraie
 * plage 125–250 µs, donc toute la résolution de commande.
 *
 * ── Principe de la calibration BLHeli ───────────────────────────────────
 * Si l'ESC reçoit, au moment où il est ALIMENTÉ, un signal plus long que son
 * seuil "plein gaz", il entre en calibration : il mémorise ce signal comme
 * plein gaz (bips), puis le signal bas qui suit comme arrêt (bips de
 * confirmation). Ici la sortie suit directement le manche des gaz : c'est
 * l'utilisateur qui fait la procédure à la main, au rythme des bips.
 *
 * ── Procédure (batterie DÉBRANCHÉE, FC alimenté en USB) ─────────────────
 *   1. Désarmé, gaz EN HAUT, SA + SD tenus 2 s → mode calibration (LED rouge).
 *   2. AVERTISSEMENT : tant que le manche des gaz n'est pas redescendu, les
 *      4 moteurs "clignotent" à 1 Hz entre arrêt et ESC_CMD_START (~15 %).
 *      Batterie débranchée, il ne se passe rien. Si au contraire la batterie
 *      est branchée (et des hélices montées !), les moteurs donnent des à-coups
 *      bien audibles : on sait qu'on est entré dans ce mode par erreur, on en
 *      sort avec SA + SD.
 *   3. Gaz en bas → MANCHE DIRECT : sortie = gaz (0 % = 125 µs … 100 % = 250 µs),
 *      sans expo ni filtre (rc_axes.throttle, saturé à 0/1 aux extrémités).
 *   4. Remonter les gaz à fond, BRANCHER LA BATTERIE, attendre les bips
 *      "plein gaz enregistré", redescendre les gaz, attendre les bips de fin.
 *   5. SA + SD → sortie du mode : impulsion d'arrêt sur les 4 moteurs.
 * Ensuite : remesurer le seuil de démarrage (ESC_CMD_START) avec motor_test.
 *
 * ── Sécurités ────────────────────────────────────────────────────────────
 *   - entrée seulement désarmé, lien radio sain, sans calibration accel en
 *     cours ; il faut avoir relâché SA + SD depuis la dernière utilisation ;
 *   - pendant le mode : armement refusé (security.c) et calibration accel
 *     bloquée (accel_calib.c), donc personne d'autre n'écrit dans les ESC ;
 *   - perte du lien radio (failsafe) → sortie immédiate, impulsion d'arrêt ;
 *   - SA + SD tenus ESCCAL_EXIT_HOLD_MS → sortie, à tout moment, quelle que
 *     soit la position des gaz.
 *
 * Gaz en haut à l'entrée (et en bas pour la calibration accel) : les deux
 * combinaisons SA + SD ne peuvent pas se confondre.
 */
#include "esc_calib.h"
#include "esc_driver.h"
#include "security.h"
#include "accel_calib.h"
#include "remote_driver.h"
#include "led_driver.h"
#include "global.h"
#include "debug.h"
#include <string.h>

typedef enum {
    ESCCAL_IDLE,    /* mode inactif, ce module n'écrit pas dans les ESC */
    ESCCAL_WARN,    /* entré, gaz encore en haut : clignotement moteurs 1 Hz */
    ESCCAL_DIRECT,  /* manche des gaz branché en direct sur les 4 ESC */
} esccal_state_t;

static esccal_state_t state = ESCCAL_IDLE;
static uint32_t       t_blink;              /* get_ms() du dernier basculement */
static bool           blink_on;             /* moteurs à ESC_CMD_START (true) ou arrêt */

/* Suivi de l'appui SA + SD (entrée et sortie).
 * combo_rearmed démarre à false : SA + SD tenus au boot ne déclenchent rien
 * tant qu'ils n'ont pas été relâchés une fois. */
static bool     combo_rearmed = false;
static bool     combo_holding = false;
static uint32_t combo_since;

static void ec_print(const char *s)
{
    print_to_console((char *)s, (uint16_t)strlen(s));
}

static void set_all(uint16_t cmd)
{
    for (uint8_t p = 1; p <= 4U; p++) ESC_Set_Pad(p, cmd);
}

/* Renvoie true UNE fois quand SA + SD sont tenus hold_ms d'affilée ; il faut
 * ensuite les relâcher pour pouvoir redéclencher (entrée puis sortie avec le
 * même geste, sans rebond). */
static bool combo_event(uint32_t now, uint32_t hold_ms)
{
    bool on = rc_ch[ACAL_TRIG_CH_SA] > ACAL_TRIG_HIGH &&
              rc_ch[ACAL_TRIG_CH_SD] > ACAL_TRIG_HIGH;

    if (!on) { combo_holding = false; combo_rearmed = true; return false; }
    if (!combo_holding) { combo_holding = true; combo_since = now; }
    if (combo_rearmed && (now - combo_since) >= hold_ms) {
        combo_rearmed = false;
        return true;
    }
    return false;
}

static void exit_mode(const char *why)
{
    ESC_Stop_All();
    R_LED_Off();
    state = ESCCAL_IDLE;
    ec_print("\n\rESCCAL : sortie (");
    ec_print(why);
    ec_print("), impulsion d'arret sur les 4 moteurs");
}

void EscCal_Process(void)
{
    uint32_t now   = get_ms();
    bool     link  = rc_is_valid() && rc_failsafe.state != FAILSAFE_ACTIVE;
    float    thr   = rc_axes.throttle;

    switch (state) {
    case ESCCAL_IDLE: {
        /* combo_event appelé à chaque tour pour suivre relâchement / appui */
        bool trig = combo_event(now, ESCCAL_TRIG_HOLD_MS);
        if (trig && link && !Security_IsArmed() && !AccelCal_InProgress() &&
            thr > ESCCAL_THROTTLE_HIGH) {
            ec_print("\n\rESCCAL : MODE CALIBRATION ESC, HELICES DEMONTEES !"
                     "\n\r         moteurs pulses a 1 Hz tant que les gaz sont en haut,"
                     "\n\r         baisser les gaz pour passer en manche direct. SA+SD = sortie.");
            R_LED_On();
            blink_on = true;
            set_all(ESC_CMD_START);
            t_blink = now;
            state   = ESCCAL_WARN;
        }
        break;
    }

    case ESCCAL_WARN:
        if (!link)                                       { exit_mode("lien radio perdu"); break; }
        if (combo_event(now, ESCCAL_EXIT_HOLD_MS))       { exit_mode("SA+SD");            break; }
        if (thr < ESCCAL_THROTTLE_LOW) {
            ESC_Stop_All();
            ec_print("\n\rESCCAL : manche direct. Gaz a fond, brancher la batterie, suivre les bips"
                     "\n\r         (plein gaz enregistre -> baisser les gaz -> bips de fin).");
            state = ESCCAL_DIRECT;
            break;
        }
        if ((now - t_blink) >= ESCCAL_BLINK_HALF_MS) {
            blink_on = !blink_on;
            set_all(blink_on ? ESC_CMD_START : 0U);
            t_blink = now;
        }
        break;

    case ESCCAL_DIRECT:
        if (!link)                                       { exit_mode("lien radio perdu"); break; }
        if (combo_event(now, ESCCAL_EXIT_HOLD_MS))       { exit_mode("SA+SD");            break; }
        /* thr est déjà saturé à [0, 1] : 0 → 125 µs, 1 → 250 µs exactement */
        set_all((uint16_t)(thr * (float)ESC_CMD_MAX + 0.5f));
        break;
    }
}

bool EscCal_InProgress(void)
{
    return state != ESCCAL_IDLE;
}
