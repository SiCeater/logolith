/**
 * security.c
 * Armement / désarmement et réaction à la perte du lien radio.
 *
 * ── Switch d'armement : SF (CH10) ─────────────────────────────────────────
 * SF est un poussoir VERROUILLABLE : il reste dans sa position, c'est donc un
 * vrai interrupteur. SA (CH5) ne convient pas : c'est un poussoir à rappel
 * (relâché = désarmé), et il sert déjà au déclencheur SA+SD de la calibration
 * accel (appuyer SA+SD aurait armé le drone).
 *
 * ── Règles ────────────────────────────────────────────────────────────────
 * 1. On n'arme que sur un FRONT MONTANT de SF (bas → haut), jamais sur un
 *    niveau. Conséquences, toutes voulues :
 *      - SF déjà levé au boot             → pas d'armement tant qu'il n'a pas
 *                                           été baissé une fois ;
 *      - retour du lien après un failsafe → idem (sinon le drone se réarmerait
 *                                           tout seul si SF était resté levé) ;
 *      - armement refusé (gaz pas en bas…) → il faut baisser puis relever SF
 *        une fois le problème corrigé (pas d'armement "surprise" plus tard,
 *        par exemple au moment où la calibration accel se termine).
 * 2. Conditions à remplir au moment du front montant, sinon refus (message) :
 *      - lien radio OK (pas de failsafe, LQ ≥ 50 % → FAILSAFE_OK) ;
 *      - gaz en bas (< ARM_THROTTLE_MAX) ;
 *      - pas de calibration accel en cours ;
 *      - une calibration accel valide est chargée (sans elle l'horizon du
 *        futur mode ANGLE serait faux).
 * 3. Désarmement immédiat si SF est baissé ou si le lien est perdu
 *    (FAILSAFE_ACTIVE, ~100 ms sans trame valide).
 *
 * Réciproquement, accel_calib refuse de démarrer une calibration si le drone
 * est armé (Security_IsArmed()).
 *
 * /!\ Aucun moteur ne dépend encore de Security_IsArmed() : la fast loop / PID
 * et le mixeur devront couper les sorties ESC (impulsion d'arrêt) dès qu'il
 * repasse à false.
 *
 * Tout est non bloquant (appelé depuis la slow loop ~500 Hz). Les LEDs ne sont
 * pas utilisées ici : accel_calib s'en sert pendant une calibration.
 */
#include "security.h"
#include "global.h"
#include "debug.h"
#include "remote_driver.h"
#include "accel_calib.h"
#include "esc_calib.h"
#include <string.h>

#define ARM_SWITCH_CH       9U      /* index 9 = CH10 = SF (poussoir verrouillable) */
#define ARM_SWITCH_HIGH     CRSF_CHANNEL_VALUE_MID   /* repos 191, actif ≈ 1792 */
#define ARM_THROTTLE_MAX    0.05f   /* gaz < 5 % obligatoires pour armer */

static bool armed     = false;
static bool sw_prev   = true;   /* true au boot : SF levé au démarrage n'arme pas (règle 1) */
static bool link_prev = true;   /* état du lien au tour précédent (messages perte / retour) */

static void sec_print(const char *s)
{
    if (debug_warn) print_to_console((char *)s, (uint16_t)strlen(s));
}

static void disarm(const char *why)
{
    if (!armed) return;
    armed = false;
    sec_print("\n\rARM : DESARME (");
    sec_print(why);
    sec_print(")");
}

/* Raison du refus d'armement, ou NULL si toutes les conditions sont remplies. */
static const char *arm_refusal(void)
{
    if (rc_failsafe.state != FAILSAFE_OK)  return "lien radio faible";
    if (rc_axes.throttle >= ARM_THROTTLE_MAX) return "gaz pas en bas";
    if (AccelCal_InProgress())             return "calibration accel en cours";
    if (EscCal_InProgress())               return "calibration ESC en cours";
    if (!AccelCal_IsValid())               return "pas de calibration accel";
    return NULL;
}

void Security_Process(void)
{
    /* ── Lien radio ── */
    bool link_ok = (rc_failsafe.state != FAILSAFE_ACTIVE);
    if (link_ok != link_prev) {
        sec_print(link_ok ? "\n\rRC : lien radio retabli" : "\n\rRC : LIEN RADIO PERDU (failsafe)");
        link_prev = link_ok;
    }
    if (!link_ok) {
        disarm("failsafe");
        /* rc_ch est figé sur ses dernières valeurs pendant le failsafe : on
         * force sw_prev à "levé" pour qu'au retour du lien, SF doive être vu
         * BAS avant qu'un front montant puisse réarmer (règle 1). */
        sw_prev = true;
        return;
    }

    /* ── Switch d'armement ── */
    bool sw     = (rc_ch[ARM_SWITCH_CH] > ARM_SWITCH_HIGH);
    bool rising = sw && !sw_prev;
    sw_prev = sw;

    if (!sw) { disarm("switch"); return; }

    if (rising && !armed) {
        const char *why = arm_refusal();
        if (why != NULL) {
            sec_print("\n\rARM : refuse (");
            sec_print(why);
            sec_print("), baisser puis relever SF");
        } else {
            armed = true;
            sec_print("\n\rARM : ARME");
        }
    }
}

bool Security_IsArmed(void)
{
    return armed;
}
