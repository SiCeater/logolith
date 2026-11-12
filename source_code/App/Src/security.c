/**
 * security.c
 * Gestion des situations dangereuses (perte du lien radio, armement…).
 *
 * ÉTAT : SQUELETTE, rien n'est branché. connection_lost_routine() n'est appelée
 * nulle part. La détection de perte de lien existe déjà dans remote_driver.c :
 * rc_failsafe.state == FAILSAFE_ACTIVE après CRSF_FAILSAFE_TIMEOUT_MS sans trame,
 * et dans ce cas le throttle est coupé et armed est forcé à false.
 *
 * À FAIRE ici (voir aussi "Known TODOs" dans le README) :
 *   - Réarmement : après un failsafe ou une calibration, exiger que le switch
 *     d'armement soit d'abord BAISSÉ avant d'autoriser un nouvel armement
 *     (sinon le drone se réarme tout seul au retour du lien).
 *   - Refuser l'armement si AccelCal_InProgress() ou si !AccelCal_IsValid().
 *   - Ne pas appeler de fonction bloquante ici : le do/while commenté plus bas
 *     bloquerait toute la boucle principale (RC, calibration…) pendant la perte de lien.
 *   - Attention aux LEDs : accel_calib les utilise aussi pendant une calibration.
 */
#include "security.h"
#include "global.h"

void connection_lost_routine(void)
{
    if (debug_warn)
        print_to_console("\nremote connection lost !\n", 26);

    // ICI DESACTIVER LA LOOP CONTROL ET SECURITY
    R_LED_On();

    // do
    // {

    // }
    // while(connection non re-established);

    R_LED_Off();

    if(debug_warn)
        print_to_console("\nremote connection re-etablished !\n", 35);
}
