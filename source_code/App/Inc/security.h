#ifndef __SECURITY_H__
#define __SECURITY_H__

#include <stdbool.h>

/* Armement sur SF (CH10) + suivi du lien radio : voir security.c.
 * À appeler dans la slow loop, JUSTE APRÈS Remote_Process (utilise rc_ch,
 * rc_axes et rc_failsafe tout juste mis à jour). Non bloquant. */
void Security_Process(void);

/* true = drone armé. Repasse à false dès que SF est baissé ou le lien perdu. */
bool Security_IsArmed(void);

#endif
