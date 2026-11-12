/**
 * config.c
 * Configuration persistante : une seule struct (logolith_config_t) en RAM,
 * g_config, sauvegardée/chargée en bloc via flash_storage.
 *
 * Pourquoi un bloc unique plutôt qu'un enregistrement par calibration : chaque
 * sauvegarde écrit la config complète, donc la dernière version en flash est
 * toujours cohérente. Sauver la calibration accel conserve les autres champs,
 * puisqu'ils sont dans g_config.
 *
 * Évolution du format (ex. ajout des gains PID) :
 *   1. ajouter le champ À LA FIN de logolith_config_t (config.h) ;
 *   2. incrémenter CONFIG_VERSION.
 *   Au boot, l'ancienne config (taille et version différentes) est ignorée :
 *   valeurs par défaut, et les calibrations sont à refaire. Choix volontaire :
 *   on n'interprète jamais des octets selon un autre format que celui avec
 *   lequel ils ont été écrits.
 */
#include "config.h"
#include "flash_storage.h"
#include <string.h>

/* Erreur de compilation si la config ne tient plus dans un slot flash (244 octets
 * de payload max) : agrandir alors FLASH_STORE_SLOT_SIZE dans flash_storage.h. */
_Static_assert(sizeof(logolith_config_t) <= FLASH_STORE_MAX_PAYLOAD,
               "logolith_config_t trop gros pour un slot flash_storage");

logolith_config_t g_config;

/* Défauts : tout à zéro, aucune calibration valide (flags = 0). */
static void config_defaults(logolith_config_t *c)
{
    memset(c, 0, sizeof(*c));
    c->version = CONFIG_VERSION;
}

bool Config_Load(void)
{
    /* Chargement dans une copie locale : g_config n'est remplacée que si la
     * config lue est complète et de la bonne version. */
    logolith_config_t c;
    if (FlashStore_Load(&c, sizeof(c)) && c.version == CONFIG_VERSION) {
        g_config = c;
        return true;
    }
    config_defaults(&g_config);   /* 1er boot, mass erase, ou format modifié */
    return false;
}

bool Config_Save(void)
{
    g_config.version = CONFIG_VERSION;
    return FlashStore_Save(&g_config, sizeof(g_config));
}
