/* config.h
 * Configuration persistante de Logolith (sauvegardée en flash, secteur 11).
 *
 * Pour ajouter un champ plus tard : l'ajouter À LA FIN de logolith_config_t et
 * incrémenter CONFIG_VERSION. Une config d'une autre version est ignorée au boot
 * (valeurs par défaut) — il faudra alors refaire les calibrations concernées.
 */
#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <stdint.h>
#include <stdbool.h>

#define CONFIG_VERSION          1U

/* Calibration accel : a_cal [m/s²] = T · (raw − bias), raw en LSB (±16 g) */
typedef struct {
    float bias[3];      /* LSB */
    float T[9];         /* ligne par ligne, (m/s²)/LSB — échelle + couplage inter-axes */
} accel_cal_t;

#define CONFIG_FLAG_ACCEL_CAL   (1UL << 0)

typedef struct {
    uint32_t    version;
    uint32_t    flags;      /* CONFIG_FLAG_* : quelles calibrations sont valides */
    accel_cal_t accel;
} logolith_config_t;

extern logolith_config_t g_config;

/* Charge la config depuis la flash. false si absente/invalide → g_config par défaut. */
bool Config_Load(void);

/* Sauvegarde g_config en flash (drone désarmé uniquement, peut bloquer ~2 s). */
bool Config_Save(void);

#endif
