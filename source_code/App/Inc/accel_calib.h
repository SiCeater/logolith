/* accel_calib.h
 * Calibration accéléromètre 6 positions, avec détection automatique des faces.
 *
 * Procédure (drone désarmé, moteurs à l'arrêt) :
 *   1. Throttle en bas, SA + SD en position haute pendant 2 s → la calibration démarre.
 *   2. Poser le drone sur une face (dessus, dessous, avant, arrière, gauche, droite),
 *      dans n'importe quel ordre, et ne plus y toucher.
 *        LED bleue clignotante : en attente d'une face immobile
 *        LED bleue fixe        : capture en cours (1 s de stabilisation + 2 s de mesure)
 *        LED rouge fixe        : cette face est déjà faite, passer à une autre
 *      Un mouvement pendant la capture annule cette face (à refaire).
 *   3. Après la 6e face : calcul, vérification, sauvegarde en flash.
 *        LED bleue fixe 3 s : OK          LED rouge clignotante 3 s : échec
 *   Annuler : re-maintenir SA + SD 2 s (après les avoir relâchés), ou timeout 5 min.
 *   Le détail (biais, échelles, fréquence IMU mesurée) est affiché sur la console debug.
 *
 * Modèle : a_cal [m/s²] = T · (raw − bias)
 *   bias = moyenne des 6 mesures (LSB), T = g · D⁻¹ avec D dont la colonne i vaut
 *   (m₊ᵢ − m₋ᵢ)/2. T corrige l'échelle de chaque axe ET le couplage inter-axes /
 *   le désalignement de la carte par rapport aux faces du drone.
 * Correction appliquée en logiciel dans le callback DMA de l'ICM (pas les registres
 * OFFSET_USER, qui ne corrigent que l'offset).
 */
#ifndef __ACCEL_CALIB_H__
#define __ACCEL_CALIB_H__

#include <stdint.h>
#include <stdbool.h>

/* ── Déclenchement radio (confirmé sur la radio de l'utilisateur) ──
 * Index 0-based dans rc_ch[] (CH5 = index 4). SA et SD sont des POUSSOIRS :
 * il faut les garder enfoncés TOUS LES DEUX pendant ACAL_TRIG_HOLD_MS.
 * Valeurs ELRS : repos = 191, appuyé ≈ 1792 (2 positions ELRS = 1000/2000 µs).
 * Avec debug_rc = true, la console affiche les 12 canaux pour vérifier. */
#define ACAL_TRIG_CH_SA             4U      /* CH5 = SA (poussoir) */
#define ACAL_TRIG_CH_SD             7U      /* CH8 = SD (poussoir) */
#define ACAL_TRIG_HIGH              1700U   /* poussoir considéré "appuyé" au-delà (≈1792) */
#define ACAL_TRIG_HOLD_MS           2000U
#define ACAL_TRIG_THROTTLE_MAX      0.05f   /* throttle en bas obligatoire */

/* ── Acquisition (fenêtres alimentées à 8 kHz par l'ISR DMA de l'ICM) ──
 * L'ISR découpe le flux en fenêtres de 100 ms. Pour chacune, la boucle reçoit la
 * moyenne accel et la variation (crête-crête) de l'accel et du gyro.
 * NB : les durées supposent un DRDY à 8 kHz. Si l'IMU tournait plus lentement,
 * tout serait simplement plus long (fenêtres de 800 échantillons quand même). */
#define ACAL_WIN_SAMPLES            800U    /* 100 ms @ 8 kHz */
#define ACAL_SETTLE_WIN             10U     /* 1 s immobile avant de mesurer : laisse amortir
                                               les vibrations après avoir posé le drone */
#define ACAL_CAPTURE_WIN            20U     /* 2 s de moyenne par face (16 000 échantillons) :
                                               le bruit accel moyenné devient << 0,1 mg */

/* Détection d'immobilité. Le bruit au repos est bien plus faible (accel ~1-2 mg
 * crête-crête après l'AAF à 42 Hz, gyro < 1 °/s crête-crête). Un drone posé
 * sur une table passe donc largement, et une main qui le tient ne passe pas.
 * Si ça ne capture jamais sur ton banc (table qui vibre, ventilateur…) : augmenter.
 * Si ça capture alors que tu tiens le drone : diminuer. */
#define ACAL_STILL_ACC_PTP_LSB      80      /* crête-crête accel max par fenêtre : 80/2048 ≈ 40 mg */
#define ACAL_STILL_GYR_PTP_LSB      64      /* crête-crête gyro max par fenêtre : 64/16,4 ≈ 4 °/s */

/* Reconnaissance d'une face. Au repos, |a| = 1 g : une norme très différente
 * veut dire que le drone bouge. L'axe dominant doit porter quasiment toute la
 * gravité : cos(18°) ≈ 0,95, donc face inclinée de moins de ~18°.
 * Ce seuil sert seulement à reconnaître la face. La précision de la
 * calibration suppose que le drone est posé bien à plat (voir accel_calib.c). */
#define ACAL_POSE_NORM_TOL          0.10f   /* |a| doit être à ±10 % de 1 g */
#define ACAL_POSE_ALIGN_MIN         0.95f   /* axe dominant ≥ 95 % de |a| */
#define ACAL_TIMEOUT_MS             300000U /* 5 min pour faire les 6 faces, sinon annulation */

/* ── Contrôles de vraisemblance du résultat ──
 * Larges par rapport aux specs de l'ICM-42688-P (offset typ. ±20 mg, sensibilité
 * typ. ±0,5 %, couplage typ. ±1 %). Le but n'est pas de juger le capteur, mais
 * d'attraper une calibration ratée : face mal posée, choc pendant une capture.
 * Un échec ne modifie rien, l'ancienne calibration reste en place. */
#define ACAL_BIAS_MAX_LSB           410.0f  /* biais max 0,2 g par axe */
#define ACAL_SCALE_TOL              0.05f   /* échelle de chaque axe à ±5 % du nominal */
#define ACAL_CROSS_MAX              0.10f   /* couplage inter-axes < 10 % (~6° de désalignement) */
#define ACAL_RESIDUAL_MAX           0.02f   /* après correction, chaque face doit donner 1 g à ±2 % */

#define ACAL_LSB_PER_G              2048.0f /* ±16 g */
#define ACAL_GRAVITY                9.80665f

/* Charge la calibration depuis g_config (appeler Config_Load() avant). */
void AccelCal_Init(void);

/* ISR DMA ICM (8 kHz) : alimente l'acquisition si une calibration est en cours. */
void AccelCal_Feed(const int16_t raw_a[3], const int16_t raw_g[3]);

/* ISR DMA ICM (8 kHz) : raw (LSB) → m/s² calibrés (échelle nominale si pas de calibration). */
void AccelCal_Apply(const int16_t raw_a[3], float out_mps2[3]);

/* Boucle principale : déclenchement radio, machine d'état, LEDs, calcul, sauvegarde. */
void AccelCal_Process(void);

bool AccelCal_IsValid(void);     /* une calibration valide est appliquée */
bool AccelCal_InProgress(void);  /* calibration en cours → interdire l'armement */

#endif
