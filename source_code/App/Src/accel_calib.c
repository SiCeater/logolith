/**
 * accel_calib.c
 * Calibration accéléromètre 6 faces avec détection automatique des faces.
 * Mode d'emploi côté utilisateur : voir accel_calib.h.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * POURQUOI CALIBRER L'ACCEL
 * ═══════════════════════════════════════════════════════════════════════════
 * Un accéléromètre MEMS réel mesure :
 *
 *      raw = bias + M · a_vrai          (raw en LSB, a_vrai en g)
 *
 *   - bias : décalage de zéro (ce que l'accel lit en apesanteur). Quelques
 *            dizaines de mg typiquement, augmenté par les contraintes de soudure.
 *   - M    : matrice 3x3. Diagonale = sensibilité de chaque axe (nominal 2048
 *            LSB/g à ±16 g, à quelques % près). Hors diagonale = couplage entre
 *            axes (axes du capteur pas parfaitement orthogonaux) + désalignement
 *            de la carte par rapport au châssis du drone.
 *
 * Un biais de 20 mg donne ~1,1° d'erreur d'horizon en mode angle → le drone
 * dérive. D'où la calibration : on estime bias et M, puis on inverse :
 *
 *      a_cal = T · (raw − bias)     avec T = g · M⁻¹   (sortie en m/s²)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * POURQUOI 6 FACES, ET LE CALCUL
 * ═══════════════════════════════════════════════════════════════════════════
 * Posé immobile, l'accel ne mesure que la gravité (1 g). Sur chaque face, la
 * gravité est alignée sur un axe, dans un sens : +X, −X, +Y, −Y, +Z, −Z.
 * On note m₊ᵢ et m₋ᵢ la mesure moyenne (vecteur 3D, LSB) sur les faces ±i.
 *
 * D'après le modèle :   m₊ᵢ = bias + Mᵢ·1g     et     m₋ᵢ = bias − Mᵢ·1g
 * (Mᵢ = colonne i de M). Donc :
 *
 *   - Somme  : m₊ᵢ + m₋ᵢ = 2·bias      → bias = moyenne des 6 mesures
 *              (la moyenne sur les 6 faces est l'estimation aux moindres carrés)
 *   - Différence : (m₊ᵢ − m₋ᵢ)/2 = Mᵢ·1g → colonne i de la matrice D
 *
 *   D = M exprimée en LSB/g, donc T = g · D⁻¹ convertit directement en m/s².
 *   Vérification : T · (m₊ᵢ − bias) = g · D⁻¹ · Dᵢ = g · eᵢ  ✓ (1 g sur l'axe i)
 *
 * Remarque : comme D est une matrice complète (pas seulement diagonale), T
 * absorbe aussi une petite rotation de la carte par rapport aux faces du
 * drone. Les faces doivent donc être les vraies faces du châssis (posé bien à
 * plat sur une table), pas des positions approximatives tenues à la main.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * ARCHITECTURE : ISR 8 kHz ↔ BOUCLE PRINCIPALE ~500 Hz
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   ISR DMA ICM (8 kHz)                       Boucle principale (~500 Hz)
 *   ───────────────────                       ───────────────────────────
 *   AccelCal_Feed()                           AccelCal_Process()
 *     accumule 800 échantillons                 lit la radio (trigger)
 *     (= 1 fenêtre de 100 ms) :                 si win_ready : copie `win`,
 *     somme + min/max accel et gyro             remet win_ready = false,
 *     → publie `win`, win_ready = true  ──────►  machine d'état (faces, capture)
 *                                               en fin : calcul + flash
 *   AccelCal_Apply()                 ◄──────  set_active() (IRQ coupées)
 *     a_cal = T·(raw − bias) avec `active`
 *
 * Pourquoi accumuler dans l'ISR : la boucle principale ne voit pas chaque
 * échantillon (8 kHz contre 500 Hz). Dans l'ISR, on prend les 8000 éch./s,
 * pour un coût de quelques additions et comparaisons.
 *
 * Règles de partage (ne pas casser) :
 *   - `win` n'est écrit par l'ISR QUE si win_ready == false, et lu par la
 *     boucle QUE si win_ready == true → jamais les deux en même temps.
 *     Si la boucle est en retard, l'ISR jette la fenêtre (sans conséquence).
 *   - `active` (calibration appliquée) est lu par l'ISR à chaque échantillon :
 *     on le modifie IRQ coupées (set_active), sinon l'ISR pourrait utiliser une
 *     matrice à moitié recopiée.
 *   - `acc` n'est touché que par l'ISR, sauf la remise à zéro dans feed_start()
 *     (faite IRQ coupées).
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * MACHINE D'ÉTAT
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   IDLE ──(SA+SD 2 s)──► WAIT_POSE ──(même face immobile 1 s)──► CAPTURE
 *                            ▲  ▲                                  │   │
 *                            │  └──────(mouvement / autre face)────┘   │
 *                            └────────(face mesurée 2 s, < 6/6)────────┘
 *                                                                      │
 *   IDLE ◄──(3 s d'affichage LED)── RESULT ◄──(6/6 : calcul + flash)───┘
 *
 *   Depuis WAIT_POSE ou CAPTURE : SA+SD 2 s ou timeout 5 min → IDLE (annulé).
 */

#include "accel_calib.h"
#include "config.h"
#include "remote_driver.h"
#include "led_driver.h"
#include "debug.h"
#include <math.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Calibration active (lue par l'ISR à 8 kHz, mise à jour IRQ coupées)
 * ═══════════════════════════════════════════════════════════════════════════ */
static accel_cal_t   active;                /* copie RAM utilisée par AccelCal_Apply */
static volatile bool active_valid = false;  /* false → échelle nominale, pas de correction */

/* ═══════════════════════════════════════════════════════════════════════════
 * Acquisition par fenêtres — côté ISR
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Accumulateur de la fenêtre en cours (propriété de l'ISR).
 * sum : 800 éch. × 32767 max = 26 M → tient dans un int32 sans risque. */
typedef struct {
    int32_t  sum[3];
    int16_t  amin[3], amax[3], gmin[3], gmax[3];
    uint16_t n;                 /* nb d'échantillons déjà accumulés */
} win_acc_t;

/* Résumé d'une fenêtre terminée, transmis à la boucle principale. */
typedef struct {
    float    mean[3];   /* moyenne accel sur la fenêtre (LSB) */
    int16_t  aptp;      /* crête-crête ("peak-to-peak") accel, max des 3 axes */
    int16_t  gptp;      /* crête-crête gyro, max des 3 axes */
} win_t;

/* Le crête-crête sert à détecter le mouvement. On ne regarde pas la valeur du
 * gyro mais sa VARIATION : le critère reste valable même si la calibration
 * gyro a échoué au boot et qu'il reste un biais gyro. */

static win_acc_t         acc;            /* fenêtre en cours d'accumulation */
static win_t             win;            /* dernière fenêtre publiée */
static volatile bool     feeding   = false;  /* true = calibration en cours, l'ISR accumule */
static volatile bool     win_ready = false;  /* "boîte aux lettres" ISR → boucle */
static volatile uint32_t win_count = 0;      /* nb total de fenêtres → fréquence IMU réelle */

/* Appelée par l'ISR DMA de l'ICM à chaque échantillon (8 kHz). Doit rester
 * très courte : pas de flottant dans le chemin courant, une seule division
 * par fenêtre (tous les 800 échantillons). */
void AccelCal_Feed(const int16_t a[3], const int16_t g[3])
{
    if (!feeding) return;      /* hors calibration : coût = un test */

    if (acc.n == 0U) {         /* début de fenêtre : min/max initialisés sur le 1er échantillon */
        for (uint8_t k = 0; k < 3U; k++) {
            acc.sum[k] = 0;
            acc.amin[k] = acc.amax[k] = a[k];
            acc.gmin[k] = acc.gmax[k] = g[k];
        }
    }
    for (uint8_t k = 0; k < 3U; k++) {
        acc.sum[k] += a[k];
        if (a[k] < acc.amin[k]) acc.amin[k] = a[k];
        if (a[k] > acc.amax[k]) acc.amax[k] = a[k];
        if (g[k] < acc.gmin[k]) acc.gmin[k] = g[k];
        if (g[k] > acc.gmax[k]) acc.gmax[k] = g[k];
    }

    if (++acc.n >= ACAL_WIN_SAMPLES) {
        /* Fenêtre complète. On ne publie que si la boucle a consommé la
         * précédente (win_ready == false) ; sinon on la jette pour ne pas
         * écrire dans `win` pendant que la boucle le lit. */
        if (!win_ready) {
            int16_t ap = 0, gp = 0;
            for (uint8_t k = 0; k < 3U; k++) {
                win.mean[k] = (float)acc.sum[k] / (float)ACAL_WIN_SAMPLES;
                if (acc.amax[k] - acc.amin[k] > ap) ap = (int16_t)(acc.amax[k] - acc.amin[k]);
                if (acc.gmax[k] - acc.gmin[k] > gp) gp = (int16_t)(acc.gmax[k] - acc.gmin[k]);
            }
            win.aptp  = ap;
            win.gptp  = gp;
            win_ready = true;   /* en dernier : `win` est complet quand la boucle le voit */
        }
        win_count++;
        acc.n = 0;
    }
}

/* Appelée par l'ISR DMA de l'ICM à chaque échantillon : raw (LSB) → m/s².
 * Coût : 3 soustractions + 9 multiplications-additions flottantes (FPU). */
void AccelCal_Apply(const int16_t raw[3], float out[3])
{
    if (!active_valid) {
        /* Pas de calibration : conversion nominale ±16 g (2048 LSB/g). */
        const float s = ACAL_GRAVITY / ACAL_LSB_PER_G;
        out[0] = (float)raw[0] * s;
        out[1] = (float)raw[1] * s;
        out[2] = (float)raw[2] * s;
        return;
    }
    /* a_cal = T · (raw − bias), T stockée ligne par ligne */
    const float vx = (float)raw[0] - active.bias[0];
    const float vy = (float)raw[1] - active.bias[1];
    const float vz = (float)raw[2] - active.bias[2];
    out[0] = active.T[0] * vx + active.T[1] * vy + active.T[2] * vz;
    out[1] = active.T[3] * vx + active.T[4] * vy + active.T[5] * vz;
    out[2] = active.T[6] * vx + active.T[7] * vy + active.T[8] * vz;
}

/* Remplace la calibration utilisée par l'ISR (NULL = plus de calibration).
 * IRQ coupées le temps de recopier 48 octets (quelques centaines de ns) : l'ISR
 * voit soit l'ancienne calibration complète, soit la nouvelle, jamais un mélange. */
static void set_active(const accel_cal_t *c)
{
    __disable_irq();
    if (c) { active = *c; active_valid = true; }
    else   { active_valid = false; }
    __enable_irq();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Console debug
 * printf avec flottants n'est pas dispo par défaut avec newlib-nano (il
 * faudrait -u _printf_float, +10 Ko) : on affiche des entiers mis à l'échelle
 * (mg, 0,01 %).
 * ═══════════════════════════════════════════════════════════════════════════ */
static void cal_print(const char *s) { print_to_console((char *)s, (uint16_t)strlen(s)); }

static void cal_print_i32(int32_t v)
{
    char buf[12], tmp[11];
    uint8_t n = 0, t = 0;
    uint32_t u = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
    if (v < 0) buf[n++] = '-';
    do { tmp[t++] = (char)('0' + (u % 10U)); u /= 10U; } while (u);  /* chiffres à l'envers */
    while (t) buf[n++] = tmp[--t];                                    /* remis à l'endroit */
    print_to_console(buf, n);
}

/* Index de face = 2·axe + (1 si sens négatif) : 0=X+ 1=X- 2=Y+ 3=Y- 4=Z+ 5=Z-.
 * "Z+" = l'accel lit +1 g sur Z. Selon le montage de la carte, ça correspond
 * au drone posé à l'endroit ou à l'envers. */
static const char *const pose_name[6] = { "X+", "X-", "Y+", "Y-", "Z+", "Z-" };

/* ═══════════════════════════════════════════════════════════════════════════
 * Machine d'état — côté boucle principale (voir schéma en tête de fichier)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef enum { ACAL_IDLE, ACAL_WAIT_POSE, ACAL_CAPTURE, ACAL_RESULT } acal_state_t;

static acal_state_t state = ACAL_IDLE;
static uint32_t     t_start;         /* get_ms() au démarrage (timeout, mesure fréquence) */
static uint32_t     t_result;        /* get_ms() à la fin (durée affichage LED résultat) */
static uint32_t     win_start;       /* win_count au démarrage (mesure fréquence) */
static bool         rate_printed;    /* fréquence IMU déjà affichée */
static bool         result_ok;       /* résultat du calcul, pour la LED en RESULT */
static float        meas[6][3];      /* moyenne mesurée par face (LSB), index = pose_name */
static uint8_t      done_mask;       /* bit p = face p mesurée ; 0x3F = les 6 faites */
static int8_t       cand_pose;       /* face candidate en cours d'observation (-1 = aucune) */
static uint8_t      settle_cnt;      /* nb de fenêtres consécutives immobiles sur cand_pose */
static uint8_t      cap_cnt;         /* nb de fenêtres déjà accumulées pendant la capture */
static float        cap_sum[3];      /* somme des moyennes de fenêtres pendant la capture */
static bool         announced;       /* message "face déjà faite" déjà affiché */

/* ── Déclencheur radio ─────────────────────────────────────────────────────
 * Renvoie true UNE SEULE FOIS quand SA + SD (+ throttle bas + lien RC OK) sont
 * tenus ACAL_TRIG_HOLD_MS d'affilée. Il faut ensuite relâcher (condition
 * fausse au moins une fois) pour pouvoir redéclencher : ça évite qu'en gardant
 * les switches levés on démarre, annule, redémarre… en boucle.
 *
 * trig_rearmed démarre à false : si SA + SD sont déjà levés au boot, ça ne
 * déclenche rien tant qu'ils n'ont pas été baissés une fois. */
static bool     trig_rearmed = false;
static bool     trig_holding = false;
static uint32_t trig_since;

static bool trigger_event(uint32_t now)
{
    bool on = rc_is_valid() && rc_failsafe.state != FAILSAFE_ACTIVE &&
              rc_ch[ACAL_TRIG_CH_SA] > ACAL_TRIG_HIGH &&
              rc_ch[ACAL_TRIG_CH_SD] > ACAL_TRIG_HIGH &&
              rc_axes.throttle < ACAL_TRIG_THROTTLE_MAX;

    if (!on) { trig_holding = false; trig_rearmed = true; return false; }
    if (!trig_holding) { trig_holding = true; trig_since = now; }   /* début de l'appui */
    if (trig_rearmed && (now - trig_since) >= ACAL_TRIG_HOLD_MS) {
        trig_rearmed = false;   /* consommé : il faudra relâcher */
        return true;
    }
    return false;
}

static void leds_off(void) { R_LED_Off(); B_LED_Off(); }

/* debug_imu et debug_rc sont coupés pendant la calibration puis remis dans leur
 * état d'avant.
 *   - debug_imu : le print se fait DANS l'ISR DMA de l'ICM et bloque ~0,5 ms.
 *     Pendant ce temps des DRDY sont perdus (cf. ICM42688_EXTI4_Callback) : moins
 *     d'échantillons par fenêtre et fréquence IMU mesurée fausse (< 8000 Hz).
 *   - debug_rc : imprimé depuis la boucle principale (5 Hz), il ne gêne pas la
 *     mesure, mais noierait les messages ACAL (faces, résultat) dans la console.
 * Les messages ACAL, eux, sont imprimés depuis la boucle principale. */
static bool dbg_imu_saved   = false;   /* valeur de debug_imu avant la calibration */
static bool dbg_rc_saved    = false;   /* valeur de debug_rc avant la calibration */
static bool dbg_stashed     = false;   /* true = valeurs sauvegardées, à restaurer */

/* Démarre l'accumulation côté ISR, en repartant d'une fenêtre vide.
 * IRQ coupées : on modifie `acc`, qui appartient normalement à l'ISR. */
static void feed_start(void)
{
    dbg_imu_saved = debug_imu;
    dbg_rc_saved  = debug_rc;
    dbg_stashed   = true;
    debug_imu     = false;
    debug_rc      = false;

    __disable_irq();
    acc.n     = 0;
    win_ready = false;
    feeding   = true;
    __enable_irq();
}

static void feed_stop(void) { feeding = false; win_ready = false; }

/* Remet debug_imu et debug_rc comme avant la calibration. Appelé au retour en
 * ACAL_IDLE (annulation, ou fin des 3 s d'affichage du résultat) et pas dans
 * feed_stop : sinon les prints se mélangeraient aux messages du résultat. */
static void dbg_restore(void)
{
    if (dbg_stashed) {
        debug_imu   = dbg_imu_saved;
        debug_rc    = dbg_rc_saved;
        dbg_stashed = false;
    }
}

/* Identifie la face à partir d'une moyenne accel (LSB). Renvoie -1 si :
 *   - |a| n'est pas proche de 1 g (±10 %) : le drone est accéléré, pas posé ;
 *   - aucun axe ne porte au moins 95 % de |a| : le drone est penché (> ~18°),
 *     pas posé sur une face.
 * Sinon renvoie 2·axe + (1 si négatif), voir pose_name. */
static int8_t pose_of(const float m[3])
{
    float n = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
    if (fabsf(n - ACAL_LSB_PER_G) > ACAL_POSE_NORM_TOL * ACAL_LSB_PER_G) return -1;
    uint8_t k = 0;
    for (uint8_t i = 1; i < 3U; i++) if (fabsf(m[i]) > fabsf(m[k])) k = i;   /* axe dominant */
    if (fabsf(m[k]) < ACAL_POSE_ALIGN_MIN * n) return -1;
    return (int8_t)(2U * k + (m[k] < 0.0f ? 1U : 0U));
}

/* Inverse d'une matrice 3x3 (stockée ligne par ligne) par la méthode des
 * cofacteurs : A⁻¹ = adj(A) / det(A), où adj(A) est la transposée de la
 * matrice des cofacteurs.
 * c0, c1, c2 = cofacteurs de la 1re ligne, réutilisés pour le déterminant
 * (développement selon la 1re ligne) et pour la 1re colonne de l'inverse.
 * Retourne false si la matrice est quasi singulière : deux faces mesurées
 * quasi identiques, ce qui n'arrive pas si pose_of() a bien fait son travail.
 * Seuil : D vaut ~2048 sur sa diagonale, donc det ≈ 2048³ ; on refuse en
 * dessous de 1e-6 fois ça. */
static bool inv3(const float a[9], float r[9])
{
    float c0 = a[4] * a[8] - a[5] * a[7];
    float c1 = a[5] * a[6] - a[3] * a[8];
    float c2 = a[3] * a[7] - a[4] * a[6];
    float det = a[0] * c0 + a[1] * c1 + a[2] * c2;
    if (fabsf(det) < 1e-6f * ACAL_LSB_PER_G * ACAL_LSB_PER_G * ACAL_LSB_PER_G) return false;
    float id = 1.0f / det;
    r[0] = c0 * id; r[1] = (a[2] * a[7] - a[1] * a[8]) * id; r[2] = (a[1] * a[5] - a[2] * a[4]) * id;
    r[3] = c1 * id; r[4] = (a[0] * a[8] - a[2] * a[6]) * id; r[5] = (a[2] * a[3] - a[0] * a[5]) * id;
    r[6] = c2 * id; r[7] = (a[1] * a[6] - a[0] * a[7]) * id; r[8] = (a[0] * a[4] - a[1] * a[3]) * id;
    return true;
}

/* Calcule bias et T à partir des 6 faces (voir les maths en tête de fichier),
 * vérifie la vraisemblance, puis applique et sauvegarde.
 * En cas d'échec, RIEN n'est modifié : l'ancienne calibration reste active et
 * reste en flash. */
static bool solve_and_save(void)
{
    accel_cal_t c;
    float D[9];   /* ligne par ligne : D[3·r + i] = composante r de la colonne i */

    /* bias = moyenne des 6 faces ; colonne i de D = (m₊ᵢ − m₋ᵢ)/2.
     * meas[2i] = face +i, meas[2i+1] = face −i (voir pose_name). */
    for (uint8_t r = 0; r < 3U; r++) {
        float s = 0.0f;
        for (uint8_t p = 0; p < 6U; p++) s += meas[p][r];
        c.bias[r] = s / 6.0f;
        for (uint8_t i = 0; i < 3U; i++)
            D[3U * r + i] = 0.5f * (meas[2U * i][r] - meas[2U * i + 1U][r]);
    }

    /* T = g · D⁻¹ */
    float Di[9];
    if (!inv3(D, Di)) { cal_print("\n\rACAL : ECHEC (matrice singuliere)"); return false; }
    for (uint8_t k = 0; k < 9U; k++) c.T[k] = ACAL_GRAVITY * Di[k];

    /* ── Contrôles de vraisemblance ──────────────────────────────────────
     * Ils attrapent une capture ratée : drone pas à plat, face mal posée,
     * choc pendant la mesure, capteur défectueux. Seuils dans accel_calib.h.
     * s_nom = valeur de T si le capteur était parfait (diagonale = g/2048). */
    const float s_nom = ACAL_GRAVITY / ACAL_LSB_PER_G;
    bool ok = true;

    /* 1) Biais, affiché en mg */
    cal_print("\n\rACAL : biais (mg) X="); cal_print_i32(lroundf(c.bias[0] * 1000.0f / ACAL_LSB_PER_G));
    cal_print(" Y=");                      cal_print_i32(lroundf(c.bias[1] * 1000.0f / ACAL_LSB_PER_G));
    cal_print(" Z=");                      cal_print_i32(lroundf(c.bias[2] * 1000.0f / ACAL_LSB_PER_G));
    for (uint8_t k = 0; k < 3U; k++)
        if (fabsf(c.bias[k]) > ACAL_BIAS_MAX_LSB) ok = false;

    /* 2) Matrice T rapportée au nominal, en 0,01 % :
     *    diagonale     → erreur d'échelle de l'axe (ex. 120 = l'axe lit 1,2 % de trop)
     *    hors diagonale → couplage entre axes (ex. 50 = 0,5 %, soit ~0,3° de désalignement) */
    cal_print("\n\rACAL : matrice T / nominal, en 0,01 % (diag = erreur d'echelle)");
    for (uint8_t r = 0; r < 3U; r++) {
        cal_print("\n\r   ");
        for (uint8_t i = 0; i < 3U; i++) {
            float rel = c.T[3U * r + i] / s_nom;
            if (r == i) { rel -= 1.0f; if (fabsf(rel) > ACAL_SCALE_TOL) ok = false; }
            else if (fabsf(rel) > ACAL_CROSS_MAX) ok = false;
            cal_print(" "); cal_print_i32(lroundf(rel * 10000.0f));
        }
    }

    /* 3) Résidu : on réapplique la calibration aux 6 mesures. Chaque face doit
     *    donner exactement 1 g en norme. Un résidu élevé = les 6 mesures ne
     *    sont pas cohérentes entre elles (une face mal posée, par exemple). */
    float res_max = 0.0f;
    for (uint8_t p = 0; p < 6U; p++) {
        float v[3], o[3];
        for (uint8_t k = 0; k < 3U; k++) v[k] = meas[p][k] - c.bias[k];
        for (uint8_t r = 0; r < 3U; r++) o[r] = c.T[3U*r] * v[0] + c.T[3U*r+1U] * v[1] + c.T[3U*r+2U] * v[2];
        float e = fabsf(sqrtf(o[0]*o[0] + o[1]*o[1] + o[2]*o[2]) / ACAL_GRAVITY - 1.0f);
        if (e > res_max) res_max = e;
    }
    cal_print("\n\rACAL : residu max |a|-1g (0,01 %) = "); cal_print_i32(lroundf(res_max * 10000.0f));
    if (res_max > ACAL_RESIDUAL_MAX) ok = false;

    if (!ok) {
        cal_print("\n\rACAL : ECHEC (valeurs hors tolerances), calibration precedente conservee");
        return false;
    }

    /* Tout est bon : appliqué immédiatement, puis sauvegardé. Si la flash
     * échoue, la calibration reste valable jusqu'au prochain reboot. */
    set_active(&c);
    g_config.accel  = c;
    g_config.flags |= CONFIG_FLAG_ACCEL_CAL;
    if (Config_Save()) cal_print("\n\rACAL : OK, sauvegardee en flash");
    else               cal_print("\n\rACAL : OK mais ECHEC sauvegarde flash (appliquee jusqu'au reboot)");
    return true;
}

static void abort_cal(const char *why)
{
    feed_stop();
    cal_print("\n\rACAL : annulee ("); cal_print(why); cal_print(")");
    leds_off();
    dbg_restore();
    state = ACAL_IDLE;   /* rien n'a été modifié : l'ancienne calibration reste active */
}

/* Traite une fenêtre de 100 ms en état WAIT_POSE ou CAPTURE. */
static void handle_window(const win_t *w)
{
    /* Immobile = l'accel ET le gyro ont peu varié pendant 100 ms.
     * Si ce n'est pas le cas, pose = -1, comme une face non reconnue. */
    bool   still = (w->aptp <= ACAL_STILL_ACC_PTP_LSB) && (w->gptp <= ACAL_STILL_GYR_PTP_LSB);
    int8_t pose  = still ? pose_of(w->mean) : -1;

    if (state == ACAL_WAIT_POSE) {
        /* On attend ACAL_SETTLE_WIN fenêtres consécutives immobiles sur la même
         * face. Ça laisse le drone se stabiliser (vibrations après l'avoir posé)
         * avant de mesurer. */
        if (pose < 0) { cand_pose = -1; settle_cnt = 0; R_LED_Off(); return; }
        if (pose != cand_pose) { cand_pose = pose; settle_cnt = 0; announced = false; }

        if (done_mask & (1U << pose)) {
            /* Face déjà mesurée : LED rouge tant qu'on ne change pas de face. */
            R_LED_On();
            if (!announced) { cal_print("\n\rACAL : face "); cal_print(pose_name[pose]); cal_print(" deja faite"); }
            announced = true;
            return;
        }
        R_LED_Off();
        if (++settle_cnt >= ACAL_SETTLE_WIN) {
            cap_sum[0] = cap_sum[1] = cap_sum[2] = 0.0f;
            cap_cnt = 0;
            state   = ACAL_CAPTURE;
            B_LED_On();
            cal_print("\n\rACAL : capture face "); cal_print(pose_name[pose]); cal_print("...");
        }
        return;
    }

    /* ── ACAL_CAPTURE ──
     * Chaque fenêtre doit rester immobile ET sur la même face. Sinon la capture
     * de cette face est perdue et on retourne attendre (rien n'est enregistré). */
    if (pose != cand_pose) {
        cal_print(" mouvement, a refaire");
        state = ACAL_WAIT_POSE; cand_pose = -1; settle_cnt = 0;
        return;
    }
    /* Moyenne des moyennes de fenêtres : elles contiennent toutes 800
     * échantillons, donc c'est la moyenne exacte des 16 000 échantillons. */
    for (uint8_t k = 0; k < 3U; k++) cap_sum[k] += w->mean[k];
    if (++cap_cnt < ACAL_CAPTURE_WIN) return;

    for (uint8_t k = 0; k < 3U; k++) meas[pose][k] = cap_sum[k] / (float)cap_cnt;
    done_mask |= (uint8_t)(1U << pose);

    uint8_t n = 0;
    for (uint8_t p = 0; p < 6U; p++) if (done_mask & (1U << p)) n++;
    cal_print(" OK ("); cal_print_i32(n); cal_print("/6)");

    if (done_mask == 0x3FU) {
        /* 6 faces : on arrête l'acquisition et on calcule.
         * solve_and_save() peut bloquer ~1-2 s si le secteur flash doit être
         * effacé (au plus une fois toutes les 512 sauvegardes). */
        feed_stop();
        leds_off();
        result_ok = solve_and_save();
        t_result  = get_ms();
        state     = ACAL_RESULT;
    } else {
        /* cand_pose reste sur la face qu'on vient de mesurer : comme elle est
         * marquée faite, la LED rouge s'allume jusqu'à ce qu'on change de face. */
        state = ACAL_WAIT_POSE; settle_cnt = 0;
    }
}

void AccelCal_Process(void)
{
    uint32_t now  = get_ms();
    bool     trig = trigger_event(now);   /* appelé à chaque tour, pour suivre l'appui */

    switch (state) {

    case ACAL_IDLE:
        if (trig) {
            memset(meas, 0, sizeof(meas));
            done_mask = 0; cand_pose = -1; settle_cnt = 0;
            t_start = now; win_start = win_count; rate_printed = false;
            feed_start();
            state = ACAL_WAIT_POSE;
            cal_print("\n\r\n\rACAL : calibration accel 6 faces demarree. Poser le drone sur chaque face,"
                      "\n\r       dans n'importe quel ordre, sans le toucher (SA+SD 2 s pour annuler).");
        }
        break;

    case ACAL_WAIT_POSE:
    case ACAL_CAPTURE:
        if (trig)                                  { abort_cal("SA+SD");   break; }
        if ((now - t_start) > ACAL_TIMEOUT_MS)     { abort_cal("timeout"); break; }

        if (!rate_printed && (now - t_start) >= 2000U) {
            /* Diagnostic gratuit : fréquence réelle du DRDY de l'ICM, mesurée
             * en comptant les fenêtres sur 2 s. Attendu : 8000 Hz. Une valeur
             * plus basse = interruptions perdues (cf. INT_CONFIG1 dans le driver ICM). */
            uint32_t hz = ((win_count - win_start) * ACAL_WIN_SAMPLES * 1000U) / (now - t_start);
            cal_print("\n\rACAL : frequence IMU mesuree = "); cal_print_i32((int32_t)hz); cal_print(" Hz");
            rate_printed = true;
        }

        if (state == ACAL_WAIT_POSE) {
            if (((now / 250U) & 1U) != 0U) B_LED_On(); else B_LED_Off();   /* clignote 2 Hz */
        }

        if (win_ready) {
            win_t w = win;          /* copie locale, PUIS on libère la boîte aux lettres */
            win_ready = false;
            handle_window(&w);
        }
        break;

    case ACAL_RESULT:
        /* 3 s d'affichage : bleu fixe = OK, rouge clignotant 4 Hz = échec */
        if (result_ok) B_LED_On();
        else { if (((now / 125U) & 1U) != 0U) R_LED_On(); else R_LED_Off(); }
        if ((now - t_result) >= 3000U) { leds_off(); dbg_restore(); state = ACAL_IDLE; }
        break;
    }
}

/* Au boot, après Config_Load() : applique la calibration sauvegardée si elle existe. */
void AccelCal_Init(void)
{
    if (g_config.flags & CONFIG_FLAG_ACCEL_CAL) {
        set_active(&g_config.accel);
        cal_print("\n\rACAL : calibration accel chargee depuis la flash");
    } else {
        set_active(NULL);
        cal_print("\n\rACAL : /!\\ AUCUNE calibration accel (throttle bas + SA+SD 2 s pour calibrer)");
    }
}

bool AccelCal_IsValid(void)    { return active_valid; }
bool AccelCal_InProgress(void) { return state == ACAL_WAIT_POSE || state == ACAL_CAPTURE; }
