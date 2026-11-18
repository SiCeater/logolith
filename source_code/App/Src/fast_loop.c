/**
 * fast_loop.c
 * Boucle rapide à ICM42688_ODR_HZ (1 kHz) + mesure de son budget CPU.
 *
 * ── Où elle tourne ────────────────────────────────────────────────────────
 * FastLoop_Run() est appelée à la fin de l'IRQ DMA2_Stream0 (fin de lecture SPI
 * de l'ICM), donc une fois par échantillon IMU, juste après la conversion et la
 * calibration des données (icm42688.gx…). Priorité NVIC 1, la plus haute du
 * firmware : rien ne la préempte sauf EXTI4 au même niveau, qui attend.
 * C'est ici que viendront filtres gyro, PID rate et mixeur. La sortie ESC est
 * déclenchée en FIN d'itération (ESC_Update) : l'impulsion OneShot125 part
 * quelques µs après le calcul, avec un retard fixe (voir esc_driver.c).
 *
 * ── Mesure avec le compteur de cycles DWT ────────────────────────────────
 * DWT->CYCCNT compte les cycles CPU (168 MHz → 5,95 ns), 32 bits, reboucle
 * toutes les ~25,6 s ; les différences non signées restent justes tant qu'un
 * intervalle mesuré fait moins de 25 s.
 *   - période  = écart entre deux entrées dans l'IRQ → dt RÉEL de la boucle
 *                (l'ODR de l'ICM n'est pas exactement 1000 Hz : ~996 Hz
 *                attendus d'après les 7967 Hz mesurés à 8 kHz, ±1,5 %
 *                garanti, et ça dérive avec la température) ;
 *   - exécution = de l'entrée dans l'IRQ (t_entry) à la fin de FastLoop_Run :
 *                parsing SPI, conversion, calibration accel, instantané debug
 *                ET boucle rapide. Ne compte pas l'IRQ EXTI4 (relance du DMA,
 *                quelques µs) ni les cycles d'entrée/sortie d'interruption.
 * Les stats sont cumulées sur FL_STATS_N échantillons (~1 s) puis publiées
 * dans un instantané, imprimé hors ISR par FastLoop_Debug_Process()
 * (même poignée de main que le debug IMU : l'ISR n'écrit que si le précédent
 * a été lu).
 */
#include "fast_loop.h"
#include "main.h"      /* CMSIS : DWT, CoreDebug */
#include "global.h"
#include "debug.h"
#include "icm42688_driver.h"   /* ICM42688_ODR_HZ */
#include "esc_driver.h"        /* ESC_Update */
#include <string.h>

#define CPU_HZ          168000000UL
#define FL_STATS_N      ICM42688_ODR_HZ   /* ~1 s de stats */

/* dt réel de la dernière itération (s), à utiliser par filtres / PID */
volatile float fast_loop_dt = 1.0f / (float)ICM42688_ODR_HZ;

/* ── Stats en cours d'accumulation (ISR uniquement) ── */
static uint32_t t_prev;
static bool     have_prev = false;
static uint32_t n;
static uint32_t per_min, per_max;
static uint64_t per_sum;
static uint32_t exe_max;
static uint64_t exe_sum;

/* ── Instantané publié pour l'affichage ── */
typedef struct {
    uint32_t n;
    uint32_t per_min, per_max, exe_max;
    uint64_t per_sum, exe_sum;
} fl_stats_t;
static fl_stats_t    snap;
static volatile bool snap_ready = false;

void FastLoop_Init(void)
{
    /* Active le bloc de trace (TRCENA) puis le compteur de cycles. Sur F4 il
     * n'y a pas de registre de verrou (LAR) à déverrouiller. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t FastLoop_Cycles(void)
{
    return DWT->CYCCNT;
}

void FastLoop_Run(uint32_t t_entry)
{
    /* ── dt réel ── */
    uint32_t per = 0;
    if (have_prev) {
        per = t_entry - t_prev;
        fast_loop_dt = (float)per * (1.0f / (float)CPU_HZ);
    }
    t_prev    = t_entry;
    have_prev = true;

    /* ──────────────────────────────────────────────────────────────────
     * ICI, plus tard : filtres gyro → PID rate → mixeur
     * (commande d'arrêt si !Security_IsArmed()).
     * ────────────────────────────────────────────────────────────────── */

    /* Émet l'impulsion OneShot125 avec les dernières commandes moteur. Toujours
     * appelée : sans elle, aucun signal ne sort (TIM8 en one-pulse). */
    ESC_Update();

    /* ── Stats (à garder en dernier : mesure tout ce qui précède) ── */
    uint32_t exe = DWT->CYCCNT - t_entry;
    if (per == 0U) return;               /* 1re itération : pas encore de période */

    if (n == 0U) { per_min = per_max = per; per_sum = 0; exe_max = 0; exe_sum = 0; }
    if (per < per_min) per_min = per;
    if (per > per_max) per_max = per;
    if (exe > exe_max) exe_max = exe;
    per_sum += per;
    exe_sum += exe;

    if (++n >= FL_STATS_N) {
        if (!snap_ready) {               /* sinon : fenêtre jetée, l'affichage n'a pas suivi */
            snap.n = n;
            snap.per_min = per_min; snap.per_max = per_max; snap.exe_max = exe_max;
            snap.per_sum = per_sum; snap.exe_sum = exe_sum;
            snap_ready = true;           /* en dernier */
        }
        n = 0;
    }
}

/* ── Affichage ─────────────────────────────────────────────────────────── */

/* Écrit v en décimal dans out, renvoie le nombre de caractères. */
static uint8_t u32_to_dec(uint32_t v, char *out)
{
    char tmp[10];
    uint8_t k = 0;
    do { tmp[k++] = (char)('0' + v % 10U); v /= 10U; } while (v != 0U);
    for (uint8_t i = 0; i < k; i++) out[i] = tmp[k - 1U - i];
    return k;
}

/* Cycles → µs avec 2 décimales ("12.34") */
static uint8_t cyc_to_us(uint32_t cyc, char *out)
{
    uint32_t us100 = (uint32_t)(((uint64_t)cyc * 100U + CPU_HZ / 2000000U) / (CPU_HZ / 1000000U));
    uint8_t k = u32_to_dec(us100 / 100U, out);
    out[k++] = '.';
    out[k++] = (char)('0' + (us100 / 10U) % 10U);
    out[k++] = (char)('0' + us100 % 10U);
    return k;
}

static uint8_t put(char *out, const char *s)
{
    uint8_t k = (uint8_t)strlen(s);
    memcpy(out, s, k);
    return k;
}

/* Format : "LOOP 996 Hz | per 1004.10 us (1003.80-1004.40) | exec moy 6.50 max 7.50 us | CPU 0.6 %" */
void FastLoop_Debug_Process(void)
{
    if (!snap_ready) return;
    fl_stats_t s = snap;
    snap_ready = false;

    uint32_t per_avg = (uint32_t)(s.per_sum / s.n);
    uint32_t exe_avg = (uint32_t)(s.exe_sum / s.n);
    uint32_t hz      = (uint32_t)(((uint64_t)CPU_HZ * s.n + s.per_sum / 2U) / s.per_sum);
    uint32_t cpu_x10 = (uint32_t)((s.exe_sum * 1000U + s.per_sum / 2U) / s.per_sum);   /* ‰ */

    char line[128];
    uint8_t p = 0;
    p += put(&line[p], "LOOP ");        p += u32_to_dec(hz, &line[p]);
    p += put(&line[p], " Hz | per ");       p += cyc_to_us(per_avg, &line[p]);
    p += put(&line[p], " us (");            p += cyc_to_us(s.per_min, &line[p]);
    line[p++] = '-';                        p += cyc_to_us(s.per_max, &line[p]);
    p += put(&line[p], ") | exec moy ");    p += cyc_to_us(exe_avg, &line[p]);
    p += put(&line[p], " max ");            p += cyc_to_us(s.exe_max, &line[p]);
    p += put(&line[p], " us | CPU ");       p += u32_to_dec(cpu_x10 / 10U, &line[p]);
    line[p++] = '.';                        line[p++] = (char)('0' + cpu_x10 % 10U);
    p += put(&line[p], " %\r\n");   /* fin de ligne en FIN, comme les prints IMU et RC */
    print_to_console(line, p);
}
