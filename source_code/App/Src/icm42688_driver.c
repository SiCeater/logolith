/**
 * icm42688_driver.c
 * Driver LL ICM-42688-P — SPI1 + DMA2 — STM32F405
 * Gyro et accel à ICM42688_ODR_HZ (1 kHz), mode Low-Noise, AAF actif
 * 
 * Architecture multi-banques (BANK 0/1/2/4) — accès via REG_BANK_SEL
 * Config optimisée pour drone cinelifter F380 (stabilité prioritaire)
 */
 
#include "icm42688_driver.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_utils.h"
#include "accel_calib.h"
#include "fast_loop.h"
 
/* ═══════════════════════════════════════════════════════════════════════════
 * INSTANCE PUBLIQUE et BUFFERS DMA
 * ═══════════════════════════════════════════════════════════════════════════ */
icm42688_data_t icm42688 = {0};

static volatile bool icm_ready = false;   /* true après init complète, prêt pour la fast loop */
/* true entre le lancement d'une lecture DMA (CS bas) et son callback de fin (CS haut).
 * Empêche de relancer une lecture alors que la précédente n'est pas terminée :
 * sans CS remonté entre les deux, l'ICM verrait UNE seule transaction continue et
 * renverrait les registres suivants (auto-incrément) → données fausses. */
static volatile bool dma_busy = false;
/* Nombre de lectures DMA perdues puis relancées (voir ICM42688_EXTI4_Callback).
 * Doit rester à 0 : à surveiller au debugger (Live Expressions). */
volatile uint32_t icm42688_dma_lost = 0;

/* Instantané pour le debug IMU : écrit par l'ISR DMA, lu par la boucle
 * principale (ICM42688_Debug_Process). dbg_snap_ready sert de poignée de main. */
typedef struct {
    int16_t  raw_gx, raw_gy, raw_gz;
    float    ax, ay, az;
    uint32_t lost;
} icm_dbg_snap_t;
static icm_dbg_snap_t dbg_snap;
static volatile bool  dbg_snap_ready = false;
 
/*
 * IMPORTANT : buffers DMA DOIVENT être en SRAM (0x20000000+), PAS en CCM RAM.
 * Le DMA2 sur F405 n'a pas accès au CCM (0x10000000).
 * Variables statiques globales → placées en SRAM par défaut (linker script).
 */
static uint8_t dma_tx_buf[ICM42688_BURST_LEN];
static uint8_t dma_rx_buf[ICM42688_BURST_LEN];
 
/* Macros CS */
#define ICM42688_CS_LOW()   LL_GPIO_ResetOutputPin(ICM42688_CS_PORT, ICM42688_CS_PIN)
#define ICM42688_CS_HIGH()  LL_GPIO_SetOutputPin(ICM42688_CS_PORT, ICM42688_CS_PIN)

static float accel_norm_ref_lsb = 2048.0f;   /* norme accel au repos (LSB, ±16g) */

float ICM42688_GetAccelNormRefLsb(void) { return accel_norm_ref_lsb; }

static int16_t cal_clamp12(int32_t v)
{
    if (v >  2047) return  2047;
    if (v < -2048) return -2048;
    return (int16_t)v;
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 * FONCTIONS SPI POLLING — Init uniquement (bloquantes)
 * ═══════════════════════════════════════════════════════════════════════════ */
 
/**
 * @brief  Sélectionne la banque de registres active.
 * @param  bank : 0, 1, 2 ou 4 (autres valeurs invalides sur ICM-42688-P)
 * @note   REG_BANK_SEL existe dans toutes les banques à la même adresse (0x76).
 *         Pas besoin de read-modify-write, écriture directe suffit.
 */
static void icm42688_select_bank(uint8_t bank)
{

    ICM42688_CS_LOW();

    /* Adresse REG_BANK_SEL (0x76) — mode write */
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, ICM42688_REG_REG_BANK_SEL & ~ICM42688_READ_FLAG);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    (void)LL_SPI_ReceiveData8(SPI1);  /* flush */

    while(LL_SPI_IsActiveFlag_BSY(SPI1));

    /* Valeur banque (bits [2:0]) */
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, bank & 0x07U);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    (void)LL_SPI_ReceiveData8(SPI1);  /* flush */
    
    while (LL_SPI_IsActiveFlag_BSY(SPI1));

    ICM42688_CS_HIGH();
}
 
/**
 * @brief  Écrit un octet dans un registre ICM-42688-P (SPI polling, bloquant).
 * @param  bank : banque cible (0/1/2/4)
 * @param  reg  : adresse registre dans la banque
 * @param  val  : valeur à écrire
 * @note   Change automatiquement de banque puis écrit le registre.
 */
static void spi_write_reg(uint8_t bank, uint8_t reg, uint8_t val)
{

    icm42688_select_bank(bank);

    ICM42688_CS_LOW();
    
    /* Adresse registre (bit7=0 → write) */
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, reg & ~ICM42688_READ_FLAG);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    (void)LL_SPI_ReceiveData8(SPI1);

    while(LL_SPI_IsActiveFlag_BSY(SPI1));

    /* Donnée */
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, val);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    (void)LL_SPI_ReceiveData8(SPI1);
    
    while (LL_SPI_IsActiveFlag_BSY(SPI1));

    
    ICM42688_CS_HIGH();
}
 
/**
 * @brief  Lit un octet depuis un registre ICM-42688-P (SPI polling, bloquant).
 * @param  bank : banque cible
 * @param  reg  : adresse registre dans la banque
 * @retval Valeur du registre
 */
static uint8_t spi_read_reg(uint8_t bank, uint8_t reg)
{
    uint8_t val;
    
    icm42688_select_bank(bank);

    ICM42688_CS_LOW();
    
    /* Adresse registre (bit7=1 → read) */
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, reg | ICM42688_READ_FLAG);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    (void)LL_SPI_ReceiveData8(SPI1);  /* flush premier octet */

    while(LL_SPI_IsActiveFlag_BSY(SPI1));

    /* Dummy TX pour générer clocks */
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, 0x00U);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    val = LL_SPI_ReceiveData8(SPI1);  /* récupère donnée */
    
    while (LL_SPI_IsActiveFlag_BSY(SPI1));

    ICM42688_CS_HIGH();

    return val;
}

/**
 * @brief  Lecture burst (polling) de n registres consécutifs en BANK 0.
 * @note   Une seule transaction CS : l'ICM garantit que les octets d'un burst
 *         viennent du même échantillon. Lire registre par registre (spi_read_reg)
 *         peut mélanger le MSB d'un échantillon et le LSB du suivant (saut de ±256 LSB).
 *         Init / calibration uniquement (avant activation du DMA).
 */
static void spi_read_burst_b0(uint8_t reg, uint8_t *buf, uint8_t n)
{
    icm42688_select_bank(ICM42688_BANK_SEL_0);

    ICM42688_CS_LOW();
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, reg | ICM42688_READ_FLAG);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    (void)LL_SPI_ReceiveData8(SPI1);

    for (uint8_t i = 0; i < n; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(SPI1));
        LL_SPI_TransmitData8(SPI1, 0x00U);
        while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
        buf[i] = LL_SPI_ReceiveData8(SPI1);
    }
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    ICM42688_CS_HIGH();
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SPs SELF TEST ICM-42688-P
 * ═══════════════════════════════════════════════════════════════════════════ */

static void st_print_i32(int32_t v)
{
    char buf[12], tmp[11];
    uint8_t n = 0, t = 0;
    uint32_t u = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
    if (v < 0) buf[n++] = '-';
    do { tmp[t++] = (char)('0' + (u % 10U)); u /= 10U; } while (u);
    while (t) buf[n++] = tmp[--t];
    print_to_console(buf, n);
}

/* Moyenne de N échantillons bruts (big-endian, lecture burst cohérente). */
static void st_average(int32_t g[3], int32_t a[3])
{
    int32_t sg[3] = {0, 0, 0}, sa[3] = {0, 0, 0};
    uint8_t b[12];

    for (uint32_t n = 0; n < ICM42688_ST_NSAMPLES; n++) {
        spi_read_burst_b0(ICM42688_REG_ACCEL_DATA_X1, b, 12U);   /* ACCEL_DATA_X1..GYRO_DATA_Z0 */
        for (uint8_t k = 0; k < 3U; k++) {
            sa[k] += (int16_t)((b[2U*k] << 8) | b[2U*k + 1U]);
            sg[k] += (int16_t)((b[6U + 2U*k] << 8) | b[7U + 2U*k]);
        }
        LL_mDelay(1);   /* ODR 1 kHz */
    }
    for (uint8_t k = 0; k < 3U; k++) {
        g[k] = sg[k] / (int32_t)ICM42688_ST_NSAMPLES;
        a[k] = sa[k] / (int32_t)ICM42688_ST_NSAMPLES;
    }
}

static void ICM42688_SelfTest(void)
{
    int32_t g0[3], a0[3], g1[3], a1[3], g2[3], a2[3];
    uint8_t gyro_ok = 1, accel_ok = 1;

    if (debug_init) ST_PRINT("\n\rICM-42688-P : SELF-TEST (carte immobile)...");

    /* ±250 dps (131 LSB/dps) / ±4 g (8192 LSB/g), ODR 1 kHz */
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_CONFIG0,  0x66U); /* FS_SEL=3 | ODR 1k */
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_ACCEL_CONFIG0, 0x46U); /* FS_SEL=2 | ODR 1k */
    LL_mDelay(100);

    /* 1) Référence sans self-test */
    st_average(g0, a0);

    /* 2) Self-test gyro 3 axes */
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_SELF_TEST_CONFIG, ICM42688_ST_EN_GXYZ);
    LL_mDelay(200);
    st_average(g1, a1);

    /* 3) Self-test accel 3 axes */
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_SELF_TEST_CONFIG,
                  ICM42688_ST_ACCEL_POWER | ICM42688_ST_EN_AXYZ);
    LL_mDelay(200);
    st_average(g2, a2);

    /* Retour à l'état normal (ACCEL_ST_POWER repassé à 0 comme exigé) */
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_SELF_TEST_CONFIG, 0x00U);

    static const char axis[3] = {'X', 'Y', 'Z'};
    for (uint8_t k = 0; k < 3U; k++) {
        int32_t dg = g1[k] - g0[k]; if (dg < 0) dg = -dg;
        int32_t da = a2[k] - a0[k]; if (da < 0) da = -da;
        int32_t resp_dps  = dg / 131;                /* ±250 dps : 131 LSB/(°/s)  */
        int32_t resp_mg   = (da * 1000) / 8192;      /* ±4 g     : 8192 LSB/g     */
        int32_t bias_cdps = (g0[k] * 100) / 131;     /* biais au repos, centi-°/s */

        if (resp_dps < ICM42688_ST_GYRO_MIN_DPS) gyro_ok = 0;
        if (resp_mg < ICM42688_ST_ACCEL_MIN_MG || resp_mg > ICM42688_ST_ACCEL_MAX_MG) accel_ok = 0;

        if (debug_init) {
            ST_PRINT("\n\r  ST "); print_to_console((char *)&axis[k], 1);
            ST_PRINT(" : gyro resp="); st_print_i32(resp_dps);
            ST_PRINT(" dps | accel resp="); st_print_i32(resp_mg);
            ST_PRINT(" mg | bias gyro(+/-250)="); st_print_i32(bias_cdps);
            ST_PRINT(" cdps");
        }
    }

    if (debug_init) {
        if (gyro_ok)  ST_PRINT("\n\rICM-42688-P : SELF-TEST GYRO PASS");
        else          ST_PRINT("\n\rICM-42688-P : SELF-TEST GYRO FAIL");
        if (accel_ok) ST_PRINT("\n\rICM-42688-P : SELF-TEST ACCEL PASS");
        else          ST_PRINT("\n\rICM-42688-P : SELF-TEST ACCEL FAIL");
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CALIBRATION GYRO ICM-42688-P
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Écrit les offsets gyro (LSB = 1/32 dps, 12 bits signés). Préserve ACCEL_X[11:8]. */
static void icm42688_set_gyro_offset(int16_t ox, int16_t oy, int16_t oz)
{
    uint8_t r4 = spi_read_reg(ICM_B4, ICM_OFFUSER4);
    spi_write_reg(ICM_B4, ICM_OFFUSER0, (uint8_t)(ox & 0xFF));
    spi_write_reg(ICM_B4, ICM_OFFUSER1, (uint8_t)((((oy >> 8) & 0x0F) << 4) | ((ox >> 8) & 0x0F)));
    spi_write_reg(ICM_B4, ICM_OFFUSER2, (uint8_t)(oy & 0xFF));
    spi_write_reg(ICM_B4, ICM_OFFUSER3, (uint8_t)(oz & 0xFF));
    spi_write_reg(ICM_B4, ICM_OFFUSER4, (uint8_t)((r4 & 0xF0U) | ((oz >> 8) & 0x0F)));
}

/* Moyenne gyro/accel sur N échantillons (polling, 1 kHz). Retourne true si immobile. */
static bool cal_measure(float g[3], float a[3])
{
    int32_t sg[3] = {0, 0, 0}, sa[3] = {0, 0, 0}, sg1[3] = {0, 0, 0};
    int16_t gmin[3] = {32767, 32767, 32767}, gmax[3] = {-32768, -32768, -32768};
    uint8_t b[12];
    bool still = true;

    for (uint32_t n = 0; n < CAL_NSAMPLES; n++) {
        spi_read_burst_b0(ICM42688_REG_ACCEL_DATA_X1, b, 12U);
        for (uint8_t k = 0; k < 3U; k++) {
            int16_t av = (int16_t)((b[2U*k] << 8) | b[2U*k + 1U]);
            int16_t gv = (int16_t)((b[6U + 2U*k] << 8) | b[7U + 2U*k]);
            sa[k] += av;
            sg[k] += gv;
            if (n < CAL_NSAMPLES / 2U) sg1[k] += gv;
            if (gv < gmin[k]) gmin[k] = gv;
            if (gv > gmax[k]) gmax[k] = gv;
        }
        LL_mDelay(1);
    }
    for (uint8_t k = 0; k < 3U; k++) {
        float h1 = (float)sg1[k] / (float)(CAL_NSAMPLES / 2U);
        float h2 = (float)(sg[k] - sg1[k]) / (float)(CAL_NSAMPLES - CAL_NSAMPLES / 2U);
        g[k] = (float)sg[k] / (float)CAL_NSAMPLES;
        a[k] = (float)sa[k] / (float)CAL_NSAMPLES;
        if ((float)(gmax[k] - gmin[k]) > CAL_SPAN_MAX_LSB) still = false;
        if (fabsf(h1 - h2) > CAL_HALF_DIFF_MAX)            still = false;
    }
    return still;
}

bool ICM42688_CalibrateGyro(void)
{
    float g[3], a[3], r[3], bsum = 0.0f;
    int16_t off[3];
    bool still = false;
    int8_t sign = 0;

    if (debug_init) ST_PRINT("\n\rICM-42688-P : CALIBRATION GYRO (carte immobile)...");
    LL_mDelay(100);   /* établissement AAF / filtres UI */

    for (uint8_t t = 0; t < CAL_MAX_ATTEMPTS && !still; t++) {
        still = cal_measure(g, a);
        if (!still) {
            if (debug_init) ST_PRINT("\n\r  mouvement detecte, nouvel essai");
            LL_mDelay(200);
        }
    }
    if (!still) {
        if (debug_init) ST_PRINT("\n\rICM-42688-P : CALIBRATION GYRO FAIL (carte pas immobile)");
        return false;
    }

    /* Norme accel de référence (invariante à l'inclinaison) → gate Mahony */
    accel_norm_ref_lsb = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);

    for (uint8_t k = 0; k < 3U; k++) {
        if (fabsf(g[k]) > CAL_BIAS_MAX_LSB) {
            if (debug_init) ST_PRINT("\n\rICM-42688-P : CALIBRATION GYRO FAIL (biais hors plage +/-64 dps)");
            return false;
        }
        bsum += fabsf(g[k]);
        /* 1 LSB offset = 1/32 dps = 16,4/32 LSB gyro */
        off[k] = cal_clamp12(lroundf(g[k] * (32.0f / 16.4f)));
    }

    if (debug_init) {
        ST_PRINT("\n\r  biais (LSB) X="); st_print_i32((int32_t)g[0]);
        ST_PRINT(" Y=");                 st_print_i32((int32_t)g[1]);
        ST_PRINT(" Z=");                 st_print_i32((int32_t)g[2]);
        ST_PRINT("\n\r  |a| ref (mg) = "); st_print_i32((int32_t)(accel_norm_ref_lsb * 1000.0f / 2048.0f));
    }

    if (bsum < CAL_BIAS_MIN_LSB) {
        if (debug_init) ST_PRINT("\n\rICM-42688-P : CALIBRATION GYRO OK (biais deja < 0,5 dps, aucun offset)");
        return true;
    }

    /* Convention de signe non documentée : on teste +off puis -off */
    for (int8_t s = 1; s >= -1 && sign == 0; s -= 2) {
        icm42688_set_gyro_offset((int16_t)(s * off[0]), (int16_t)(s * off[1]), (int16_t)(s * off[2]));
        LL_mDelay(100);
        (void)cal_measure(r, a);
        if ((fabsf(r[0]) + fabsf(r[1]) + fabsf(r[2])) < 0.25f * bsum) sign = s;
    }
    if (sign == 0) {
        icm42688_set_gyro_offset(0, 0, 0);
        if (debug_init) ST_PRINT("\n\rICM-42688-P : CALIBRATION GYRO FAIL (offset sans effet)");
        return false;
    }

     /* 2e passe : rattrape le résiduel mesuré (sortie = brut + offset, sign = -1 chez toi) */
    int16_t v[3];
    for (uint8_t k = 0; k < 3U; k++) {
        v[k] = cal_clamp12((int32_t)(sign * off[k]) +
                           lroundf((float)sign * r[k] * (32.0f / 16.4f)));
    }
    icm42688_set_gyro_offset(v[0], v[1], v[2]);
    LL_mDelay(100);
    (void)cal_measure(r, a);

    /* Vérification lecture des registres */
    if (spi_read_reg(ICM_B4, ICM_OFFUSER0) != (uint8_t)(v[0] & 0xFF) ||
        spi_read_reg(ICM_B4, ICM_OFFUSER2) != (uint8_t)(v[1] & 0xFF) ||
        spi_read_reg(ICM_B4, ICM_OFFUSER3) != (uint8_t)(v[2] & 0xFF)) {
        if (debug_init) ST_PRINT("\n\rREG_OFFSET_USER (BANK4) MISMATCH");
    }

    if (debug_init) {
        ST_PRINT("\n\r  OFFSET_USER final (1/32 dps) X="); st_print_i32(v[0]);
        ST_PRINT(" Y="); st_print_i32(v[1]);
        ST_PRINT(" Z="); st_print_i32(v[2]);
        ST_PRINT("\n\r  residuel final (LSB) X="); st_print_i32((int32_t)r[0]);
        ST_PRINT(" Y="); st_print_i32((int32_t)r[1]);
        ST_PRINT(" Z="); st_print_i32((int32_t)r[2]);
        ST_PRINT("\n\rICM-42688-P : CALIBRATION GYRO OK");
    }
    return true;
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 * INITIALISATION ICM-42688-P
 * ═══════════════════════════════════════════════════════════════════════════ */
void ICM42688_Init(void)
{
    uint8_t cfg;
    uint32_t sr = SPI1->SR;

    NVIC_DisableIRQ(EXTI4_IRQn); 
    NVIC_DisableIRQ(DMA2_Stream0_IRQn); // ils ont surrement été activés par CubeMX, mais on ne veut pas d'interruptions pendant l'init du capteur (DMA2_Stream0 = SPI1_RX)
    
    if (debug_init)
        print_to_console("\n\r\n\rICM-42688-P : initialisation...", sizeof("\n\r\n\rICM-42688-P : initialisation..."));
    
    ICM42688_CS_HIGH();   // ou ICM42688_CS_HIGH();
    LL_mDelay(1);

    LL_SPI_Enable(SPI1);
    LL_mDelay(100);

    /* ── ICI, avant tout accès registre ── */
    sr = SPI1->SR;
    if (sr & SPI_SR_MODF) {
        if (debug_init)
            print_to_console("\n\rSPI1 : MODF detected, clearing...", 
                       sizeof("\n\rSPI1 : MODF detected, clearing..."));
        (void)SPI1->SR;
        LL_SPI_Enable(SPI1);
    }
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 1 : Vérification WHO_AM_I
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * WHO_AM_I (BANK 0, 0x75) doit retourner 0x47 pour ICM-42688-P.
     * Si 0x00 ou autre → capteur absent ou SPI KO → bloquer firmware.
     */
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_WHO_AM_I);
    if (cfg != ICM42688_WHO_AM_I_VAL) {
        if (debug_init) {
            print_to_console("\n\rICM-42688-P : WHO_AM_I FAILED (read: 0x", 
                           sizeof("\n\rICM-42688-P : WHO_AM_I FAILED (read: 0x"));
            /* Debug : afficher valeur lue en hexa */
            char hex[3];
            hex[0] = (cfg >> 4) > 9 ? 'A' + (cfg >> 4) - 10 : '0' + (cfg >> 4);
            hex[1] = (cfg & 0x0F) > 9 ? 'A' + (cfg & 0x0F) - 10 : '0' + (cfg & 0x0F);
            hex[2] = ')';
            print_to_console(hex, 3);
        }
        while(1);  /* Stopper firmware — ICM-42688-P non détecté */
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : WHO_AM_I OK (0x47)", 
                           sizeof("\n\rICM-42688-P : WHO_AM_I OK (0x47)"));
    }

    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 2 : Soft Reset
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * DEVICE_CONFIG (BANK 0, 0x11) bit[0] = SOFT_RESET_CONFIG
     * Datasheet §14.1 : reset tous les registres à leurs valeurs par défaut.
     * Délai stabilisation : 1 ms typ., 10 ms max après reset.
     */

    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_DEVICE_CONFIG, 
                  ICM42688_DEVICE_CONFIG_SOFT_RESET_CONFIG);
    LL_mDelay(5);  /* Marge sécurité : 5 ms */
    
    if (debug_init)
        print_to_console("\n\rICM-42688-P : SOFT_RESET OK", sizeof("\n\rICM-42688-P : SOFT_RESET OK"));
    
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 3 : Configuration PWR_MGMT0 — Mode Low-Noise
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * PWR_MGMT0 (BANK 0, 0x4E) :
     * - GYRO_MODE  = 0b11 (Low-Noise)  → meilleur SNR, biais gyro plus stable
     * - ACCEL_MODE = 0b11 (Low-Noise)  → meilleur SNR vs Low-Power
     * - TEMP_DIS   = 1 (température disabled, utile pour compensation thermique mais pa utilisé pour l'instant)
     * - IDLE       = 0 (capteurs actifs)
     * 
     * Justification Low-Noise : drone cinelifter nécessite stabilité maximale.
     * Consommation ~1.9 mA gyro + 0.68 mA accel en LN @ 1kHz (datasheet table 1).
     */
    cfg = ICM42688_PWR_MGMT0_GYRO_MODE_LN | ICM42688_PWR_MGMT0_ACCEL_MODE_LN | ICM42688_PWR_MGMT0_TEMP_DIS;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_PWR_MGMT0, cfg);
    LL_mDelay(50);  /* Startup time gyro : 30 ms typ. (datasheet table 7) */
    
    /* Vérification lecture */
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_PWR_MGMT0);
    if ((cfg & 0x3FU) != (ICM42688_PWR_MGMT0_GYRO_MODE_LN | ICM42688_PWR_MGMT0_ACCEL_MODE_LN | ICM42688_PWR_MGMT0_TEMP_DIS)) {
        if (debug_init)
            print_to_console("\n\rREG_PWR_MGMT0 MISMATCH", sizeof("\n\rREG_PWR_MGMT0 MISMATCH"));
        while(1);
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : REG_PWR_MGMT0 OK (Low-Noise mode)", 
                           sizeof("\n\rICM-42688-P : REG_PWR_MGMT0 OK (Low-Noise mode)"));
    }

    ICM42688_SelfTest();
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 4 : Configuration GYRO_CONFIG0 — ±2000 dps, ODR = ICM42688_ODR_HZ
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * GYRO_CONFIG0 (BANK 0, 0x4F) :
     * - GYRO_FS_SEL = 0b000 → ±2000 dps (16.4 LSB/dps)
     * - GYRO_ODR    = ICM42688_ODR_SEL_GYRO (0b0110 → 1 kHz, voir icm42688_driver.h)
     * 
     * Choix ±2000 dps : vitesses angulaires max ~500-700 dps en vol stable,
     * marge confortable + meilleure résolution que ±4000 dps.
     */
    cfg = ICM42688_GYRO_FS_SEL_2000DPS | ICM42688_ODR_SEL_GYRO;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_CONFIG0, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_CONFIG0);
    if (cfg != (ICM42688_GYRO_FS_SEL_2000DPS | ICM42688_ODR_SEL_GYRO)) {
        if (debug_init)
            print_to_console("\n\rREG_GYRO_CONFIG0 MISMATCH", sizeof("\n\rREG_GYRO_CONFIG0 MISMATCH"));
        while(1);
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : REG_GYRO_CONFIG0 OK (+/-2000dps, 1kHz)", 
                           sizeof("\n\rICM-42688-P : REG_GYRO_CONFIG0 OK (+/-2000dps, 1kHz)"));
    }
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 5 : Configuration ACCEL_CONFIG0 — ±16g, ODR = ICM42688_ODR_HZ
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * ACCEL_CONFIG0 (BANK 0, 0x50) :
     * - ACCEL_FS_SEL = 0b000 → ±16g (2048 LSB/g)
     * - ACCEL_ODR    = ICM42688_ODR_SEL_ACCEL (aligné sur le gyro : UI_DRDY et donc
     *                  la fast loop tombent à l'ODR sans dépendre de quel capteur pilote le DRDY.
     *                  Le filtrage accel reste fixé par l'AAF accel ~42 Hz, BANK 2)
     * 
     * Choix ±16g : manoeuvres agressives peuvent atteindre 6-8g,
     * ±16g évite saturation (plus de marge que ±8g du MPU6000).
     */
    cfg = ICM42688_ACCEL_FS_SEL_16G | ICM42688_ODR_SEL_ACCEL;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_ACCEL_CONFIG0, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_ACCEL_CONFIG0);
    if (cfg != (ICM42688_ACCEL_FS_SEL_16G | ICM42688_ODR_SEL_ACCEL)) {
        if (debug_init)
            print_to_console("\n\rREG_ACCEL_CONFIG0 MISMATCH", sizeof("\n\rREG_ACCEL_CONFIG0 MISMATCH"));
        while(1);
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : REG_ACCEL_CONFIG0 OK (+/-16g, 1kHz)", 
                           sizeof("\n\rICM-42688-P : REG_ACCEL_CONFIG0 OK (+/-16g, 1kHz)"));
    }
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 6 : Configuration AAF (Anti-Aliasing Filter) gyro uniquement — BANK 1
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * AAF GYRO (anti-aliasing filter), BANK 1, registres GYRO_CONFIG_STATIC3/4/5.
     *
     * Place dans la chaîne interne (datasheet fig. 9) :
     *   ADC → décimation 32 kHz → notch d'usine → AAF → offset utilisateur → UI filter → ODR
     * L'AAF tourne donc à 32 kHz, AVANT la décimation vers l'ODR : c'est lui, avec
     * le UI filter, qui protège du repliement quand la puce ne garde qu'un
     * échantillon sur 32 (ODR 1 kHz → Nyquist 500 Hz).
     *
     * Choix : 3 dB à 213 Hz (table §5.3 : DELT = 5, DELTSQR = 25, BITSHIFT = 10).
     *   - Bande utile du contrôle : quelques dizaines de Hz (coupure de la boucle
     *     ~5–15 Hz sur un 9") → 213 Hz laisse de la marge de phase.
     *   - Atténuation à Nyquist (500 Hz) : ≈ −15 dB SI l'AAF est d'ordre 2 (l'ordre
     *     n'est pas donné par la datasheet, À VÉRIFIER) ; ≈ −29 dB à 1,14 kHz,
     *     d'où la 9e harmonique de passage de pales qui se replierait à ~137 Hz.
     *   - Les harmoniques moteur dans la bande (1×, 3× ≈ 126 / 379 Hz à 4S plein
     *     gaz, estimation) ne sont PAS retirées ici : notchs logiciels plus tard.
     * Ancienne config (8 kHz) : DELT 21 / DELTSQR 440 / BITSHIFT 6 = 997 Hz,
     * inutilisable à 1 kHz (au-delà de Nyquist).
     *
     * Le notch d'usine (GYRO_NF, 1–3 kHz) vise le pic de résonance du MEMS, pas
     * les moteurs : on le laisse tel quel. Ne JAMAIS écrire 0x00 dans
     * GYRO_CONFIG_STATIC2 (désactiverait AAF et notch). L'AAF est actif en mode
     * Low-Noise dès que ces registres sont non nuls (pas de bit enable séparé).
     */
    spi_write_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC3_B1, 0x05U);  // GYRO_AAF_DELT = 5
    LL_mDelay(10);
    spi_write_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC4_B1, 0x19U);  // GYRO_AAF_DELTSQR[7:0] = 25
    LL_mDelay(10);
    spi_write_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC5_B1, 0xA0U);  // BITSHIFT=10 | DELTSQR[11:8]=0 → 213 Hz
    LL_mDelay(10);
    if (spi_read_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC3_B1) != 0x05U ||
        spi_read_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC4_B1) != 0x19U ||
        spi_read_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC5_B1) != 0xA0U) {
        if (debug_init)
            print_to_console("\n\rREG_GYRO_CONFIG_STATIC3/4/5 (AAF 213 Hz) MISMATCH",
                           sizeof("\n\rREG_GYRO_CONFIG_STATIC3/4/5 (AAF 213 Hz) MISMATCH"));
        while(1);
    }

    /* Vérification lecture (BANK 1) que le premier registre est correct */
    cfg = spi_read_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC2_B1);
    if (cfg != 0xA0U) {
        if (debug_init)
            print_to_console("\n\rREG_GYRO_CONFIG_STATIC2 (BANK1) MISMATCH", 
                           sizeof("\n\rREG_GYRO_CONFIG_STATIC2 (BANK1) MISMATCH"));
        while(1);
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : AAF gyro config OK (BANK 1, 213 Hz)", 
                           sizeof("\n\rICM-42688-P : AAF gyro config OK (BANK 1, 213 Hz)"));
    }

    /* AAF accel (BANK 2) — ACTIVÉ à ~42 Hz : c'est lui qui fixe la bande passante accel */
    spi_write_reg(ICM42688_BANK_SEL_2, ICM42688_REG_ACCEL_CONFIG_STATIC2_B2, 0x02U); // ACCEL_AAF_DIS=0, DELT=1
    LL_mDelay(10);
    spi_write_reg(ICM42688_BANK_SEL_2, ICM42688_REG_ACCEL_CONFIG_STATIC3_B2, 0x01U); // DELTSQR[7:0] = 1
    LL_mDelay(10);
    spi_write_reg(ICM42688_BANK_SEL_2, ICM42688_REG_ACCEL_CONFIG_STATIC4_B2, 0xF0U); // BITSHIFT=15 | DELTSQR[11:8]=0 → 42 Hz exact
    LL_mDelay(10);
    
    if (debug_init)
        print_to_console("\n\rICM-42688-P : AAF accel enabled (BANK 2)", 
                       sizeof("\n\rICM-42688-P : AAF accel enabled (BANK 2)"));
    
    /* Retour BANK 0 pour config interruptions */
    icm42688_select_bank(ICM42688_BANK_SEL_0);

     /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 7 : Configuration filtres UI (User Interface)
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * GYRO_CONFIG1 (BANK 0, 0x51) :
     * - GYRO_UI_FILT_ORD = 2nd order (compromis latence/rejection)
     * - GYRO_DEC2_M2_ORD = ignored (AAF enabled)
     * 
     * L'ICM-42688-P a un AAF (Anti-Aliasing Filter) hardware dédié en amont
     * du filtre UI. Le filtre UI agit en post-AAF comme LPF additionnel.
     */

    /* UI filter gyro : 2nd order (post-AAF) */
    cfg = ICM42688_GYRO_UI_FILT_ORD_2ND;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_CONFIG1, cfg); // GYRO_UI_FILT_ORD = 2nd
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_CONFIG1);
    if ((cfg & 0x0CU) != ICM42688_GYRO_UI_FILT_ORD_2ND) {
        if (debug_init)
            print_to_console("\n\rREG_GYRO_CONFIG1 MISMATCH", sizeof("\n\rREG_GYRO_CONFIG1 MISMATCH"));
        while(1);
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : REG_GYRO_CONFIG1 OK (UI filt 2nd order)", 
                           sizeof("\n\rICM-42688-P : REG_GYRO_CONFIG1 OK (UI filt 2nd order)"));
    }
    
    /*
     * GYRO_ACCEL_CONFIG0 (BANK 0, 0x52) : bande passante des UI filters.
     * Valeurs à ODR 1 kHz (datasheet §5.5, tables 3 dB / retard de groupe à DC) :
     *
     *   GYRO, 2e ordre (GYRO_CONFIG1)   3 dB      retard
     *     BW 0  (ODR/2)  ← choisi        493 Hz    0,7 ms
     *     BW 15 (basse latence)        2096 Hz    0,2 ms   (plus de filtrage avant décimation)
     *     BW 14                         262 Hz    1,5 ms
     *     BW 1  (ODR/4)                 231 Hz    2,1 ms
     *
     * BW 0 : +0,5 ms par rapport à BW 15 (≈ 1,8° de phase à 10 Hz, 2,7° à 15 Hz)
     * pour ~−3 dB de plus à Nyquist et ~−12 dB vers 1 kHz, en plus de l'AAF. Les
     * raies de passage de pales repliées (9×, 12× f_rot) étant le vrai risque,
     * on paie ce demi-milliseconde. À revoir avec les spectres blackbox.
     *
     *   ACCEL : BW 0 (ODR/2, 493 Hz) → l'AAF accel à 42 Hz reste le seul limiteur,
     *   comme à 8 kHz. (ODR/20 ferait 48 Hz / 6,1 ms de retard à 1 kHz.)
     */
    cfg = ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_2 | ICM42688_GYRO_UI_FILT_BW_0;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_ACCEL_CONFIG0, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_ACCEL_CONFIG0);
    if (cfg != (ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_2 | ICM42688_GYRO_UI_FILT_BW_0)) {
        if (debug_init)
            print_to_console("\n\rREG_GYRO_ACCEL_CONFIG0 MISMATCH", 
                           sizeof("\n\rREG_GYRO_ACCEL_CONFIG0 MISMATCH"));
        while(1);
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : REG_GYRO_ACCEL_CONFIG0 OK (accel UI filt ODR/2, gyro UI filt ODR/2)", 
                           sizeof("\n\rICM-42688-P : REG_GYRO_ACCEL_CONFIG0 OK (accel UI filt ODR/2, gyro UI filt ODR/2)"));
    }
    
    /*
     * ACCEL_CONFIG1 (BANK 0, 0x53) :
     * - ACCEL_UI_FILT_ORD = 2nd order (compromis latence/rejection)
     * - ACCEL_DEC2_M2_ORD = ignored (AAF enabled)
     */
    cfg = ICM42688_ACCEL_UI_FILT_ORD_2ND;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_ACCEL_CONFIG1, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_ACCEL_CONFIG1);
    if ((cfg & 0x0FU) != ICM42688_ACCEL_UI_FILT_ORD_2ND) {
        if (debug_init)
            print_to_console("\n\rREG_ACCEL_CONFIG1 MISMATCH", sizeof("\n\rREG_ACCEL_CONFIG1 MISMATCH"));
        while(1);
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : REG_ACCEL_CONFIG1 OK (UI filt 2nd order)", 
                           sizeof("\n\rICM-42688-P : REG_ACCEL_CONFIG1 OK (UI filt 2nd order)"));
    }
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 8 : Configuration interruptions INT2 — UI Data Ready
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * INT_CONFIG (BANK 0, 0x14) :
     * - INT2_MODE     = Pulsed (100 µs pulse à chaque DRDY)
     * - INT2_DRIVE    = Push-pull (compatible EXTI STM32)
     * - INT2_POLARITY = Active-high (rising edge EXTI3)
     * 
     * Justification pulsed : évite de devoir clear l'interrupt status en lecture.
     * La pulse 100 µs suffit pour déclencher EXTI (latence NVIC <1 µs).
     */
    cfg = ICM42688_INT_CONFIG_INT2_MODE_PULSED |
          ICM42688_INT_CONFIG_INT2_DRIVE_PP |
          ICM42688_INT_CONFIG_INT2_POLARITY_HIGH;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INT_CONFIG, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INT_CONFIG);
    if ((cfg & 0x38U) != (ICM42688_INT_CONFIG_INT2_MODE_PULSED | 
                          ICM42688_INT_CONFIG_INT2_DRIVE_PP | 
                          ICM42688_INT_CONFIG_INT2_POLARITY_HIGH)) {
        if (debug_init)
            print_to_console("\n\rREG_INT_CONFIG MISMATCH", sizeof("\n\rREG_INT_CONFIG MISMATCH"));
        while(1);
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : REG_INT_CONFIG OK (INT2 pulsed, PP, high)", 
                           sizeof("\n\rICM-42688-P : REG_INT_CONFIG OK (INT2 pulsed, PP, high)"));
    }
    
    /*
     * INT_CONFIG1 (BANK 0, 0x64) :
     * - INT_ASYNC_RESET       = 0 : OBLIGATOIRE, datasheet §12.6 (défaut = 1,
     *                               "user should change to 0 for proper INT1/INT2 operation")
     * - INT_TPULSE_DURATION   = 1 : pulse 8 µs   (obligatoire si ODR >= 4 kHz, gardé à 1 kHz :
     *                               un front court suffit à l'EXTI)
     * - INT_TDEASSERT_DISABLE = 1 : pas de 100 µs de dé-assertion (idem)
     */
    cfg = ICM42688_INT_CONFIG1_TPULSE_8US | ICM42688_INT_CONFIG1_TDEASSERT_DIS;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INT_CONFIG1, cfg);
    LL_mDelay(10);

    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INT_CONFIG1);
    if ((cfg & 0x70U) != (ICM42688_INT_CONFIG1_TPULSE_8US | ICM42688_INT_CONFIG1_TDEASSERT_DIS)) {
        if (debug_init)
            print_to_console("\n\rREG_INT_CONFIG1 MISMATCH", sizeof("\n\rREG_INT_CONFIG1 MISMATCH"));
        while(1);
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : REG_INT_CONFIG1 OK (pulse 8us, async reset off)",
                           sizeof("\n\rICM-42688-P : REG_INT_CONFIG1 OK (pulse 8us, async reset off)"));
    }
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 9 : Configuration endianness et FIFO (optionnel)
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * INTF_CONFIG0 (BANK 0, 0x4C) :
     * - SENSOR_DATA_ENDIAN = Big-endian (MSB first, standard SPI)
     * 
     * Par défaut l'ICM-42688-P est en little-endian pour compatibilité I2C.
     * En SPI, big-endian est plus naturel (MSB arrive en premier).
     * 
     * Note : si on garde little-endian, il faut inverser les octets lors du
     * parsing dans le callback DMA. Ici on force big-endian pour cohérence
     * avec le code de parsing (compatible MPU6000).
     */
    cfg = ICM42688_INTF_CONFIG0_SENSOR_DATA_ENDIAN_BIG;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INTF_CONFIG0, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INTF_CONFIG0);
    if (!(cfg & ICM42688_INTF_CONFIG0_SENSOR_DATA_ENDIAN_BIG)) {
        if (debug_init)
            print_to_console("\n\rREG_INTF_CONFIG0 MISMATCH", sizeof("\n\rREG_INTF_CONFIG0 MISMATCH"));
        while(1);
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : REG_INTF_CONFIG0 OK (big-endian)", 
                           sizeof("\n\rICM-42688-P : REG_INTF_CONFIG0 OK (big-endian)"));
    }

    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 9b : Calibration biais gyro (OFFSET_USER, Bank 4) + |a| de référence
     * ══════════════════════════════════════════════════════════════════════ */
    (void)ICM42688_CalibrateGyro();   /* non bloquant : en cas d'échec, biais non compensé */
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 10 : Préparation buffers DMA (burst read 13 octets)
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * Burst : 1 adresse + 6 ACCEL + 6 GYRO = 13 octets
     * Adresse départ : ICM42688_REG_ACCEL_DATA_X1 (0x1F) → 12 lectures consécutives jusqu'à GYRO_DATA_Z0
     * 
     * tx_buf[0] = adresse read (bit7=1) : 0x1F | 0x80 = 0x9F
     * tx_buf[1..12] = dummy 0x00 pour générer clocks SPI
     */
    memset(dma_tx_buf, 0x00, ICM42688_BURST_LEN);
    dma_tx_buf[0] = ICM42688_REG_ACCEL_DATA_X1 | ICM42688_READ_FLAG;  /* 0x9F */
    
    /* Préparer DMA en mode Normal (réarmement manuel après chaque transfert) */
    LL_DMA_SetMemoryAddress(ICM42688_DMA, ICM42688_DMA_TX_STREAM, (uint32_t)dma_tx_buf);
    LL_DMA_SetPeriphAddress(ICM42688_DMA, ICM42688_DMA_TX_STREAM, (uint32_t)&SPI1->DR);
    LL_DMA_SetDataLength(ICM42688_DMA, ICM42688_DMA_TX_STREAM, ICM42688_BURST_LEN);
    
    LL_DMA_SetMemoryAddress(ICM42688_DMA, ICM42688_DMA_RX_STREAM, (uint32_t)dma_rx_buf);
    LL_DMA_SetPeriphAddress(ICM42688_DMA, ICM42688_DMA_RX_STREAM, (uint32_t)&SPI1->DR);
    LL_DMA_SetDataLength(ICM42688_DMA, ICM42688_DMA_RX_STREAM, ICM42688_BURST_LEN);
    
    /* Clear flags DMA */
    LL_DMA_ClearFlag_TC0(ICM42688_DMA);   /* RX stream = Stream0 */
    LL_DMA_ClearFlag_TC3(ICM42688_DMA);   /* TX stream = Stream3 */
    LL_DMA_ClearFlag_TE0(ICM42688_DMA);
    LL_DMA_ClearFlag_TE3(ICM42688_DMA);

    /* Enable Transfer Complete interrupt sur RX stream uniquement */
    LL_DMA_EnableIT_TC(ICM42688_DMA, ICM42688_DMA_RX_STREAM);
    
    if (debug_init)
        print_to_console("\n\rICM-42688-P : DMA buffers prepared (burst 13 bytes)", 
                       sizeof("\n\rICM-42688-P : DMA buffers prepared (burst 13 bytes)"));
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 12 : Initialisation terminée, on active les interruption coté ICM-42688-P, et coté STM32 (NVIC) pour EXTI4 (INT2)
     * ══════════════════════════════════════════════════════════════════════ */

    /*
     * INT_SOURCE3 (BANK 0, 0x68) — c'est le registre des sources de la broche INT2
     * (INT_SOURCE0 = celles d'INT1, non câblée ici) :
     * - UI_DRDY_INT2_EN = 1 (router UI Data Ready → INT2 pin)
     * 
     * UI_DRDY à ICM42688_ODR_HZ (gyro et accel à la même ODR) → fast loop / PID.
     */
    cfg = ICM42688_INT_SOURCE3_UI_DRDY_INT2_EN;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INT_SOURCE3, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INT_SOURCE3);
    if (!(cfg & ICM42688_INT_SOURCE3_UI_DRDY_INT2_EN)) {
        if (debug_init)
            print_to_console("\n\rREG_INT_SOURCE3 MISMATCH", sizeof("\n\rREG_INT_SOURCE3 MISMATCH"));
        while(1);
    }
    else {
        if (debug_init)
            print_to_console("\n\rICM-42688-P : REG_INT_SOURCE3 OK (UI_DRDY -> INT2)", 
                           sizeof("\n\rICM-42688-P : REG_INT_SOURCE3 OK (UI_DRDY -> INT2)"));
    }

    if (debug_init)
        print_to_console("\n\rICM-42688-P : Init complete, ready for fast loop 1kHz\n\r", 
                       sizeof("\n\rICM-42688-P : Init complete, ready for fast loop 1kHz\n\r"));


    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    NVIC_ClearPendingIRQ(EXTI4_IRQn);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    /* ── 9. Configurer NVIC pour EXTI4 (INT DATA_RDY) ── */
    NVIC_EnableIRQ(EXTI4_IRQn);

    icm_ready = true;
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 * LECTURE DMA
 * ═══════════════════════════════════════════════════════════════════════════ */
void ICM42688_Start_DMA_Read(void)
{
    /*
     * Réarmer les streams DMA (mode Normal, pas Circular).
     * Séquence :
     * 1. Désactiver streams
     * 2. Recharger compteurs NDTR
     * 3. Activer streams
     * 4. CS LOW
     * 5. Activer DMA requests SPI TX/RX
     * 
     * Le callback RX Complete mettra CS HIGH et parsera les données.
     */
    
    /* Désactiver streams */
    LL_DMA_DisableStream(ICM42688_DMA, ICM42688_DMA_TX_STREAM);
    LL_DMA_DisableStream(ICM42688_DMA, ICM42688_DMA_RX_STREAM);
    while (LL_DMA_IsEnabledStream(ICM42688_DMA, ICM42688_DMA_RX_STREAM));
    while (LL_DMA_IsEnabledStream(ICM42688_DMA, ICM42688_DMA_TX_STREAM));
    
    /* Clear flags DMA (évite faux TC si flag resté levé) */
    LL_DMA_ClearFlag_TC0(ICM42688_DMA);   /* RX stream = Stream0 */
    LL_DMA_ClearFlag_TC3(ICM42688_DMA);   /* TX stream = Stream3 */
    LL_DMA_ClearFlag_TE0(ICM42688_DMA);
    LL_DMA_ClearFlag_TE3(ICM42688_DMA);

    /* Recharger NDTR (compteurs) */
    LL_DMA_SetDataLength(ICM42688_DMA, ICM42688_DMA_TX_STREAM, ICM42688_BURST_LEN);
    LL_DMA_SetDataLength(ICM42688_DMA, ICM42688_DMA_RX_STREAM, ICM42688_BURST_LEN);
    
   /* CS LOW → démarrer transaction SPI */
    dma_busy = true;
    ICM42688_CS_LOW();

    /* Activer streams */
    LL_DMA_EnableStream(ICM42688_DMA, ICM42688_DMA_RX_STREAM);
    LL_DMA_EnableStream(ICM42688_DMA, ICM42688_DMA_TX_STREAM);
    
    /* Activer DMA requests SPI (TX puis RX) */
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_SPI_EnableDMAReq_RX(SPI1);
}
 
void ICM42688_DMA_RX_Complete_Callback(void)
{
    /*
     * Callback Transfer Complete DMA RX (Stream0).
     * Appelé depuis DMA2_Stream0_IRQHandler quand les 13 octets sont reçus (à l'ODR).
     * 
     * Séquence :
     * 1. Désactiver DMA requests SPI
     * 2. CS HIGH (fin transaction)
     * 3. Parser les octets reçus (big-endian)
     * 4. Convertir en unités SI (rad/s, m/s²)
     * 5. Lever flag data_ready pour la fast loop
     */
    
    /* Désactiver DMA requests */
    LL_SPI_DisableDMAReq_TX(SPI1);
    LL_SPI_DisableDMAReq_RX(SPI1);
    
    /* Attendre fin shift register SPI avant de relâcher CS */
    while (LL_SPI_IsActiveFlag_BSY(SPI1));
    
    /* CS HIGH */
    ICM42688_CS_HIGH();
    dma_busy = false;   /* dma_rx_buf est stable : seul un nouveau DRDY le réécrira */

    /* ──────────────────────────────────────────────────────────────────────
     * Parsing données burst (big-endian, MSB first) :
     * 
     * dma_rx_buf[0]  : dummy (adresse echo)
     * dma_rx_buf[1]  : ACCEL_DATA_X1 (MSB)
     * dma_rx_buf[2]  : ACCEL_DATA_X0 (LSB)
     * dma_rx_buf[3]  : ACCEL_DATA_Y1
     * dma_rx_buf[4]  : ACCEL_DATA_Y0
     * dma_rx_buf[5]  : ACCEL_DATA_Z1
     * dma_rx_buf[6]  : ACCEL_DATA_Z0
     * dma_rx_buf[7]  : GYRO_DATA_X1
     * dma_rx_buf[8]  : GYRO_DATA_X0
     * dma_rx_buf[9]  : GYRO_DATA_Y1
     * dma_rx_buf[10] : GYRO_DATA_Y0
     * dma_rx_buf[11] : GYRO_DATA_Z1
     * dma_rx_buf[12] : GYRO_DATA_Z0
     * ────────────────────────────────────────────────────────────────────── */
    
    /* Accéléromètre (big-endian → MSB first) */
    icm42688.raw_ax = (int16_t)((dma_rx_buf[1] << 8) | dma_rx_buf[2]);
    icm42688.raw_ay = (int16_t)((dma_rx_buf[3] << 8) | dma_rx_buf[4]);
    icm42688.raw_az = (int16_t)((dma_rx_buf[5] << 8) | dma_rx_buf[6]);
    

    /* Gyroscope (big-endian → MSB first) */
    icm42688.raw_gx = (int16_t)((dma_rx_buf[7]  << 8) | dma_rx_buf[8]);
    icm42688.raw_gy = (int16_t)((dma_rx_buf[9]  << 8) | dma_rx_buf[10]);
    icm42688.raw_gz = (int16_t)((dma_rx_buf[11] << 8) | dma_rx_buf[12]);
    
    icm42688.gx = (float)icm42688.raw_gx * ICM42688_GYRO_SCALE_2000DPS_RAD_S;
    icm42688.gy = (float)icm42688.raw_gy * ICM42688_GYRO_SCALE_2000DPS_RAD_S;
    icm42688.gz = (float)icm42688.raw_gz * ICM42688_GYRO_SCALE_2000DPS_RAD_S;

    /* Accel : calibration 6 faces (biais + matrice 3x3) appliquée ici, en m/s².
     * Sans calibration valide : échelle nominale ±16 g. */
    {
        const int16_t ra[3] = { icm42688.raw_ax, icm42688.raw_ay, icm42688.raw_az };
        const int16_t rg[3] = { icm42688.raw_gx, icm42688.raw_gy, icm42688.raw_gz };
        float a[3];
        AccelCal_Feed(ra, rg);      /* no-op hors calibration */
        AccelCal_Apply(ra, a);
        icm42688.ax = a[0];
        icm42688.ay = a[1];
        icm42688.az = a[2];
    }
    
    /* ── Debug IMU : instantané seulement, le print se fait hors ISR ──
     * On ne print PLUS ici : un print bloquant (~0,55 ms) dans cette ISR de
     * priorité 1 faisait perdre ~4 DRDY à chaque ligne. L'ISR se contente de
     * copier les valeurs dans dbg_snap (quelques dizaines de ns), 1 échantillon
     * sur ICM42688_DEBUG_PRINT_DIV (~2 Hz) ; ICM42688_Debug_Process(), appelé
     * depuis la boucle principale, formate et envoie la ligne pendant le temps
     * libre, interruptible par toutes les IRQ.
     * Le print n'a lieu QUE si une calibration accel valide est appliquée
     * (chargée de la flash au boot, ou venant d'être faite) : sans elle, l'accel
     * affiché ne serait que l'échelle nominale, peu utile. Pendant une calibration,
     * accel_calib coupe debug_imu (voir dbg_restore). */
    static uint16_t dbg_div = 0;
    if (debug_imu && AccelCal_IsValid() && ++dbg_div >= ICM42688_DEBUG_PRINT_DIV)
    {
        dbg_div = 0;
        /* Poignée de main sans couper les IRQ : l'ISR n'écrit que si la boucle
         * principale a fini de lire (dbg_snap_ready == false), et lève le flag
         * APRÈS avoir tout écrit. Si l'instantané précédent n'a pas encore été
         * consommé, on saute simplement celui-ci. */
        if (!dbg_snap_ready) {
            dbg_snap.raw_gx = icm42688.raw_gx;
            dbg_snap.raw_gy = icm42688.raw_gy;
            dbg_snap.raw_gz = icm42688.raw_gz;
            dbg_snap.ax     = icm42688.ax;
            dbg_snap.ay     = icm42688.ay;
            dbg_snap.az     = icm42688.az;
            dbg_snap.lost   = icm42688_dma_lost;
            dbg_snap_ready  = true;
        }
    }

    /* ──────────────────────────────────────────────────────────────────────
     * Flag data_ready → la fast loop peut consommer gx/gy/gz
     * ────────────────────────────────────────────────────────────────────── */
    icm42688.data_ready = true;
}
 
/* ─── Print debug IMU, côté boucle principale ──────────────────────────────
 * Consomme l'instantané posé par ICM42688_DMA_RX_Complete_Callback. Le print
 * reste bloquant (~0,55 ms, polling UART), mais il tourne en mode thread : les
 * IRQ (DRDY, DMA SPI, CRSF…) le préemptent normalement, il n'utilise donc que le
 * temps CPU que personne d'autre ne réclame. Seul effet : il retarde d'autant la
 * slow loop qui le suit (dt est mesuré, donc sans conséquence).
 * /!\ Si un jour la fast loop / PID tourne dans la boucle principale (au lieu
 * de l'ISR), ce print la bloquerait : à revoir à ce moment-là. */
void ICM42688_Debug_Process(void)
{
    if (!dbg_snap_ready) return;

    /* Copie locale puis libération immédiate du tampon pour l'ISR */
    icm_dbg_snap_t s = dbg_snap;
    dbg_snap_ready = false;

    /* Gyro brut → rad/s × 1000 sans flottant :
     * 16,4 LSB/(°/s) × 57,2958 (°/rad) = 939,7 LSB/(rad/s)
     * → rad/s × 1000 = raw × 1000 / 939,7 = raw × 10000 / 9397 */
    int32_t gx = (int32_t)s.raw_gx * 10000 / 9397;
    int32_t gy = (int32_t)s.raw_gy * 10000 / 9397;
    int32_t gz = (int32_t)s.raw_gz * 10000 / 9397;

    /* Accel : valeur CALIBRÉE (m/s²) × 100, et non le brut. Ça permet de vérifier
     * la calibration : posé à plat, on doit lire ~0 / 0 / ±9,81. */
    int16_t ax = (int16_t)(s.ax * 100.0f);
    int16_t ay = (int16_t)(s.ay * 100.0f);
    int16_t az = (int16_t)(s.az * 100.0f);

    print_to_console("G:", 2);
    print_gyro_rads(gx); UART_Debug_Transmit_Char_LL(' ');
    print_gyro_rads(gy); UART_Debug_Transmit_Char_LL(' ');
    print_gyro_rads(gz);
    print_to_console("  A:", 4);
    print_accel_mps2(ax); UART_Debug_Transmit_Char_LL(' ');
    print_accel_mps2(ay); UART_Debug_Transmit_Char_LL(' ');
    print_accel_mps2(az);
    print_to_console("  lost:", 7);       /* lectures DMA perdues puis relancées, doit rester 0 */
    st_print_i32((int32_t)s.lost);
    UART_Debug_Transmit_Char_LL('\n'); UART_Debug_Transmit_Char_LL('\r');
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CALLBACKS INTERRUPTIONS
 * ═══════════════════════════════════════════════════════════════════════════ */
/*
 * PRIORITÉS NVIC (réglées dans le .ioc) : EXTI4 et DMA2_Stream0 ont la MÊME
 * priorité de préemption (1). Ils ne peuvent donc pas s'interrompre l'un l'autre :
 * un DRDY qui arrive pendant le callback DMA attend sa fin.
 * Avant, EXTI4 était en 0 et pouvait préempter le callback DMA (2). Avec debug_imu
 * (print bloquant ~0,5 ms dans le callback), un DRDY relançait alors une lecture
 * pendant que la précédente n'était pas finie (CS toujours bas) → échantillons
 * corrompus, y compris ceux fournis à la calibration accel.
 * Rançon de la même priorité : si un callback dure plus d'une période, les DRDY sont
 * fusionnés (il ne reste qu'un seul "pending") → échantillons PERDUS, mais jamais
 * faux. C'est pour ça que le print debug_imu a été sorti de l'ISR (instantané +
 * ICM42688_Debug_Process dans la boucle principale).
 * /!\ Si on régénère CubeMX avec d'autres priorités, ce raisonnement tombe.
 */
void ICM42688_EXTI4_Callback(void)
{
    if (!icm_ready) return;
    /* Garde-fou : une lecture SPI dure ~5 µs et le DRDY revient toutes les 1000 µs.
     * Grâce aux priorités égales, si dma_busy est encore vrai ici, ce n'est pas une
     * lecture "en cours" mais une lecture PERDUE (erreur DMA, TC jamais reçu).
     * On ne la saute pas, sinon l'IMU resterait bloquée pour toujours : on remonte CS
     * pour clore la transaction côté ICM, puis Start_DMA_Read en relance une neuve
     * (le temps de désactiver les streams laisse CS haut assez longtemps). */
    if (dma_busy)
    {
        ICM42688_CS_HIGH();
        icm42688_dma_lost++;
    }
    /*
     * INT2 rising edge à l'ODR (UI_DRDY).
     * Déclenche immédiatement la lecture DMA.
     */
    ICM42688_Start_DMA_Read();
}
 
void ICM42688_DMA2_Stream0_Callback(void)
{
    /*
     * Wrapper pour appel depuis stm32f4xx_it.c (DMA2_Stream0_IRQHandler).
     * t_entry est lu en tout premier : FastLoop_Run s'en sert pour le dt réel
     * et pour mesurer le temps de toute la chaîne (lecture + boucle rapide).
     */
    uint32_t t_entry = FastLoop_Cycles();
    ICM42688_DMA_RX_Complete_Callback();
    FastLoop_Run(t_entry);
}