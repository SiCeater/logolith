/**
 * icm42688_driver.c
 * Driver LL ICM-42688-P — SPI1 + DMA2 — STM32F405
 * Gyro and accel, fast loop 1kHz, Low-Noise mode, AAF enabled
 * 
 * Architecture multi-banques (BANK 0/1/2/4) — accès via REG_BANK_SEL
 * Config optimisée pour drone cinelifter F330/F380 (stabilité prioritaire)
 */
 
#include "icm42688_driver.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_utils.h"
 
/* ═══════════════════════════════════════════════════════════════════════════
 * INSTANCE PUBLIQUE et BUFFERS DMA
 * ═══════════════════════════════════════════════════════════════════════════ */
icm42688_data_t icm42688 = {0};
 
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
    // volatile uint8_t dummy;
    // dummy = SPI1->DR;
    // dummy = SPI1->SR;
    // (void)dummy;

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
    // volatile uint8_t dummy;

    icm42688_select_bank(bank);

    // dummy = SPI1->DR;
    // dummy = SPI1->SR;
    // (void)dummy;

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
    // volatile uint8_t dummy;
    
    // icm42688_select_bank(bank);
    
    // dummy = SPI1->DR;
    // dummy = SPI1->SR;
    // (void)dummy;

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
 
/* ═══════════════════════════════════════════════════════════════════════════
 * INITIALISATION ICM-42688-P
 * ═══════════════════════════════════════════════════════════════════════════ */
void ICM42688_Init(void)
{
    uint8_t cfg;
    
    if (debug)
        print_to_console("\n\r\n\rICM-42688-P : initialisation...", sizeof("\n\r\n\rICM-42688-P : initialisation..."));

    ICM42688_CS_HIGH();
    LL_mDelay(1);
    
    LL_SPI_Enable(SPI1);
    LL_mDelay(100);
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 1 : Vérification WHO_AM_I
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * WHO_AM_I (BANK 0, 0x75) doit retourner 0x47 pour ICM-42688-P.
     * Si 0x00 ou autre → capteur absent ou SPI KO → bloquer firmware.
     */
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_WHO_AM_I);
    if (cfg != ICM42688_WHO_AM_I_VAL) {
        if (debug) {
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
        if (debug)
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
    
    if (debug)
        print_to_console("\n\rICM-42688-P : SOFT_RESET OK", sizeof("\n\rICM-42688-P : SOFT_RESET OK"));
    
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 3 : Configuration PWR_MGMT0 — Mode Low-Noise
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * PWR_MGMT0 (BANK 0, 0x4E) :
     * - GYRO_MODE  = 0b11 (Low-Noise)  → meilleur SNR, biais gyro plus stable
     * - ACCEL_MODE = 0b11 (Low-Noise)  → meilleur SNR vs Low-Power
     * - TEMP_DIS   = 1 (température enabled, utile pour compensation thermique)
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
        if (debug)
            print_to_console("\n\rREG_PWR_MGMT0 MISMATCH", sizeof("\n\rREG_PWR_MGMT0 MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rICM-42688-P : REG_PWR_MGMT0 OK (Low-Noise mode)", 
                           sizeof("\n\rICM-42688-P : REG_PWR_MGMT0 OK (Low-Noise mode)"));
    }
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 4 : Configuration GYRO_CONFIG0 — ±2000 dps, ODR 8 kHz
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * GYRO_CONFIG0 (BANK 0, 0x4F) :
     * - GYRO_FS_SEL = 0b000 → ±2000 dps (16.4 LSB/dps)
     * - GYRO_ODR    = 0b0011 → 8 kHz
     * 
     * Choix ±2000 dps : vitesses angulaires max ~500-700 dps en vol stable,
     * marge confortable + meilleure résolution que ±4000 dps.
     * ODR 8 kHz
     */
    cfg = ICM42688_GYRO_FS_SEL_2000DPS | ICM42688_GYRO_ODR_8kHz;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_CONFIG0, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_CONFIG0);
    if (cfg != (ICM42688_GYRO_FS_SEL_2000DPS | ICM42688_GYRO_ODR_8kHz)) {
        if (debug)
            print_to_console("\n\rREG_GYRO_CONFIG0 MISMATCH", sizeof("\n\rREG_GYRO_CONFIG0 MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rICM-42688-P : REG_GYRO_CONFIG0 OK (+/-2000dps, 8kHz)", 
                           sizeof("\n\rICM-42688-P : REG_GYRO_CONFIG0 OK (+/-2000dps, 8kHz)"));
    }
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 5 : Configuration ACCEL_CONFIG0 — ±16g, ODR 1 kHz
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * ACCEL_CONFIG0 (BANK 0, 0x50) :
     * - ACCEL_FS_SEL = 0b000 → ±16g (2048 LSB/g)
     * - ACCEL_ODR    = 0b0110 → 1 kHz
     * 
     * Choix ±16g : manoeuvres agressives peuvent atteindre 6-8g,
     * ±16g évite saturation (plus de marge que ±8g du MPU6000).
     */
    cfg = ICM42688_ACCEL_FS_SEL_16G | ICM42688_ACCEL_ODR_1kHz;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_ACCEL_CONFIG0, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_ACCEL_CONFIG0);
    if (cfg != (ICM42688_ACCEL_FS_SEL_16G | ICM42688_ACCEL_ODR_1kHz)) {
        if (debug)
            print_to_console("\n\rREG_ACCEL_CONFIG0 MISMATCH", sizeof("\n\rREG_ACCEL_CONFIG0 MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rICM-42688-P : REG_ACCEL_CONFIG0 OK (+/-16g, 1kHz)", 
                           sizeof("\n\rICM-42688-P : REG_ACCEL_CONFIG0 OK (+/-16g, 1kHz)"));
    }
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 6 : Configuration AAF (Anti-Aliasing Filter) gyro uniquement — BANK 1
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * L'AAF est un filtre notch programmable pour réjection harmoniques moteur.
     * Registres GYRO_CONFIG_STATIC2/3/4/5 (BANK 1) : AAF_DELT, AAF_DELTSQR, AAF_BITSHIFT.
     * 
     * Configuration recommandée TDK pour ODR 8kHz, notch ~200Hz (fondamentale moteur) :
     * - AAF_DELT = 63 (0x3F) → DELT[7:0]
     * - AAF_DELTSQR = 3969 (0x0F81) → DELTSQR[11:8] = 0x0F, DELTSQR[7:0] = 0x81
     * - AAF_BITSHIFT = 8 (0x08) → bits [7:4] = 8, bits [3:0] = 0
     * 
     * Ces valeurs sont issues de l'application note TDK "Gyroscope AAF Configuration".
     * Valeurs TDK app note AN-000157 pour f_notch = 200 Hz, ODR = 8 kHz : 
     * Pour cinelifter 940 Kv @ 15.5V : f_motor ≈ 130-200 Hz (2e harmonique 260-400 Hz) et blade passage (tri-blade × RPM) : 400–600 Hz (3 × f_motor).
     * Filtrage cascade prevu :
     * - AAF 200 Hz (hardware, fondamentale)
     * - Notch SW 400 Hz (software, blade passage)
     * - PT2 ~100 Hz (software, global LPF)
     * Avec 9050 tri-blade, la blade passage @ 400–600 Hz est brutale — l'AAF seul ne suffit pas, il faut le notch SW
     *
     * Note : l'AAF est activé automatiquement en mode Low-Noise si ces registres
     * sont non-nuls. Pas de bit enable séparé.
     */
    spi_write_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC2_B1, 0x3FU);  /* DELT[7:0] = 63 */
    LL_mDelay(10);
    spi_write_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC3_B1, 0x00U);  /* DELT[15:8] = 0 */
    LL_mDelay(10);
    spi_write_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC4_B1, 0x81U);  /* DELTSQR[7:0] = 0x81 (LSB de 3969) */
    LL_mDelay(10);
    spi_write_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC5_B1, 0x8FU);  /* Bits [7:4] = BITSHIFT = 8 → 0x80, bits [3:0] = DELTSQR[11:8] = 0x0F → combiné = 0x8F */
    LL_mDelay(10);
    
    /* Vérification lecture (BANK 1) que le premier registre est correct */
    cfg = spi_read_reg(ICM42688_BANK_SEL_1, ICM42688_REG_GYRO_CONFIG_STATIC2_B1);
    if (cfg != 0x3FU) {
        if (debug)
            print_to_console("\n\rREG_GYRO_CONFIG_STATIC2 (BANK1) MISMATCH", 
                           sizeof("\n\rREG_GYRO_CONFIG_STATIC2 (BANK1) MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rICM-42688-P : AAF gyro config OK (BANK 1, notch ~200Hz)", 
                           sizeof("\n\rICM-42688-P : AAF gyro config OK (BANK 1, notch ~200Hz)"));
    }

    /* AAF accel (BANK 2) — DÉSACTIVÉ (tout zéro) car UI filter suffit */
    spi_write_reg(ICM42688_BANK_SEL_2, ICM42688_REG_ACCEL_CONFIG_STATIC2_B2, 0x00U);
    LL_mDelay(10);
    spi_write_reg(ICM42688_BANK_SEL_2, ICM42688_REG_ACCEL_CONFIG_STATIC3_B2, 0x00U);
    LL_mDelay(10);
    spi_write_reg(ICM42688_BANK_SEL_2, ICM42688_REG_ACCEL_CONFIG_STATIC4_B2, 0x00U);
    LL_mDelay(10);
    
    if (debug)
        print_to_console("\n\rICM-42688-P : AAF accel disabled (BANK 2)", 
                       sizeof("\n\rICM-42688-P : AAF accel disabled (BANK 2)"));
    
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
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_CONFIG1, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_CONFIG1);
    if ((cfg & 0x0CU) != ICM42688_GYRO_UI_FILT_ORD_2ND) {
        if (debug)
            print_to_console("\n\rREG_GYRO_CONFIG1 MISMATCH", sizeof("\n\rREG_GYRO_CONFIG1 MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rICM-42688-P : REG_GYRO_CONFIG1 OK (UI filt 2nd order)", 
                           sizeof("\n\rICM-42688-P : REG_GYRO_CONFIG1 OK (UI filt 2nd order)"));
    }
    
    /*
     * GYRO_ACCEL_CONFIG0 (BANK 0, 0x52) :
     * - ACCEL_UI_FILT_ORD = 2nd order
     */
    cfg = ICM42688_ACCEL_UI_FILT_ORD_2ND;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_ACCEL_CONFIG0, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_GYRO_ACCEL_CONFIG0);
    if ((cfg & 0x18U) != ICM42688_ACCEL_UI_FILT_ORD_2ND) {
        if (debug)
            print_to_console("\n\rREG_GYRO_ACCEL_CONFIG0 MISMATCH", 
                           sizeof("\n\rREG_GYRO_ACCEL_CONFIG0 MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rICM-42688-P : REG_GYRO_ACCEL_CONFIG0 OK (accel UI filt 2nd)", 
                           sizeof("\n\rICM-42688-P : REG_GYRO_ACCEL_CONFIG0 OK (accel UI filt 2nd)"));
    }
    
    /*
     * ACCEL_CONFIG1 (BANK 0, 0x53) :
     * - ACCEL_UI_FILT_BW = ODR/20 → ~50 Hz @ ODR 1kHz
     * 
     * Choix BW ~50 Hz : filtre hardware avant notch software (250/500 Hz).
     * Empirique pour cinelifter : BW trop bas (<50 Hz) ajoute latence,
     * BW trop haut (>200 Hz) laisse passer vibrations moteur.
     */
    cfg = ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_20;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_ACCEL_CONFIG1, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_ACCEL_CONFIG1);
    if ((cfg & 0x0FU) != ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_20) {
        if (debug)
            print_to_console("\n\rREG_ACCEL_CONFIG1 MISMATCH", sizeof("\n\rREG_ACCEL_CONFIG1 MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rICM-42688-P : REG_ACCEL_CONFIG1 OK (UI BW ~50Hz)", 
                           sizeof("\n\rICM-42688-P : REG_ACCEL_CONFIG1 OK (UI BW ~50Hz)"));
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
        if (debug)
            print_to_console("\n\rREG_INT_CONFIG MISMATCH", sizeof("\n\rREG_INT_CONFIG MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rICM-42688-P : REG_INT_CONFIG OK (INT2 pulsed, PP, high)", 
                           sizeof("\n\rICM-42688-P : REG_INT_CONFIG OK (INT2 pulsed, PP, high)"));
    }
    
    /*
     * INT_CONFIG1 (BANK 0, 0x64) :
     * - INT_ASYNC_RESET = 1 (clear INT status sur n'importe quelle lecture registre)
     * - INT_TPULSE_DUR  = 0 (100 µs pulse, valeur par défaut)
     */
    cfg = ICM42688_INT_CONFIG1_INT_ASYNC_RESET;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INT_CONFIG1, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INT_CONFIG1);
    if (!(cfg & ICM42688_INT_CONFIG1_INT_ASYNC_RESET)) {
        if (debug)
            print_to_console("\n\rREG_INT_CONFIG1 MISMATCH", sizeof("\n\rREG_INT_CONFIG1 MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rICM-42688-P : REG_INT_CONFIG1 OK (async reset)", 
                           sizeof("\n\rICM-42688-P : REG_INT_CONFIG1 OK (async reset)"));
    }
    
    /*
     * INT_SOURCE0 (BANK 0, 0x65) :
     * - UI_DRDY_INT2_EN = 1 (router UI Data Ready → INT2 pin)
     * 
     * UI_DRDY se déclenche à l'ODR le plus lent (accel = 1KHz) configuré quand nouvelles données
     * gyro+accel disponibles dans les registres.
     */
    cfg = ICM42688_INT_SOURCE0_UI_DRDY_INT2_EN;
    spi_write_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INT_SOURCE0, cfg);
    LL_mDelay(10);
    
    cfg = spi_read_reg(ICM42688_BANK_SEL_0, ICM42688_REG_INT_SOURCE0);
    if (!(cfg & ICM42688_INT_SOURCE0_UI_DRDY_INT2_EN)) {
        if (debug)
            print_to_console("\n\rREG_INT_SOURCE0 MISMATCH", sizeof("\n\rREG_INT_SOURCE0 MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rICM-42688-P : REG_INT_SOURCE0 OK (UI_DRDY -> INT2)", 
                           sizeof("\n\rICM-42688-P : REG_INT_SOURCE0 OK (UI_DRDY -> INT2)"));
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
        if (debug)
            print_to_console("\n\rREG_INTF_CONFIG0 MISMATCH", sizeof("\n\rREG_INTF_CONFIG0 MISMATCH"));
        while(1);
    }
    else {
        if (debug)
            print_to_console("\n\rICM-42688-P : REG_INTF_CONFIG0 OK (big-endian)", 
                           sizeof("\n\rICM-42688-P : REG_INTF_CONFIG0 OK (big-endian)"));
    }
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 10 : Préparation buffers DMA (burst read 13 octets)
     * ══════════════════════════════════════════════════════════════════════ */
    /*
     * Burst : 1 adresse + 6 ACCEL + 6 GYRO = 13 octets
     * Adresse départ : ICM42688_REG_ACCEL_DATA_X1 (0x1F) → 12 lectures consécutives jusqu'à GYRO_DATA_Z0
     * 
     * tx_buf[0] = adresse read (bit7=1) : 0x1D | 0x80 = 0x9D
     * tx_buf[1..12] = dummy 0x00 pour générer clocks SPI
     */
    memset(dma_tx_buf, 0x00, ICM42688_BURST_LEN);
    dma_tx_buf[0] = ICM42688_REG_ACCEL_DATA_X1 | ICM42688_READ_FLAG;  /* 0x9D */
    
    /* Préparer DMA en mode Normal (réarmement manuel après chaque transfert) */
    LL_DMA_SetMemoryAddress(ICM42688_DMA, ICM42688_DMA_TX_STREAM, (uint32_t)dma_tx_buf);
    LL_DMA_SetPeriphAddress(ICM42688_DMA, ICM42688_DMA_TX_STREAM, (uint32_t)&SPI1->DR);
    LL_DMA_SetDataLength(ICM42688_DMA, ICM42688_DMA_TX_STREAM, ICM42688_BURST_LEN);
    
    LL_DMA_SetMemoryAddress(ICM42688_DMA, ICM42688_DMA_RX_STREAM, (uint32_t)dma_rx_buf);
    LL_DMA_SetPeriphAddress(ICM42688_DMA, ICM42688_DMA_RX_STREAM, (uint32_t)&SPI1->DR);
    LL_DMA_SetDataLength(ICM42688_DMA, ICM42688_DMA_RX_STREAM, ICM42688_BURST_LEN);
    
    /* Enable Transfer Complete interrupt sur RX stream uniquement */
    LL_DMA_EnableIT_TC(ICM42688_DMA, ICM42688_DMA_RX_STREAM);
    
    if (debug)
        print_to_console("\n\rICM-42688-P : DMA buffers prepared (burst 13 bytes)", 
                       sizeof("\n\rICM-42688-P : DMA buffers prepared (burst 13 bytes)"));
    
    /* ══════════════════════════════════════════════════════════════════════
     * ÉTAPE 12 : Initialisation terminée
     * ══════════════════════════════════════════════════════════════════════ */
    if (debug)
        print_to_console("\n\rICM-42688-P : Init complete, ready for fast loop @ 1kHz\n\r", 
                       sizeof("\n\rICM-42688-P : Init complete, ready for fast loop @ 1kHz\n\r"));

    /* ── 9. Configurer NVIC pour EXTI3 (INT DATA_RDY) ── */
    NVIC_EnableIRQ(EXTI3_IRQn);
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
    
    /* Recharger NDTR (compteurs) */
    LL_DMA_SetDataLength(ICM42688_DMA, ICM42688_DMA_TX_STREAM, ICM42688_BURST_LEN);
    LL_DMA_SetDataLength(ICM42688_DMA, ICM42688_DMA_RX_STREAM, ICM42688_BURST_LEN);
    
    /* Clear flags DMA (évite faux TC si flag resté levé) */
    LL_DMA_ClearFlag_TC0(ICM42688_DMA);   /* RX stream = Stream0 */
    LL_DMA_ClearFlag_TC3(ICM42688_DMA);   /* TX stream = Stream3 */
    LL_DMA_ClearFlag_TE0(ICM42688_DMA);
    LL_DMA_ClearFlag_TE3(ICM42688_DMA);
    
    /* Activer streams */
    LL_DMA_EnableStream(ICM42688_DMA, ICM42688_DMA_RX_STREAM);
    LL_DMA_EnableStream(ICM42688_DMA, ICM42688_DMA_TX_STREAM);
    
    /* CS LOW → démarrer transaction SPI */
    ICM42688_CS_LOW();
    
    /* Activer DMA requests SPI (TX puis RX) */
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_SPI_EnableDMAReq_RX(SPI1);
}
 
void ICM42688_DMA_RX_Complete_Callback(void)
{
    /*
     * Callback Transfer Complete DMA RX (Stream0).
     * Appelé depuis DMA2_Stream0_IRQHandler quand les 15 octets sont reçus.
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
    
    icm42688.ax = (float)icm42688.raw_ax * ICM42688_ACCEL_SCALE_16G_MPS2;
    icm42688.ay = (float)icm42688.raw_ay * ICM42688_ACCEL_SCALE_16G_MPS2;
    icm42688.az = (float)icm42688.raw_az * ICM42688_ACCEL_SCALE_16G_MPS2;
    
    /* Gyroscope (big-endian → MSB first) */
    icm42688.raw_gx = (int16_t)((dma_rx_buf[7]  << 8) | dma_rx_buf[8]);
    icm42688.raw_gy = (int16_t)((dma_rx_buf[9]  << 8) | dma_rx_buf[10]);
    icm42688.raw_gz = (int16_t)((dma_rx_buf[11] << 8) | dma_rx_buf[12]);
    
    icm42688.gx = (float)icm42688.raw_gx * ICM42688_GYRO_SCALE_2000DPS_RAD_S;
    icm42688.gy = (float)icm42688.raw_gy * ICM42688_GYRO_SCALE_2000DPS_RAD_S;
    icm42688.gz = (float)icm42688.raw_gz * ICM42688_GYRO_SCALE_2000DPS_RAD_S;
    
    /* ──────────────────────────────────────────────────────────────────────
     * Flag data_ready → la fast loop peut consommer gx/gy/gz
     * ────────────────────────────────────────────────────────────────────── */
    icm42688.data_ready = true;
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 * CALLBACKS INTERRUPTIONS
 * ═══════════════════════════════════════════════════════════════════════════ */
void ICM42688_EXTI3_Callback(void)
{
    /*
     * INT2 rising edge @ 1 kHz (UI_DRDY).
     * Déclenche immédiatement la lecture DMA.
     */
    ICM42688_Start_DMA_Read();
}
 
void ICM42688_DMA2_Stream0_Callback(void)
{
    /*
     * Wrapper pour appel depuis stm32f4xx_it.c (DMA2_Stream0_IRQHandler).
     */
    ICM42688_DMA_RX_Complete_Callback();
}