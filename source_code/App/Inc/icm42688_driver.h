/* icm42688_driver.h
 * Driver LL (Low-Layer) pour ICM-42688-P sur STM32F405
 * SPI1 + DMA2, fast loop 1kHz — GYRO (8KHz) + ACCEL (1KHz)
 *
 * Câblage :
 *   PC2  → ICM42688_SPI_CS   (GPIO Output)
 *   PC3  → ICM42688_INT2     (EXTI3, rising edge)
 *   PA5  → SPI1_SCLK         (AF5)
 *   PA6  → SPI1_MISO         (AF5)
 *   PA7  → SPI1_MOSI         (AF5)
 *   DMA2 Stream0 Ch3         → SPI1_RX
 *   DMA2 Stream3 Ch3         → SPI1_TX
 *
 * Notes ICM-42688-P vs MPU6000 :
 *   - Architecture multi-banques (BANK 0, 1, 2, 4) via REG_BANK_SEL
 *   - FIFO 2KB avancée (non utilisée ici, lecture directe registres)
 *   - AAF (Anti-Aliasing Filter) configurable pour SNR optimal
 *   - Mode Low-Noise pour gyro/accel (meilleur que MPU6000)
 *   - ODR ultra flexible (12.5 Hz à 32 kHz)
 *   - Sensibilité gyro meilleure : noise density ~0.004 dps/√Hz vs ~0.005 MPU6000
 */
 
#ifndef ICM42688_DRIVER_H
#define ICM42688_DRIVER_H
 
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "debug.h"
#include <stdint.h>
#include <stdbool.h>
#include "global.h"
#include <string.h>
#include "esc_driver.h"
#include <math.h>
 
/* ═══════════════════════════════════════════════════════════════════════════
 * REGISTRES ICM-42688-P — BANK 0 (registres de données et config rapide)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ICM42688_REG_DEVICE_CONFIG      0x11U   /* BANK 0 — SPI mode, soft reset */
#define ICM42688_REG_DRIVE_CONFIG       0x13U   /* BANK 0 — slew rate SPI */
#define ICM42688_REG_INT_CONFIG         0x14U   /* BANK 0 — INT1/INT2 config */
#define ICM42688_REG_FIFO_CONFIG        0x16U   /* BANK 0 — FIFO mode */
 
#define ICM42688_REG_TEMP_DATA1         0x1DU   /* BANK 0 — MSB température */
#define ICM42688_REG_TEMP_DATA0         0x1EU   /* BANK 0 — LSB température */
#define ICM42688_REG_ACCEL_DATA_X1      0x1FU   /* BANK 0 — Début burst accel */
#define ICM42688_REG_ACCEL_DATA_X0      0x20U
#define ICM42688_REG_ACCEL_DATA_Y1      0x21U
#define ICM42688_REG_ACCEL_DATA_Y0      0x22U
#define ICM42688_REG_ACCEL_DATA_Z1      0x23U
#define ICM42688_REG_ACCEL_DATA_Z0      0x24U
#define ICM42688_REG_GYRO_DATA_X1       0x25U   /* BANK 0 — Début burst gyro */
#define ICM42688_REG_GYRO_DATA_X0       0x26U
#define ICM42688_REG_GYRO_DATA_Y1       0x27U
#define ICM42688_REG_GYRO_DATA_Y0       0x28U
#define ICM42688_REG_GYRO_DATA_Z1       0x29U
#define ICM42688_REG_GYRO_DATA_Z0       0x2AU
 
#define ICM42688_REG_TMST_FSYNCH        0x2BU   /* BANK 0 — timestamp FSYNC MSB */
#define ICM42688_REG_TMST_FSYNCL        0x2CU   /* BANK 0 — timestamp FSYNC LSB */
#define ICM42688_REG_INT_STATUS         0x2DU   /* BANK 0 — status interruptions */
#define ICM42688_REG_FIFO_COUNTH        0x2EU   /* BANK 0 — FIFO count MSB */
#define ICM42688_REG_FIFO_COUNTL        0x2FU   /* BANK 0 — FIFO count LSB */
#define ICM42688_REG_FIFO_DATA          0x30U   /* BANK 0 — FIFO read port */
 
#define ICM42688_REG_APEX_DATA0         0x31U   /* BANK 0 — APEX feature data */
#define ICM42688_REG_APEX_DATA1         0x32U
#define ICM42688_REG_APEX_DATA2         0x33U
#define ICM42688_REG_APEX_DATA3         0x34U
#define ICM42688_REG_APEX_DATA4         0x35U
#define ICM42688_REG_APEX_DATA5         0x36U
 
#define ICM42688_REG_INT_STATUS2        0x37U   /* BANK 0 — status INT additionnel */
#define ICM42688_REG_INT_STATUS3        0x38U   /* BANK 0 — status FIFO */
 
#define ICM42688_REG_SIGNAL_PATH_RESET  0x4BU   /* BANK 0 — reset signal path */
#define ICM42688_REG_INTF_CONFIG0       0x4CU   /* BANK 0 — sensor data endian, FIFO count */
#define ICM42688_REG_INTF_CONFIG1       0x4DU   /* BANK 0 — mode AFSR (pour CLKIN) */
#define ICM42688_REG_PWR_MGMT0          0x4EU   /* BANK 0 — GYRO_MODE, ACCEL_MODE, TEMP_DIS */
#define ICM42688_REG_GYRO_CONFIG0       0x4FU   /* BANK 0 — GYRO_FS_SEL, GYRO_ODR */
#define ICM42688_REG_ACCEL_CONFIG0      0x50U   /* BANK 0 — ACCEL_FS_SEL, ACCEL_ODR */
#define ICM42688_REG_GYRO_CONFIG1       0x51U   /* BANK 0 — GYRO_UI_FILT_ORD, GYRO_DEC2_M2_ORD */
#define ICM42688_REG_GYRO_ACCEL_CONFIG0 0x52U   /* BANK 0 — ACCEL_UI_FILT_ORD, ACCEL_DEC2_M2_ORD */
#define ICM42688_REG_ACCEL_CONFIG1      0x53U   /* BANK 0 — ACCEL_UI_FILT_BW */
#define ICM42688_REG_GYRO_CONFIG_STATIC2 0x0BU  /* BANK 1 — AAF_DELT, AAF_DELTSQR, AAF_BITSHIFT */
#define ICM42688_REG_GYRO_CONFIG_STATIC3 0x0CU  /* BANK 1 — AAF_DELT (suite) */
#define ICM42688_REG_GYRO_CONFIG_STATIC4 0x0DU  /* BANK 1 — AAF_DELTSQR (suite) */
#define ICM42688_REG_GYRO_CONFIG_STATIC5 0x0EU  /* BANK 1 — AAF_BITSHIFT (suite) */
 
#define ICM42688_REG_TMST_CONFIG        0x54U   /* BANK 0 — timestamp config */
#define ICM42688_REG_APEX_CONFIG0       0x56U   /* BANK 0 — DMP power save */
#define ICM42688_REG_SMD_CONFIG         0x57U   /* BANK 0 — significant motion detect */
 
#define ICM42688_REG_FIFO_CONFIG1       0x5FU   /* BANK 0 — FIFO watermark, modes */
#define ICM42688_REG_FIFO_CONFIG2       0x60U   /* BANK 0 — FIFO WM MSB */
#define ICM42688_REG_FIFO_CONFIG3       0x61U   /* BANK 0 — FIFO WM LSB */
#define ICM42688_REG_FSYNC_CONFIG       0x62U   /* BANK 0 — FSYNC config */
#define ICM42688_REG_INT_CONFIG0        0x63U   /* BANK 0 — INT clear on read */
#define ICM42688_REG_INT_CONFIG1        0x64U   /* BANK 0 — INT async reset, pulse mode */
#define ICM42688_REG_INT_SOURCE0        0x65U   /* BANK 0 — INT source enable (UI_DRDY) */
#define ICM42688_REG_INT_SOURCE1        0x66U   /* BANK 0 — FIFO interrupts */
#define ICM42688_REG_INT_SOURCE3        0x68U   /* BANK 0 — APEX interrupts */
#define ICM42688_REG_INT_SOURCE4        0x69U   /* BANK 0 — additional INT sources */
 
#define ICM42688_REG_FIFO_LOST_PKT0     0x6CU   /* BANK 0 — FIFO lost packets LSB */
#define ICM42688_REG_FIFO_LOST_PKT1     0x6DU   /* BANK 0 — FIFO lost packets MSB */
 
#define ICM42688_REG_SELF_TEST_CONFIG   0x70U   /* BANK 0 — self-test enable */
#define ICM42688_REG_WHO_AM_I           0x75U   /* BANK 0 — device ID */
#define ICM42688_REG_REG_BANK_SEL       0x76U   /* BANK 0/1/2/4 — sélection banque */
 
/* ═══════════════════════════════════════════════════════════════════════════
 * REGISTRES BANK 1 (AAF gyro, self-test gyro)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ICM42688_REG_SENSOR_CONFIG0_B1  0x03U   /* BANK 1 — post AAF LPF gyro */
#define ICM42688_REG_GYRO_CONFIG_STATIC2_B1 0x0BU  /* BANK 1 — AAF gyro config */
#define ICM42688_REG_GYRO_CONFIG_STATIC3_B1 0x0CU
#define ICM42688_REG_GYRO_CONFIG_STATIC4_B1 0x0DU
#define ICM42688_REG_GYRO_CONFIG_STATIC5_B1 0x0EU
#define ICM42688_REG_XG_ST_DATA_B1      0x5FU   /* BANK 1 — self-test data gyro X */
#define ICM42688_REG_YG_ST_DATA_B1      0x60U
#define ICM42688_REG_ZG_ST_DATA_B1      0x61U
#define ICM42688_REG_TMSTVAL0_B1        0x62U   /* BANK 1 — timestamp value[7:0] */
#define ICM42688_REG_TMSTVAL1_B1        0x63U
#define ICM42688_REG_TMSTVAL2_B1        0x64U
#define ICM42688_REG_INTF_CONFIG4_B1    0x7AU   /* BANK 1 — AP interface config */
#define ICM42688_REG_INTF_CONFIG5_B1    0x7BU
#define ICM42688_REG_INTF_CONFIG6_B1    0x7CU
 
/* ═══════════════════════════════════════════════════════════════════════════
 * REGISTRES BANK 2 (AAF accel, self-test accel)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ICM42688_REG_ACCEL_CONFIG_STATIC2_B2 0x03U  /* BANK 2 — AAF accel config */
#define ICM42688_REG_ACCEL_CONFIG_STATIC3_B2 0x04U
#define ICM42688_REG_ACCEL_CONFIG_STATIC4_B2 0x05U
#define ICM42688_REG_XA_ST_DATA_B2      0x3BU   /* BANK 2 — self-test data accel X */
#define ICM42688_REG_YA_ST_DATA_B2      0x3CU
#define ICM42688_REG_ZA_ST_DATA_B2      0x3DU
 
/* ═══════════════════════════════════════════════════════════════════════════
 * REGISTRES BANK 4 (APEX, offset gyro/accel)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ICM42688_REG_GYRO_ON_OFF_CONFIG_B4  0x0EU   /* BANK 4 — gyro on/off time */
#define ICM42688_REG_APEX_CONFIG1_B4    0x40U   /* BANK 4 — DMP ODR config */
#define ICM42688_REG_APEX_CONFIG2_B4    0x41U
#define ICM42688_REG_APEX_CONFIG3_B4    0x42U
#define ICM42688_REG_APEX_CONFIG4_B4    0x43U
#define ICM42688_REG_APEX_CONFIG5_B4    0x44U
#define ICM42688_REG_APEX_CONFIG6_B4    0x45U
#define ICM42688_REG_APEX_CONFIG7_B4    0x46U
#define ICM42688_REG_APEX_CONFIG8_B4    0x47U
#define ICM42688_REG_APEX_CONFIG9_B4    0x48U
#define ICM42688_REG_ACCEL_WOM_X_THR_B4 0x4AU   /* BANK 4 — wake-on-motion threshold X */
#define ICM42688_REG_ACCEL_WOM_Y_THR_B4 0x4BU
#define ICM42688_REG_ACCEL_WOM_Z_THR_B4 0x4CU
#define ICM42688_REG_INT_SOURCE6_B4     0x4DU   /* BANK 4 — tilt, wake-on-motion INT */
#define ICM42688_REG_INT_SOURCE7_B4     0x4EU
#define ICM42688_REG_OFFSET_USER0_B4    0x77U   /* BANK 4 — offset utilisateur gyro X MSB */
#define ICM42688_REG_OFFSET_USER1_B4    0x78U
#define ICM42688_REG_OFFSET_USER2_B4    0x79U
#define ICM42688_REG_OFFSET_USER3_B4    0x7AU
#define ICM42688_REG_OFFSET_USER4_B4    0x7BU
#define ICM42688_REG_OFFSET_USER5_B4    0x7CU
#define ICM42688_REG_OFFSET_USER6_B4    0x7DU
#define ICM42688_REG_OFFSET_USER7_B4    0x7EU
#define ICM42688_REG_OFFSET_USER8_B4    0x7FU   /* BANK 4 — offset accel Z LSB */
 
/* ═══════════════════════════════════════════════════════════════════════════
 * VALEURS DE CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ICM42688_WHO_AM_I_VAL           0x47U   /* ICM-42688-P ID */
#define ICM42688_BANK_SEL_0             0x00U
#define ICM42688_BANK_SEL_1             0x01U
#define ICM42688_BANK_SEL_2             0x02U
#define ICM42688_BANK_SEL_4             0x04U
 
/* PWR_MGMT0 bits */
#define ICM42688_PWR_MGMT0_TEMP_DIS     (1U << 5)  /* Disable temperature sensor */
#define ICM42688_PWR_MGMT0_IDLE         (1U << 4)  /* IDLE mode (RC osc on, sensors off) */
#define ICM42688_PWR_MGMT0_GYRO_MODE_OFF    (0x00U << 2)
#define ICM42688_PWR_MGMT0_GYRO_MODE_STDBY  (0x01U << 2)
#define ICM42688_PWR_MGMT0_GYRO_MODE_LN     (0x03U << 2)  /* Low-Noise mode (best SNR) */
#define ICM42688_PWR_MGMT0_ACCEL_MODE_OFF   (0x00U << 0)
#define ICM42688_PWR_MGMT0_ACCEL_MODE_LP    (0x02U << 0)  /* Low-Power mode */
#define ICM42688_PWR_MGMT0_ACCEL_MODE_LN    (0x03U << 0)  /* Low-Noise mode (best SNR) */
 
/* GYRO_CONFIG0 — Full-Scale Range et ODR */
/*
 * Gyro FS pour drone cinelifter F330/F380 : ±2000 dps (comme MPU6000)
 * Justification : vitesses angulaires max ~500-700 dps en vol stable,
 * ±2000 dps donne marge confortable + meilleure résolution que ±4000 dps.
 */
#define ICM42688_GYRO_FS_SEL_2000DPS    (0x00U << 5)   /* ±2000 dps — 16.4 LSB/dps */
#define ICM42688_GYRO_FS_SEL_1000DPS    (0x01U << 5)   /* ±1000 dps — 32.8 LSB/dps */
#define ICM42688_GYRO_FS_SEL_500DPS     (0x02U << 5)   /* ±500 dps  — 65.5 LSB/dps */
#define ICM42688_GYRO_FS_SEL_250DPS     (0x03U << 5)   /* ±250 dps  — 131 LSB/dps */
#define ICM42688_GYRO_FS_SEL_125DPS     (0x04U << 5)   /* ±125 dps  — 262 LSB/dps */
#define ICM42688_GYRO_FS_SEL_62_5DPS    (0x05U << 5)   /* ±62.5 dps — 524.3 LSB/dps */
#define ICM42688_GYRO_FS_SEL_31_25DPS   (0x06U << 5)   /* ±31.25 dps — 1048.6 LSB/dps */
#define ICM42688_GYRO_FS_SEL_15_625DPS  (0x07U << 5)   /* ±15.625 dps — 2097.2 LSB/dps */
 
/* Gyro ODR — pour fast loop 1 kHz : ODR = 1 kHz */
#define ICM42688_GYRO_ODR_32kHz         0x01U   /* 32 kHz (DLPF contourne AAF) */
#define ICM42688_GYRO_ODR_16kHz         0x02U
#define ICM42688_GYRO_ODR_8kHz          0x03U
#define ICM42688_GYRO_ODR_4kHz          0x04U
#define ICM42688_GYRO_ODR_2kHz          0x05U
#define ICM42688_GYRO_ODR_1kHz          0x06U   /* ← 1 kHz — FAST LOOP DRONE */
#define ICM42688_GYRO_ODR_200Hz         0x07U
#define ICM42688_GYRO_ODR_100Hz         0x08U
#define ICM42688_GYRO_ODR_50Hz          0x09U
#define ICM42688_GYRO_ODR_25Hz          0x0AU
#define ICM42688_GYRO_ODR_12_5Hz        0x0BU
 
/* ACCEL_CONFIG0 — Full-Scale Range et ODR */
/*
 * Accel FS pour drone cinelifter : ±16g (marge > MPU6000 ±8g)
 * Justification : manoeuvres agressives peuvent atteindre 6-8g,
 * ±16g évite saturation + meilleure marge thermique.
 */
#define ICM42688_ACCEL_FS_SEL_16G       (0x00U << 5)   /* ±16g — 2048 LSB/g */
#define ICM42688_ACCEL_FS_SEL_8G        (0x01U << 5)   /* ±8g  — 4096 LSB/g */
#define ICM42688_ACCEL_FS_SEL_4G        (0x02U << 5)   /* ±4g  — 8192 LSB/g */
#define ICM42688_ACCEL_FS_SEL_2G        (0x03U << 5)   /* ±2g  — 16384 LSB/g */
 
/* Accel ODR — pour fast loop 1 kHz : ODR = 1 kHz */
#define ICM42688_ACCEL_ODR_32kHz        0x01U
#define ICM42688_ACCEL_ODR_16kHz        0x02U
#define ICM42688_ACCEL_ODR_8kHz         0x03U
#define ICM42688_ACCEL_ODR_4kHz         0x04U
#define ICM42688_ACCEL_ODR_2kHz         0x05U
#define ICM42688_ACCEL_ODR_1kHz         0x06U   /* ← 1 kHz — FAST LOOP DRONE */
#define ICM42688_ACCEL_ODR_200Hz        0x07U
#define ICM42688_ACCEL_ODR_100Hz        0x08U
#define ICM42688_ACCEL_ODR_50Hz         0x09U
#define ICM42688_ACCEL_ODR_25Hz         0x0AU
#define ICM42688_ACCEL_ODR_12_5Hz       0x0BU
#define ICM42688_ACCEL_ODR_6_25Hz       0x0CU   /* Low-power only */
#define ICM42688_ACCEL_ODR_3_125Hz      0x0DU
#define ICM42688_ACCEL_ODR_1_5625Hz     0x0EU
 
/* GYRO_CONFIG1 — UI Filter Order (post-AAF LPF) */
#define ICM42688_GYRO_UI_FILT_ORD_1ST   (0x00U << 2)   /* 1st order */
#define ICM42688_GYRO_UI_FILT_ORD_2ND   (0x01U << 2)   /* 2nd order */
#define ICM42688_GYRO_UI_FILT_ORD_3RD   (0x02U << 2)   /* 3rd order */
 
/* GYRO_ACCEL_CONFIG0 — UI Filter Order accel */
#define ICM42688_ACCEL_UI_FILT_ORD_1ST  (0x00U << 3)
#define ICM42688_ACCEL_UI_FILT_ORD_2ND  (0x01U << 3)
#define ICM42688_ACCEL_UI_FILT_ORD_3RD  (0x02U << 3)
 
/* ACCEL_CONFIG1 — UI Filter BW (post-AAF LPF bandwidth) */
/*
 * Pour drone cinelifter 1 kHz : BW ~50-100 Hz (filtre HW, avant notch SW)
 * Empirique : BW=4 → ~53 Hz @ ODR 1kHz (config TDK app note)
 */
#define ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_2   0x00U  /* ODR/2 (BW max) */
#define ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_4   0x01U  /* ODR/4 */
#define ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_5   0x02U
#define ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_8   0x03U
#define ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_10  0x04U  /* ← ~100 Hz @ ODR 1kHz */
#define ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_16  0x05U
#define ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_20  0x06U  /* ← ~50 Hz @ ODR 1kHz */
#define ICM42688_ACCEL_UI_FILT_BW_ODR_DIV_40  0x07U
 
/* INT_CONFIG bits (BANK 0, 0x14) */
#define ICM42688_INT_CONFIG_INT2_MODE_PULSED    (0x00U << 5)
#define ICM42688_INT_CONFIG_INT2_MODE_LATCHED   (0x01U << 5)
#define ICM42688_INT_CONFIG_INT2_DRIVE_OD       (0x00U << 4)  /* Open-drain */
#define ICM42688_INT_CONFIG_INT2_DRIVE_PP       (0x01U << 4)  /* Push-pull */
#define ICM42688_INT_CONFIG_INT2_POLARITY_LOW   (0x00U << 3)
#define ICM42688_INT_CONFIG_INT2_POLARITY_HIGH  (0x01U << 3)  /* Active-high (rising edge) */
#define ICM42688_INT_CONFIG_INT1_MODE_PULSED    (0x00U << 2)
#define ICM42688_INT_CONFIG_INT1_MODE_LATCHED   (0x01U << 2)
#define ICM42688_INT_CONFIG_INT1_DRIVE_OD       (0x00U << 1)
#define ICM42688_INT_CONFIG_INT1_DRIVE_PP       (0x01U << 1)
#define ICM42688_INT_CONFIG_INT1_POLARITY_LOW   (0x00U << 0)
#define ICM42688_INT_CONFIG_INT1_POLARITY_HIGH  (0x01U << 0)
 
/* INT_CONFIG1 bits (BANK 0, 0x64) */
#define ICM42688_INT_CONFIG1_INT_ASYNC_RESET    (1U << 4)  /* Clear INT status on any read */
#define ICM42688_INT_CONFIG1_INT_TPULSE_DUR     (0x00U << 6)  /* 100 µs pulse */
#define ICM42688_INT_CONFIG1_INT_TDEASSERT      (0x00U << 5)  /* Deassert disable */
 
/* INT_SOURCE0 bits (BANK 0, 0x65) — UI Data Ready routing */
#define ICM42688_INT_SOURCE0_UI_DRDY_INT1_EN    (1U << 3)  /* Route UI DRDY → INT1 */
#define ICM42688_INT_SOURCE0_UI_DRDY_INT2_EN    (1U << 0)  /* Route UI DRDY → INT2 */
 
/* DEVICE_CONFIG bits (BANK 0, 0x11) */
#define ICM42688_DEVICE_CONFIG_SOFT_RESET_CONFIG (1U << 0)  /* Soft reset */
 
/* SIGNAL_PATH_RESET bits (BANK 0, 0x4B) */
#define ICM42688_SIGNAL_PATH_RESET_DMP_INIT_EN      (1U << 6)
#define ICM42688_SIGNAL_PATH_RESET_DMP_MEM_RESET_EN (1U << 5)
#define ICM42688_SIGNAL_PATH_RESET_ABORT_AND_RESET  (1U << 3)
#define ICM42688_SIGNAL_PATH_RESET_TMST_STROBE      (1U << 2)
#define ICM42688_SIGNAL_PATH_RESET_FIFO_FLUSH       (1U << 1)
 
/* INTF_CONFIG0 bits (BANK 0, 0x4C) */
#define ICM42688_INTF_CONFIG0_FIFO_COUNT_REC    (0x00U << 6)  /* Records mode */
#define ICM42688_INTF_CONFIG0_FIFO_COUNT_BYTE   (0x01U << 6)  /* Bytes mode */
#define ICM42688_INTF_CONFIG0_FIFO_COUNT_ENDIAN_LITTLE (0x00U << 5)
#define ICM42688_INTF_CONFIG0_FIFO_COUNT_ENDIAN_BIG    (0x01U << 5)
#define ICM42688_INTF_CONFIG0_SENSOR_DATA_ENDIAN_LITTLE (0x00U << 4)
#define ICM42688_INTF_CONFIG0_SENSOR_DATA_ENDIAN_BIG    (0x01U << 4)  /* Big-endian (MSB first) */
 
/* ═══════════════════════════════════════════════════════════════════════════
 * CONSTANTES PHYSIQUES — Conversion LSB → unités SI
 * ═══════════════════════════════════════════════════════════════════════════ */
/*
 * Gyro ±2000 dps : sensibilité = 16.4 LSB/dps (datasheet ICM-42688-P table 4)
 * Conversion → rad/s : (1 / 16.4) × (π / 180)
 */
#define ICM42688_GYRO_SCALE_2000DPS_RAD_S   (1.0f / 16.4f * 0.01745329252f)
 
/*
 * Accel ±16g : sensibilité = 2048 LSB/g (datasheet table 3)
 * Conversion → m/s² : (1 / 2048) × 9.80665
 */
#define ICM42688_ACCEL_SCALE_16G_MPS2       (9.80665f / 2048.0f)
 
/* ═══════════════════════════════════════════════════════════════════════════
 * SPI et GPIO
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ICM42688_READ_FLAG              0x80U   /* Bit 7 = 1 en lecture SPI */
#define ICM42688_CS_PORT                GPIOC
#define ICM42688_CS_PIN                 LL_GPIO_PIN_2
 
/* DMA */
#define ICM42688_DMA                    DMA2
#define ICM42688_DMA_RX_STREAM          LL_DMA_STREAM_0
#define ICM42688_DMA_TX_STREAM          LL_DMA_STREAM_3
#define ICM42688_DMA_CHANNEL            LL_DMA_CHANNEL_3
 
/*
 * Taille burst lecture registres de données (BANK 0) :
 * 1 adresse + 6 ACCEL + 6 GYRO = 13 octets
 * (identique MPU6000, compatible avec architecture existante)
 * Temps SPI @ 10.5 MHz : 13×8 / 10.5e6 ≈ 10.2 µs — OK pour 1 kHz
 */
#define ICM42688_BURST_LEN              13U
 
/* ═══════════════════════════════════════════════════════════════════════════
 * STRUCTURE DE DONNÉES
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    /* Valeurs converties (rad/s pour gyro, m/s² pour accel) */
    float    gx;
    float    gy;
    float    gz;
 
    float    ax;
    float    ay;
    float    az;
 
    /* Bruts signés (debug, calibration, logging) */
    int16_t  raw_gx;
    int16_t  raw_gy;
    int16_t  raw_gz;
 
    int16_t  raw_ax;
    int16_t  raw_ay;
    int16_t  raw_az;
 
    volatile bool data_ready;
} icm42688_data_t;
 
/* Instance globale */
extern icm42688_data_t icm42688;
 
/* ═══════════════════════════════════════════════════════════════════════════
 * API PUBLIQUE
 * ═══════════════════════════════════════════════════════════════════════════ */
 
/**
 * @brief  Initialise l'ICM-42688-P : reset, WHO_AM_I, config registres multi-banques,
 *         activation AAF, préparation DMA. À appeler une fois dans main() après
 *         MX_SPI1_Init() et MX_DMA_Init().
 * @note   Séquence complète :
 *         1. Soft reset (DEVICE_CONFIG)
 *         2. Vérification WHO_AM_I (0x47)
 *         3. Config PWR_MGMT0 : Low-Noise mode gyro + accel
 *         4. Config GYRO_CONFIG0 : ±2000 dps, ODR 8 kHz
 *         5. Config ACCEL_CONFIG0 : ±16g, ODR 1 kHz
 *         6. Config filtres UI (GYRO_CONFIG1, ACCEL_CONFIG1)
 *         7. Config INT2 : push-pull, active-high, pulsed, UI_DRDY routing
 *         8. Lecture test registres pour vérifier config
 * @retval Aucune (bloque en while(1) si échec détection ou config)
 */
void ICM42688_Init(void);
 
/**
 * @brief  Lance une transaction DMA SPI1 non-bloquante (13 octets burst).
 *         Relance le burst RX en réarmant les streams DMA (mode Normal).
 *         Appelé depuis ICM42688_EXTI3_Callback() (INT2 rising edge @ 1 kHz).
 */
void ICM42688_Start_DMA_Read(void);
 
/**
 * @brief  Callback Transfer Complete DMA RX (Stream0).
 *         À appeler depuis DMA2_Stream0_IRQHandler dans stm32f4xx_it.c.
 *         Désactive les DMA req SPI → CS HIGH → parse raw → rad/s → data_ready.
 */
void ICM42688_DMA_RX_Complete_Callback(void);
 
/**
 * @brief  Callback EXTI3 (flanc montant INT2 de l'ICM-42688-P @ 1kHz).
 *         À appeler depuis EXTI3_IRQHandler dans stm32f4xx_it.c.
 */
void ICM42688_EXTI3_Callback(void);
 
/**
 * @brief  Wrapper pour callback DMA2 Stream0 (compatibilité stm32f4xx_it.c).
 */
void ICM42688_DMA2_Stream0_Callback(void);
 
#endif /* ICM42688_DRIVER_H */