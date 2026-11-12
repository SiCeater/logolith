/**
 * debug.c
 * Console de debug sur UART5 (on n'utilise que le TX, broche DEBUG_TX / PC12),
 * 921600 bauds (CubeMX, usart.c). Régler le moniteur série à 921600.
 *
 * Fonctionnement : envoi en "polling", octet par octet, en attendant TXE (registre
 * de transmission vide). Simple et fiable, mais BLOQUANT : à 921600 bauds, un
 * caractère prend ~11 µs, soit ~0,5 ms pour une ligne de 50 caractères. Appelé depuis une
 * ISR (debug_imu), ça décale toutes les autres interruptions de même
 * priorité ou moins prioritaires. Exemple : une ligne IMU (~0,5 ms) représente 4
 * périodes de la boucle 8 kHz, d'où les prints IMU décimés à ~10 Hz.
 *
 * Pas de printf : avec newlib-nano, printf ne gère pas les flottants sans l'option
 * de link -u _printf_float (+~10 Ko). Les valeurs sont donc converties en
 * entiers mis à l'échelle (×10, ×100, ×1000) puis affichées en ASCII à la main.
 */
#include "debug.h"
#include "stm32f405xx.h"
#include <stdint.h>

void UART_Debug_Transmit_Char_LL(uint8_t c)
{
    while (!LL_USART_IsActiveFlag_TXE(UART5));
    LL_USART_TransmitData8(UART5, c);
}

/* Envoie `size` octets. Attend TC (fin de transmission) à la fin : au retour, le
 * dernier octet est réellement sorti sur la broche. */
void UART_Debug_Transmit_Buffer_LL(uint8_t *data, uint16_t size)
{
  for (uint16_t i = 0; i < size; i++)
  {
    // Attendre que le registre de transmission soit vide (TXE)
    while (!LL_USART_IsActiveFlag_TXE(UART5));
    // Écrire le caractère dans le registre de transmission
    LL_USART_TransmitData8(UART5, data[i]);
  }
  // Attendre que la transmission soit terminée (TC)
  while (!LL_USART_IsActiveFlag_TC(UART5));
}

/* ─── Debug UART5 — gyro en DPS au dixième, ASCII, no float ────────────── */
/*
 * Conversion : raw × 10 / 164  →  dps × 10  en int16
 * Exemple : raw = 820 → 820×10/164 = 50 → affiche "5.0"
 *           raw = -328 → -20 → affiche "-2.0"
 * Précision : ±0.06°/s (erreur arrondi entier / 164) — largement suffisant
 */

/* Envoie un int16 représentant dps×10 sous la forme [-]XXX.X */
void print_gyro_data(int16_t dps_x10)
{
    char buf[7];   /* max "-2000.0" = 7 chars */
    uint8_t len = 0;

    if (dps_x10 < 0) {
        buf[len++] = '-';
        dps_x10 = (int16_t)-dps_x10;
    }

    uint16_t intpart = (uint16_t)dps_x10 / 10U;
    uint8_t  decpart = (uint8_t)(dps_x10 % 10U);

    /* Partie entière (1 à 4 digits, jamais vide) */
    uint8_t start = len;
    if (intpart == 0U) {
        buf[len++] = '0';
    } else {
        while (intpart > 0U) {
            buf[len++] = (char)('0' + intpart % 10U);
            intpart /= 10U;
        }
        /* Inverser les digits (on les a écrits à l'envers) */
        uint8_t a = start, b = (uint8_t)(len - 1U);
        while (a < b) {
            char tmp = buf[a]; buf[a] = buf[b]; buf[b] = tmp;
            a++; b--;
        }
    }

    buf[len++] = '.';
    buf[len++] = (char)('0' + decpart);

    for (uint8_t i = 0U; i < len; i++) {
        UART_Debug_Transmit_Char_LL((uint8_t)buf[i]);
    }
}

/* ─── Debug UART5 — accel en g au dixième, ASCII, no float ─────────────── */
/*
 * Même mécanique que print_gyro_data.
 * Entrée : g×10 en int16  (ex: 10 → "1.0g",  -24 → "-2.4g")
 * Plage : ±16g → ±160 en entrée → max "-16.0g" = 6 chars, pile la taille de buf
 * (pas de '\0' : on envoie len caractères, ce n'est pas une chaîne C)
 */
void print_accel_data(int16_t g_x10)
{
    char buf[6];   /* max "-8.0" = 4 chars + 'g' + '\0' = 6 */
    uint8_t len = 0;
 
    if (g_x10 < 0) {
        buf[len++] = '-';
        g_x10 = (int16_t)-g_x10;
    }
 
    uint16_t intpart = (uint16_t)g_x10 / 10U;
    uint8_t  decpart  = (uint8_t)(g_x10 % 10U);
 
    uint8_t start = len;
    if (intpart == 0U) {
        buf[len++] = '0';
    } else {
        while (intpart > 0U) {
            buf[len++] = (char)('0' + intpart % 10U);
            intpart /= 10U;
        }
        uint8_t a = start, b = (uint8_t)(len - 1U);
        while (a < b) {
            char tmp = buf[a]; buf[a] = buf[b]; buf[b] = tmp;
            a++; b--;
        }
    }
 
    buf[len++] = '.';
    buf[len++] = (char)('0' + decpart);
    buf[len++] = 'g';
 
    for (uint8_t i = 0U; i < len; i++) {
        UART_Debug_Transmit_Char_LL((uint8_t)buf[i]);
    }
}

/* ─── Gyro en rad/s, 3 décimales ───────────────────────────────────────── */
/* Entrée : rad/s × 1000. int32_t car ±2000 dps = ±34,906 rad/s → ±34906, qui
 * déborde un int16 (±32767). Appelant : icm42688_driver.c (debug_imu). */
void print_gyro_rads(int32_t rad_x1000)
{
    char    buf[9];   /* "-34.906" = 7 chars max */
    uint8_t len = 0;

    if (rad_x1000 < 0) {
        buf[len++] = '-';
        rad_x1000 = -rad_x1000;
    }

    uint32_t intpart = (uint32_t)rad_x1000 / 1000U;
    uint16_t decpart = (uint16_t)(rad_x1000 % 1000U);  /* 3 chiffres */

    /* Partie entière */
    uint8_t start = len;
    if (intpart == 0U) {
        buf[len++] = '0';
    } else {
        while (intpart > 0U) {
            buf[len++] = (char)('0' + intpart % 10U);
            intpart /= 10U;
        }
        uint8_t a = start, b = (uint8_t)(len - 1U);
        while (a < b) { char t = buf[a]; buf[a]=buf[b]; buf[b]=t; a++; b--; }
    }

    /* Décimales sur 3 chiffres avec zéros de tête ("034" si 34) */
    buf[len++] = '.';
    buf[len++] = (char)('0' + (decpart / 100U));
    buf[len++] = (char)('0' + (decpart / 10U) % 10U);
    buf[len++] = (char)('0' + (decpart % 10U));

    for (uint8_t i = 0U; i < len; i++)
        UART_Debug_Transmit_Char_LL((uint8_t)buf[i]);
}

/* ─── Accel en m/s², 2 décimales ───────────────────────────────────────── */
/* Entrée : m/s² × 100. ±16 g = ±156,9 m/s² → ±15691, tient dans un int16. */
void print_accel_mps2(int16_t mps2_x100)
{
    char    buf[8];   /* "-78.48" = 6 chars max */
    uint8_t len = 0;

    if (mps2_x100 < 0) {
        buf[len++] = '-';
        mps2_x100 = (int16_t)-mps2_x100;
    }

    uint16_t intpart = (uint16_t)mps2_x100 / 100U;
    uint8_t  dec10   = (uint8_t)((mps2_x100 / 10U) % 10U);
    uint8_t  dec1    = (uint8_t)(mps2_x100 % 10U);

    uint8_t start = len;
    if (intpart == 0U) {
        buf[len++] = '0';
    } else {
        while (intpart > 0U) {
            buf[len++] = (char)('0' + intpart % 10U);
            intpart /= 10U;
        }
        uint8_t a = start, b = (uint8_t)(len - 1U);
        while (a < b) { char t = buf[a]; buf[a]=buf[b]; buf[b]=t; a++; b--; }
    }

    buf[len++] = '.';
    buf[len++] = (char)('0' + dec10);
    buf[len++] = (char)('0' + dec1);

    for (uint8_t i = 0U; i < len; i++)
        UART_Debug_Transmit_Char_LL((uint8_t)buf[i]);
}

/* ─── Debug UART5 — angles en degrés au dixième ────────────────────────── */
/*
 * Entrée : float en radians
 * Sortie : ASCII [-]XXX.X°  (ex: "45.3°", "-179.8°")
 * Conversion : rad × 572.957795 → deg×10 en int32
 * Plage : ±180° → ±1800 → max "-180.0°" = 7 chars
 */

void print_roll_deg(float roll_rad)
{
    char    buf[8];   /* max "-180.0°" = 7 chars + sécurité */
    uint8_t len = 0;

    /* Conversion rad → deg×10 */
    int32_t deg_x10 = (int32_t)(roll_rad * 572.957795f);

    if (deg_x10 < 0) {
        buf[len++] = '-';
        deg_x10 = -deg_x10;
    }

    uint32_t intpart = (uint32_t)deg_x10 / 10U;
    uint8_t  decpart = (uint8_t)(deg_x10 % 10U);

    /* Partie entière */
    uint8_t start = len;
    if (intpart == 0U) {
        buf[len++] = '0';
    } else {
        while (intpart > 0U) {
            buf[len++] = (char)('0' + intpart % 10U);
            intpart /= 10U;
        }
        uint8_t a = start, b = (uint8_t)(len - 1U);
        while (a < b) { char t = buf[a]; buf[a]=buf[b]; buf[b]=t; a++; b--; }
    }

    buf[len++] = '.';
    buf[len++] = (char)('0' + decpart);
    buf[len++] = 0xB0;  /* Symbole ° (degré) en ASCII étendu, ou utiliser 'd' si UTF-8 pose problème */

    for (uint8_t i = 0U; i < len; i++)
        UART_Debug_Transmit_Char_LL((uint8_t)buf[i]);
}

void print_pitch_deg(float pitch_rad)
{
    char    buf[8];
    uint8_t len = 0;

    int32_t deg_x10 = (int32_t)(pitch_rad * 572.957795f);

    if (deg_x10 < 0) {
        buf[len++] = '-';
        deg_x10 = -deg_x10;
    }

    uint32_t intpart = (uint32_t)deg_x10 / 10U;
    uint8_t  decpart = (uint8_t)(deg_x10 % 10U);

    uint8_t start = len;
    if (intpart == 0U) {
        buf[len++] = '0';
    } else {
        while (intpart > 0U) {
            buf[len++] = (char)('0' + intpart % 10U);
            intpart /= 10U;
        }
        uint8_t a = start, b = (uint8_t)(len - 1U);
        while (a < b) { char t = buf[a]; buf[a]=buf[b]; buf[b]=t; a++; b--; }
    }

    buf[len++] = '.';
    buf[len++] = (char)('0' + decpart);
    buf[len++] = 0xB0;

    for (uint8_t i = 0U; i < len; i++)
        UART_Debug_Transmit_Char_LL((uint8_t)buf[i]);
}

void print_yaw_deg(float yaw_rad)
{
    char    buf[8];
    uint8_t len = 0;

    int32_t deg_x10 = (int32_t)(yaw_rad * 572.957795f);

    if (deg_x10 < 0) {
        buf[len++] = '-';
        deg_x10 = -deg_x10;
    }

    uint32_t intpart = (uint32_t)deg_x10 / 10U;
    uint8_t  decpart = (uint8_t)(deg_x10 % 10U);

    uint8_t start = len;
    if (intpart == 0U) {
        buf[len++] = '0';
    } else {
        while (intpart > 0U) {
            buf[len++] = (char)('0' + intpart % 10U);
            intpart /= 10U;
        }
        uint8_t a = start, b = (uint8_t)(len - 1U);
        while (a < b) { char t = buf[a]; buf[a]=buf[b]; buf[b]=t; a++; b--; }
    }

    buf[len++] = '.';
    buf[len++] = (char)('0' + decpart);
    buf[len++] = 0xB0;

    for (uint8_t i = 0U; i < len; i++)
        UART_Debug_Transmit_Char_LL((uint8_t)buf[i]);
}

/* Point d'entrée générique pour les messages texte.
 * NB : beaucoup d'appelants passent sizeof("...") comme taille, ce qui envoie
 * aussi le '\0' final. C'est invisible dans la plupart des terminaux série. */
void print_to_console(char *buffer, uint16_t buffer_size)
{
  UART_Debug_Transmit_Buffer_LL((uint8_t *)buffer, buffer_size);
}