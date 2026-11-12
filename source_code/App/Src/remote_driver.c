#include "remote_driver.h"
#include "global.h"
#include "stm32f405xx.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_usart.h"
#include <string.h>
#include <math.h>
#include "debug.h"

/* ==========================================================================
 * Variables globales (définition unique)
 * ========================================================================== */

static uint8_t   crsf_dma_buf[CRSF_DMA_BUF_SIZE];   // buffer circulaire DMA UART
static uint32_t  crsf_dma_last_pos;                  // position dernière lecture

crsf_parser_t       crsf_parser;
crsf_rc_t           crsf_rc;
crsf_link_stats_t   crsf_link;
crsf_device_info_t  crsf_device;

rc_axes_t           rc_axes;
rc_switches_t       rc_switches;
rc_failsafe_t       rc_failsafe;
uint16_t            rc_ch[16];      // snapshot des canaux bruts (172-1811), mis à jour par Remote_Process

/* Table CRC8 — générée une fois au boot (256*8 opérations, négligeable) */
static uint8_t crsf_crc8_tab[256];

/* États des filtres PT1 par axe (privés au module) */
typedef struct { float y; } pt1_state_t;
static pt1_state_t lpf_roll, lpf_pitch, lpf_throttle, lpf_yaw;

/* ==========================================================================
 * Init DMA — pointe le DMA1 Stream5 vers buffer applicatif
 * ========================================================================== */

static void Remote_UART_StartDMA(uint8_t *buffer, uint16_t size)
{
    /* NOTE : horloge, GPIO AF, baudrate USART2 et mode CIRCULAIRE du DMA
     * sont supposés déjà configurés par CubeMX (MX_USART2_UART_Init /
     * MX_DMA_Init). Vérifier impérativement dans CubeMX que le DMA RX de
     * USART2 est en mode Circular — sinon toute la logique de wrap-around
     * ci-dessous casse après le premier tour de buffer. */

    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_5);   // sécurité si déjà armé ailleurs

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)&USART2->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, size);

    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
    LL_USART_EnableDMAReq_RX(USART2);
}

/* ==========================================================================
 * CRC8 — poly 0xD5 (DVB-S2), table générée au boot
 * ========================================================================== */

static void crsf_crc8_init_table(void)
{
    for (uint16_t i = 0; i < 256; i++) {
        uint8_t crc = (uint8_t)i;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ CRSF_CRC_POLY)
                                : (uint8_t)(crc << 1);
        crsf_crc8_tab[i] = crc;
    }
}

uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
        crc = crsf_crc8_tab[crc ^ data[i]];   // lookup, O(1)/octet, safe en ISR
    return crc;
}

/* ==========================================================================
 * Décodage bit-packing RC_CHANNELS_PACKED (0x16)
 * 16 canaux x 11 bits = 176 bits = 22 octets, LSB-first
 * ========================================================================== */

void crsf_unpack_channels(const uint8_t *payload, uint16_t out[16])
{
    out[0]  = (payload[0]       | payload[1]  << 8) & 0x7FF;
    out[1]  = (payload[1] >> 3  | payload[2]  << 5) & 0x7FF;
    out[2]  = (payload[2] >> 6  | payload[3]  << 2 | payload[4] << 10) & 0x7FF;
    out[3]  = (payload[4] >> 1  | payload[5]  << 7) & 0x7FF;
    out[4]  = (payload[5] >> 4  | payload[6]  << 4) & 0x7FF;
    out[5]  = (payload[6] >> 7  | payload[7]  << 1 | payload[8] << 9)  & 0x7FF;
    out[6]  = (payload[8] >> 2  | payload[9]  << 6) & 0x7FF;
    out[7]  = (payload[9] >> 5  | payload[10] << 3) & 0x7FF;
    out[8]  = (payload[11]      | payload[12] << 8) & 0x7FF;
    out[9]  = (payload[12] >> 3 | payload[13] << 5) & 0x7FF;
    out[10] = (payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x7FF;
    out[11] = (payload[15] >> 1 | payload[16] << 7) & 0x7FF;
    out[12] = (payload[16] >> 4 | payload[17] << 4) & 0x7FF;
    out[13] = (payload[17] >> 7 | payload[18] << 1 | payload[19] << 9)  & 0x7FF;
    out[14] = (payload[19] >> 2 | payload[20] << 6) & 0x7FF;
    out[15] = (payload[20] >> 5 | payload[21] << 3) & 0x7FF;
}

/* ==========================================================================
 * Fonctions de debug UART5 — affichage console, ASCII, pas de float
 * ========================================================================== */

 /* Convertit un uint16_t (0-1811) en ASCII décimal, retourne nb caractères écrits */
static uint8_t uint16_to_dec(uint16_t val, char *out)
{
    char tmp[5];
    uint8_t n = 0;

    if (val == 0) {
        out[0] = '0';
        return 1;
    }

    while (val > 0) {
        tmp[n++] = '0' + (val % 10);
        val /= 10;
    }

    for (uint8_t i = 0; i < n; i++)
        out[i] = tmp[n - 1 - i];

    return n;
}

/* ── Debug RX (debug_rc), même principe que debug_imu ──────────────────────
 * Appelé depuis Remote_Process (boucle principale ~500 Hz), JAMAIS depuis l'ISR :
 * avant, chaque trame (jusqu'à 500-1000/s) était imprimée dans l'IRQ USART2, soit
 * ~0,8 ms de print bloquant par trame → console saturée et IRQ monopolisée.
 * Ici : une ligne toutes les CRSF_DEBUG_PERIOD_MS (5 Hz), format :
 *   "RC LQ:100 THR:0% CH: 992 992 172 992 1811 172 172 1811 992 992 992 992"
 *   (12 premiers canaux bruts 172-1811, CH1 en premier ; LQ = link quality %)
 * ou "RC: pas de lien (failsafe)" si aucune trame valide depuis 100 ms. */
#define CRSF_DEBUG_PERIOD_MS   200U
#define CRSF_DEBUG_NB_CH       12U     /* la radio n'envoie que 12 canaux utiles */

static void crsf_debug_print(const uint16_t ch[16], bool link_ok, uint32_t now)
{
    static uint32_t last_print_ms = 0;
    if ((now - last_print_ms) < CRSF_DEBUG_PERIOD_MS) return;
    last_print_ms = now;

    if (!link_ok) {
        static const char msg[] = "RC: pas de lien (failsafe)\r\n";
        UART_Debug_Transmit_Buffer_LL((uint8_t *)msg, sizeof(msg) - 1U);   /* -1 : sans le NUL */
        return;
    }

    char line[16 + 12 + 6 + CRSF_DEBUG_NB_CH * 5U + 2U];
    uint16_t pos = 0;

    memcpy(&line[pos], "RC LQ:", 6); pos += 6;
    pos += uint16_to_dec(crsf_link.uplink_lq, &line[pos]);   /* 0 tant qu'aucune trame LINK_STATISTICS */

    memcpy(&line[pos], " THR:", 5); pos += 5;
    pos += uint16_to_dec((uint16_t)(rc_axes.throttle * 100.0f + 0.5f), &line[pos]);
    line[pos++] = '%';

    memcpy(&line[pos], " CH:", 4); pos += 4;
    for (uint8_t i = 0; i < CRSF_DEBUG_NB_CH; i++) {
        line[pos++] = ' ';
        pos += uint16_to_dec(ch[i], &line[pos]);
    }
    line[pos++] = '\r'; line[pos++] = '\n';

    UART_Debug_Transmit_Buffer_LL((uint8_t *)line, pos);
}

/* ==========================================================================
 * State machine de parsing — octet par octet
 * ========================================================================== */

void crsf_parse_byte(crsf_parser_t *p, uint8_t byte)
{
    /* Timeout resync : si >5 ms sans octet, on a perdu le fil (glitch RF, boot) */
    uint32_t now = get_ms();
    if (now - p->last_rx_ms > 5) p->state = CRSF_STATE_WAIT_SYNC;
    p->last_rx_ms = now;

    switch (p->state) {

    case CRSF_STATE_WAIT_SYNC:
        if (byte == CRSF_ADDR_FLIGHT_CONTROLLER) {
            p->frame[0] = byte;
            p->state = CRSF_STATE_LEN;
        }
        /* sinon : octet ignoré, on continue à scanner (absorbe trames
         * partielles au boot, resync auto sur le prochain 0xC8 valide) */
        break;

    case CRSF_STATE_LEN:
        if (byte < 2 || byte > (CRSF_FRAME_MAX_SIZE - 2)) {
            p->state = CRSF_STATE_WAIT_SYNC;
            break;
        }
        p->frame_len = byte;
        p->frame[1]  = byte;
        p->idx       = 2;
        p->state     = CRSF_STATE_PAYLOAD;
        break;

    case CRSF_STATE_PAYLOAD:
        p->frame[p->idx++] = byte;
        if (p->idx == (uint8_t)(p->frame_len + 2)) {
            uint8_t crc = crsf_crc8(&p->frame[2], p->frame_len - 1);
            if (crc == p->frame[p->idx - 1]) {

                crsf_dispatch_frame(p->frame);
            /* CRC KO -> frame droppée silencieusement, le watchdog
             * failsafe applicatif absorbe la perte de ce cycle */
            }
            p->state = CRSF_STATE_WAIT_SYNC;
        }
        break;
    }
}

/* ==========================================================================
 * Dispatch par TYPE — exécuté dans le contexte de l'IDLE IRQ, rester court
 * ========================================================================== */

void crsf_dispatch_frame(const uint8_t *frame)
{
    uint8_t type          = frame[2];
    const uint8_t *payload = &frame[3];

    switch (type) {

    case CRSF_FRAMETYPE_RC_CHANNELS:
        /* LEN = TYPE(1) + 22 octets canaux + CRC(1) : on refuse toute autre taille */
        if (frame[1] != CRSF_RC_CHANNELS_LEN) break;
        crsf_unpack_channels(payload, crsf_rc.ch);
        crsf_rc.last_update_ms       = get_ms();
        crsf_rc.first_frame_received = true;
        crsf_rc.valid                = true;

        break;

    case CRSF_FRAMETYPE_LINK_STATISTICS:
        crsf_link.uplink_rssi_ant1 = payload[0];
        crsf_link.uplink_rssi_ant2 = payload[1];
        crsf_link.uplink_lq        = payload[2];
        crsf_link.uplink_snr       = (int8_t)payload[3];
        crsf_link.active_antenna   = payload[4];
        crsf_link.rf_mode          = payload[5];
        crsf_link.uplink_tx_power  = payload[6];
        crsf_link.downlink_rssi    = payload[7];
        crsf_link.downlink_lq      = payload[8];
        crsf_link.downlink_snr     = (int8_t)payload[9];
        crsf_link.last_update_ms   = get_ms();
        crsf_link.valid            = true;
        break;

    case CRSF_FRAMETYPE_DEVICE_PING:
        /* Le récepteur ELRS peut "pinger" le bus au boot. On ne répond pas :
         * USART2 est configuré TX_RX dans CubeMX, mais le driver n'émet jamais
         * (pas de télémétrie vers la radio pour l'instant). Répondre DEVICE_INFO
         * demanderait d'écrire sur RC_TX (PA2). */
        break;

    case CRSF_FRAMETYPE_DEVICE_INFO: {
        /* Frame étendue : payload[0]=dest, payload[1]=orig, puis nom
         * ASCII null-terminated. On tronque à la taille de crsf_device.name. */
        const uint8_t *info = payload + 2;
        size_t max_len = sizeof(crsf_device.name) - 1;
        size_t i = 0;
        while (i < max_len && info[i] != '\0') {
            crsf_device.name[i] = (char)info[i];
            i++;
        }
        crsf_device.name[i] = '\0';
        crsf_device.received = true;
        break;
    }

    default:
        /* Type connu du protocole mais non exploité ici, ou inconnu :
         * la frame a déjà été correctement consommée via LEN, on ignore. */
        break;
    }
}

/* ==========================================================================
 * Handler IDLE — à appeler depuis USART2_IRQHandler()
 * ========================================================================== */

void Remote_UART_IdleHandler(void)
{
    if (!LL_USART_IsActiveFlag_IDLE(USART2) || !LL_USART_IsEnabledIT_IDLE(USART2))
        return;

    LL_USART_ClearFlag_IDLE(USART2);   // lecture SR puis DR gérée en interne par LL

    uint32_t dma_head = CRSF_DMA_BUF_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);

    uint32_t received;
    if (dma_head >= crsf_dma_last_pos)
        received = dma_head - crsf_dma_last_pos;
    else
        received = (CRSF_DMA_BUF_SIZE - crsf_dma_last_pos) + dma_head;  // wrap

    for (uint32_t i = 0; i < received; i++) {
        uint32_t pos = (crsf_dma_last_pos + i) & (CRSF_DMA_BUF_SIZE - 1); // masque AND, buf = puissance de 2
        crsf_parse_byte(&crsf_parser, crsf_dma_buf[pos]);
    }

    crsf_dma_last_pos = dma_head;
}

/* ==========================================================================
 * Couche applicative — normalisation, deadzone, expo, filtrage
 * ========================================================================== */

static float apply_deadzone(float val, float dz)
{
    return (fabsf(val) < dz) ? 0.0f : val;
}

static float apply_expo(float val, float expo)
{
    return val * (1.0f - expo) + val * fabsf(val) * expo;
}

static float rc_normalize_centered(uint16_t raw)
{
    float v = (float)((int32_t)raw - CRSF_CHANNEL_VALUE_MID) /
              (float)(CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID);
    if (v > 1.0f)  v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return v;
}

static float rc_normalize_throttle(uint16_t raw)
{
    float v = (float)((int32_t)raw - CRSF_CHANNEL_VALUE_MIN) /
              (float)(CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN);
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

static bool rc_decode_switch_2pos(uint16_t ch)
{
    return ch > CRSF_CHANNEL_VALUE_MID;
}

static flight_mode_t rc_decode_switch_3pos(uint16_t ch)
{
    if (ch < 500)  return FLIGHT_MODE_ACRO;
    if (ch < 1400) return FLIGHT_MODE_ANGLE;
    return FLIGHT_MODE_ALTHOLD;
}

static float pt1_update(pt1_state_t *s, float x, float dt, float fc)
{
    float alpha = dt / (dt + 1.0f / (6.283185307f * fc));  // 2*pi*fc
    s->y += alpha * (x - s->y);
    return s->y;
}

bool rc_is_valid(void)
{
    /* last_update_ms est écrit par l'IRQ USART2 : le lire AVANT get_ms(),
     * sinon une trame arrivée entre les deux donne last > now et la
     * soustraction non signée explose (~4e9 ms) -> faux failsafe. */
    uint32_t last = crsf_rc.last_update_ms;
    return crsf_rc.first_frame_received &&
           (get_ms() - last) < CRSF_FAILSAFE_TIMEOUT_MS;
}

void Remote_Process(float dt)
{
    /* Snapshot cohérent des canaux : l'IRQ USART2 peut réécrire ch[] en
     * plein milieu de la lecture (canaux de deux trames différentes). */
    uint16_t ch[16];
    __disable_irq();
    uint32_t last = crsf_rc.last_update_ms;
    bool     got  = crsf_rc.first_frame_received;
    memcpy(ch, (const void *)crsf_rc.ch, sizeof(ch));
    __enable_irq();

    uint32_t now = get_ms();   // lu APRÈS last (cf. rc_is_valid)

    /* --- 1. Failsafe : gate absolue avant tout traitement --- */
    /* Même seuil que rc_is_valid() (>= ici, < là-bas) : les deux sont toujours d'accord. */
    if (!got || (now - last) >= CRSF_FAILSAFE_TIMEOUT_MS) {

        if (rc_failsafe.state != FAILSAFE_ACTIVE) {
            rc_failsafe.active_since_ms = now;
            rc_failsafe.count++;
        }
        rc_failsafe.state     = FAILSAFE_ACTIVE;
        rc_switches.armed     = false;  // sinon l'état armé reste figé à sa dernière valeur
        rc_axes.throttle      = 0.0f;
        rc_axes.throttle_filt = 0.0f;   // coupe le throttle, ne pas toucher roll/pitch
        lpf_throttle.y        = 0.0f;   // repart de 0 au retour du lien (pas de saut)
        if (debug_rc) crsf_debug_print(ch, false, now);
        return;
    }

    rc_failsafe.state = (crsf_link.valid && crsf_link.uplink_lq < 50)
                       ? FAILSAFE_WARN : FAILSAFE_OK;

    memcpy(rc_ch, ch, sizeof(rc_ch));   // valeurs brutes, uniquement quand le lien est OK

    /* --- 2. Switches --- */
    rc_switches.armed       = rc_decode_switch_2pos(ch[4]);
    rc_switches.flight_mode = rc_decode_switch_3pos(ch[5]);
    rc_switches.beeper      = rc_decode_switch_2pos(ch[6]);
    rc_switches.blackbox    = rc_decode_switch_2pos(ch[7]);
    for (int i = 0; i < 8; i++)
        rc_switches.aux[i] = ch[8 + i];

    /* --- 3. Axes : normalisation --- */
    float roll  = rc_normalize_centered(ch[0]);
    float pitch = rc_normalize_centered(ch[1]);
    float thr   = rc_normalize_throttle(ch[2]);
    float yaw   = rc_normalize_centered(ch[3]);

    /* --- 4. Deadzone + expo (pas sur le throttle) --- */
    roll  = apply_expo(apply_deadzone(roll,  CRSF_RC_DEADZONE), CRSF_RC_EXPO_RATE);
    pitch = apply_expo(apply_deadzone(pitch, CRSF_RC_DEADZONE), CRSF_RC_EXPO_RATE);
    yaw   = apply_expo(apply_deadzone(yaw,   CRSF_RC_DEADZONE), CRSF_RC_EXPO_RATE);

    rc_axes.roll = roll; rc_axes.pitch = pitch;
    rc_axes.throttle = thr; rc_axes.yaw = yaw;

    /* --- 5. Filtrage PT1 --- */
    rc_axes.roll_filt     = pt1_update(&lpf_roll,     roll,  dt, CRSF_RC_LPF_HZ);
    rc_axes.pitch_filt    = pt1_update(&lpf_pitch,    pitch, dt, CRSF_RC_LPF_HZ);
    rc_axes.throttle_filt = pt1_update(&lpf_throttle, thr,   dt, CRSF_RC_LPF_HZ);
    rc_axes.yaw_filt      = pt1_update(&lpf_yaw,      yaw,   dt, CRSF_RC_LPF_HZ);

    if (debug_rc) crsf_debug_print(ch, true, now);
}

/* ==========================================================================
 * Init driver
 * ========================================================================== */

void Remote_Init(void)
{
    crsf_crc8_init_table();

    memset(&crsf_parser, 0, sizeof(crsf_parser));
    memset(&crsf_rc,     0, sizeof(crsf_rc));
    memset(&crsf_link,   0, sizeof(crsf_link));
    memset(&crsf_device, 0, sizeof(crsf_device));
    memset(&rc_axes,     0, sizeof(rc_axes));
    memset(&rc_switches, 0, sizeof(rc_switches));

    crsf_dma_last_pos    = 0;
    rc_failsafe.state    = FAILSAFE_ACTIVE;  // pas de trame reçue au boot -> sécurité par défaut

    Remote_UART_StartDMA(crsf_dma_buf, CRSF_DMA_BUF_SIZE);

    LL_USART_EnableIT_IDLE(USART2);
    /* NVIC : vérifier que USART2_IRQn est bien activé côté CubeMX (NVIC_EnableIRQ),
     * ce driver ne le fait pas pour ne pas dupliquer une config déjà générée. */
    print_to_console("\n\rRemote driver initialized", sizeof("\n\rRemote driver initialized"));
}