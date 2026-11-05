// Descripteurs USB du bootloader : une interface DFU 1.1, plus les descripteurs Microsoft OS 2.0
// qui font charger WinUSB automatiquement sous Windows (pas de pilote à installer, ni de Zadig).

#include <string.h>

#include "stm32f4xx.h"
#include "tusb.h"

// pid.codes : PID de test, à remplacer par un PID attribué avant toute diffusion.
#define USB_VID 0x1209
#define USB_PID 0x0001

#define VENDOR_REQUEST_MICROSOFT 1

enum {
    STR_LANGID,
    STR_MANUFACTURER,
    STR_PRODUCT,
    STR_SERIAL,
    STR_DFU,
};

static const tusb_desc_device_t device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0210, // 2.1 : le PC demande alors le descripteur BOS
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = STR_MANUFACTURER,
    .iProduct = STR_PRODUCT,
    .iSerialNumber = STR_SERIAL,
    .bNumConfigurations = 1,
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_DFU_DESC_LEN(1))

static const uint8_t config_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_DFU_DESCRIPTOR(0, 1, STR_DFU, DFU_ATTR_CAN_DOWNLOAD | DFU_ATTR_MANIFESTATION_TOLERANT, 1000,
                       CFG_TUD_DFU_XFER_BUFSIZE),
};

// Microsoft OS 2.0 : périphérique non composite, donc ni sous-ensemble de configuration ni de fonction.
#define MS_OS_20_DESC_LEN 0xA2

static const uint8_t ms_os_20_desc[] = {
    // En-tête : longueur, type, version de Windows (8.1), longueur totale
    U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000),
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

    // ID compatible : WINUSB
    U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID),
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

    // Propriété de registre DeviceInterfaceGUIDs (REG_MULTI_SZ), pour que libusb trouve le périphérique
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0,
    'c', 0, 'e', 0, 'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,
    U16_TO_U8S_LE(0x0050),
    '{', 0, '3', 0, 'A', 0, '7', 0, 'E', 0, '1', 0, 'B', 0, '6', 0, '2', 0, '-', 0, '4', 0, 'F', 0, '0', 0,
    'C', 0, '-', 0, '4', 0, 'B', 0, '2', 0, 'D', 0, '-', 0, '9', 0, 'C', 0, '1', 0, 'E', 0, '-', 0, '5', 0,
    'D', 0, '8', 0, 'A', 0, '0', 0, 'F', 0, '7', 0, 'B', 0, '3', 0, 'C', 0, '2', 0, '1', 0, '}', 0, 0, 0,
    0, 0,
};

TU_VERIFY_STATIC(sizeof(ms_os_20_desc) == MS_OS_20_DESC_LEN, "taille du descripteur MS OS 2.0");

#define BOS_TOTAL_LEN (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)

static const uint8_t bos_desc[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

static const char *const strings[] = {
    [STR_MANUFACTURER] = "Aerolith",
    [STR_PRODUCT] = "LithCore bootloader",
    [STR_DFU] = "Logolith application",
};

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&device_desc;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return config_desc;
}

const uint8_t *tud_descriptor_bos_cb(void)
{
    return bos_desc;
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, const tusb_control_request_t *request)
{
    if (stage != CONTROL_STAGE_SETUP) {
        return true;
    }
    if (request->bRequest == VENDOR_REQUEST_MICROSOFT && request->wIndex == 7) {
        return tud_control_xfer(rhport, request, (void *)(uintptr_t)ms_os_20_desc, MS_OS_20_DESC_LEN);
    }
    return false;
}

// Numéro de série : identifiant unique 96 bits du STM32, comme l'application.
static size_t serial_to_utf16(uint16_t *out)
{
    static const char hex[] = "0123456789ABCDEF";
    const uint8_t *uid = (const uint8_t *)UID_BASE;
    size_t n = 0;
    for (int i = 11; i >= 0; i--) {
        out[n++] = (uint16_t)hex[uid[i] >> 4];
        out[n++] = (uint16_t)hex[uid[i] & 0xF];
    }
    return n;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    static uint16_t buf[1 + 32];
    size_t len;

    if (index == STR_LANGID) {
        buf[1] = 0x0409; // anglais (US)
        len = 1;
    } else if (index == STR_SERIAL) {
        len = serial_to_utf16(&buf[1]);
    } else if (index < sizeof strings / sizeof strings[0] && strings[index]) {
        const char *s = strings[index];
        for (len = 0; s[len] && len < 32; len++) {
            buf[1 + len] = (uint16_t)s[len];
        }
    } else {
        return NULL;
    }

    buf[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
    return buf;
}
