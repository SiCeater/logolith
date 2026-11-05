// Descripteurs USB : un seul port série CDC.

#include "stm32f4xx.h"
#include "tusb.h"

// VID/PID du port COM virtuel de ST, reconnu partout sans pilote (comme Betaflight).
#define USB_VID 0x0483
#define USB_PID 0x5740

enum {
    ITF_NUM_CDC,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL,
};

enum {
    STR_LANGID,
    STR_MANUFACTURER,
    STR_PRODUCT,
    STR_SERIAL,
    STR_CDC,
};

#define EP_CDC_NOTIF 0x81
#define EP_CDC_OUT   0x02
#define EP_CDC_IN    0x82

static const tusb_desc_device_t device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    // Classe composite (IAD) : nécessaire pour que Windows charge usbser.sys sur le CDC.
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = STR_MANUFACTURER,
    .iProduct = STR_PRODUCT,
    .iSerialNumber = STR_SERIAL,
    .bNumConfigurations = 1,
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t config_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STR_CDC, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64),
};

static const char *const strings[] = {
    [STR_MANUFACTURER] = "Aerolith",
    [STR_PRODUCT] = "LithCore",
    [STR_CDC] = "Logolith console",
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

// Numéro de série : identifiant unique 96 bits du STM32, en hexadécimal.
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
