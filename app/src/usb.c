#include "usb.h"

#include "FreeRTOS.h"
#include "board.h"
#include "task.h"
#include "tusb.h"

#define USB_STACK    (configMINIMAL_STACK_SIZE * 2)
#define USB_PRIORITY (tskIDLE_PRIORITY + 2)

// Débit « magique » : même convention que les cartes Arduino et RP2040.
#define DFU_TOUCH_BAUDRATE 1200
#define DFU_DETACH_MS      250

static StackType_t stack[USB_STACK];
static StaticTask_t tcb;

static void usb_task(void *arg)
{
    (void)arg;
    tud_init(BOARD_TUD_RHPORT);

    for (;;) {
        tud_task(); // bloque jusqu'au prochain événement USB
    }
}

void usb_start(void)
{
    board_usb_init();
    xTaskCreateStatic(usb_task, "usb", USB_STACK, NULL, USB_PRIORITY, stack, &tcb);
}

void OTG_FS_IRQHandler(void)
{
    tud_int_handler(BOARD_TUD_RHPORT);
}

// Le PC ferme le port (DTR retombe) alors qu'il était réglé à 1200 bauds : on passe en DFU.
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)rts;
    cdc_line_coding_t coding;
    tud_cdc_n_get_line_coding(itf, &coding);

    if (!dtr && coding.bit_rate == DFU_TOUCH_BAUDRATE) {
        // Débranchement logiciel puis attente : sans ça, le reset ne coupe la ligne que quelques µs
        // et le PC ne voit pas qu'un autre périphérique (le bootloader) a pris la place.
        vTaskDelay(pdMS_TO_TICKS(20)); // laisse la requête USB en cours se terminer
        tud_disconnect();
        vTaskDelay(pdMS_TO_TICKS(DFU_DETACH_MS));
        board_reset_to_bootloader();
    }
}
