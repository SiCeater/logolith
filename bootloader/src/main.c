// Bootloader Logolith : lance l'application, ou reste en mode DFU USB pour en recevoir une nouvelle.
//
// Mode DFU si l'application l'a demandé avant le reset (port série ouvert à 1200 bauds),
// ou si la zone application ne contient pas d'image valide.

#include "board.h"
#include "boot.h"
#include "stm32f4xx.h"
#include "tusb.h"

#define LED_PERIOD_MS 100
#define DETACH_MS     250 // le PC doit voir un débranchement avant que l'application ne se connecte

static volatile uint32_t ticks_ms;

bool app_is_valid(void)
{
    const uint32_t *vectors = (const uint32_t *)APP_START;
    uint32_t sp = vectors[0];
    uint32_t entry = vectors[1];

    return sp >= SRAM1_BASE && sp <= SRAM1_BASE + 128U * 1024 && (sp & 3U) == 0
           && (entry & 1U) && entry > APP_START && entry < APP_END;
}

// Appelé avant toute initialisation : l'application trouve la puce comme après un reset.
static void jump_to_app(void)
{
    const uint32_t *vectors = (const uint32_t *)APP_START;
    uint32_t sp = vectors[0];
    uint32_t entry = vectors[1];

    SCB->VTOR = APP_START;
    __DSB();
    __ISB();
    __set_MSP(sp);
    ((void (*)(void))entry)();
    for (;;) {
    }
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = ticks_ms;
    while (ticks_ms - start < ms) {
        tud_task();
    }
}

int main(void)
{
    if (!board_take_bootloader_request() && app_is_valid()) {
        jump_to_app();
    }

    board_init();
    SysTick_Config(SystemCoreClock / 1000);
    board_usb_init();
    tud_init(BOARD_TUD_RHPORT);

    uint32_t last_blink = 0;
    for (;;) {
        tud_task();

        if (ticks_ms - last_blink >= LED_PERIOD_MS) {
            last_blink = ticks_ms;
            board_led_toggle(BOARD_LED_RED);
        }

        if (dfu_done()) {
            delay_ms(50); // laisse le PC lire le dernier statut
            tud_disconnect();
            delay_ms(DETACH_MS);
            board_reset();
        }
    }
}

void SysTick_Handler(void)
{
    ticks_ms++;
}

// Plantage du bootloader : LED bleue très rapide, rouge éteinte.
void HardFault_Handler(void)
{
    board_led_set(BOARD_LED_RED, false);
    for (;;) {
        board_led_toggle(BOARD_LED_BLUE);
        for (volatile uint32_t i = 0; i < 200000; i++) {
        }
    }
}

void OTG_FS_IRQHandler(void)
{
    tud_int_handler(BOARD_TUD_RHPORT);
}

// Horloge de TinyUSB sans RTOS.
uint32_t tusb_time_millis_api(void)
{
    return ticks_ms;
}
