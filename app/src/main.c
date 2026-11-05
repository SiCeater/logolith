#include "FreeRTOS.h"
#include "board.h"
#include "heartbeat.h"
#include "task.h"
#include "usb.h"

int main(void)
{
    board_init();

    // Quartz absent : on vole quand même sur l'oscillateur interne, mais on le signale.
    board_led_set(BOARD_LED_RED, board_hse_failed());

    heartbeat_start();
    usb_start();
    vTaskStartScheduler();

    // Atteint seulement si le noyau n'a pas pu démarrer.
    for (;;) {
    }
}
