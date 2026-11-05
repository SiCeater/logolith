// Erreurs fatales du noyau : on fige la carte, LED rouge allumée, pour qu'un débogueur trouve l'état intact.

#include "FreeRTOS.h"
#include "board.h"
#include "task.h"

static void fatal(void)
{
    taskDISABLE_INTERRUPTS();
    board_led_set(BOARD_LED_BLUE, false);
    board_led_set(BOARD_LED_RED, true);
    for (;;) {
    }
}

void rtos_assert_failed(const char *file, int line)
{
    (void)file;
    (void)line;
    fatal();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    fatal();
}
