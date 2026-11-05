#include "heartbeat.h"

#include "FreeRTOS.h"
#include "board.h"
#include "task.h"

#define HEARTBEAT_PERIOD_MS 500
#define HEARTBEAT_STACK     configMINIMAL_STACK_SIZE
#define HEARTBEAT_PRIORITY  (tskIDLE_PRIORITY + 1)

static StackType_t stack[HEARTBEAT_STACK];
static StaticTask_t tcb;

static void heartbeat_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        board_led_toggle(BOARD_LED_BLUE);
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
    }
}

void heartbeat_start(void)
{
    xTaskCreateStatic(heartbeat_task, "heartbeat", HEARTBEAT_STACK, NULL, HEARTBEAT_PRIORITY, stack, &tcb);
}
