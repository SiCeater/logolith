#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "clock.h"
#include "gps.h"
#include "icm42688.h"
#include "log.h"
#include "nmea.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "task.h"

// LED d'état de la LithCore, anodes au 3V3 : allumées quand la broche est à 0.
#define LED_BLUE_PORT GPIOC
#define LED_BLUE_PIN  LL_GPIO_PIN_0
#define LED_RED_PORT  GPIOB
#define LED_RED_PIN   LL_GPIO_PIN_9

#define BLINK_PERIOD_MS 420U
#define BLINK_STACK     configMINIMAL_STACK_SIZE
#define GPS_STACK       configMINIMAL_STACK_SIZE
#define IMU_STACK       configMINIMAL_STACK_SIZE
#define BLINK_PRIORITY  (tskIDLE_PRIORITY + 1)

static StackType_t blink_stack[BLINK_STACK];
static StaticTask_t blink_tcb;
static StackType_t gps_stack[GPS_STACK];
static StaticTask_t gps_tcb;
static StackType_t imu_stack[IMU_STACK];
static StaticTask_t imu_tcb;

static void leds_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB | LL_AHB1_GRP1_PERIPH_GPIOC);

    // Éteintes (broche haute) avant de passer en sortie.
    LL_GPIO_SetOutputPin(LED_BLUE_PORT, LED_BLUE_PIN);
    LL_GPIO_SetOutputPin(LED_RED_PORT, LED_RED_PIN);

    LL_GPIO_InitTypeDef gpio;
    LL_GPIO_StructInit(&gpio);
    gpio.Mode = LL_GPIO_MODE_OUTPUT;
    gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio.Speed = LL_GPIO_SPEED_FREQ_LOW;

    gpio.Pin = LED_BLUE_PIN;
    LL_GPIO_Init(LED_BLUE_PORT, &gpio);
    gpio.Pin = LED_RED_PIN;
    LL_GPIO_Init(LED_RED_PORT, &gpio);
}

static void led_red_on(void)
{
    LL_GPIO_ResetOutputPin(LED_RED_PORT, LED_RED_PIN);
}

static void blink_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();

    log_info(LOG_NORMAL, "Toggling LED!");

    for (;;) {
        LL_GPIO_TogglePin(LED_BLUE_PORT, LED_BLUE_PIN);
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(BLINK_PERIOD_MS));
    }
}

static void imu_task(void *arg)
{
    (void)arg;

    IMU_SPI1_LL_Init();

    for (;;) {

        icm_whoami();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void gps_task(void *arg)
{
    (void)arg;
    UART4_LL_IT_INIT();
    char line[96];
    uint8_t line_index = 0;
    uint8_t line_ready = 0;
    log_info(LOG_NORMAL, "GPS task started");

    for (;;) {
        while (UART4_Available()) {
            uint8_t data = UART4_Read();

            if (data == '$') {
                line_index = 0;
            } else if (data == '\n') {
                line_ready = 1;
            }

            line[line_index] = data;
            line_index++;

            if (line_ready) {
                line[line_index] = '\0';
                SEGGER_RTT_printf(0, "NMEA frame: %s\r\n", line);
                SEGGER_RTT_printf(0, "Frame correct? %d\r\n", nmea_checksum_ok(line));
                line_ready = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main(void)
{
    leds_init();

    // Sans quartz, les horloges seraient fausses (USB, UART, PWM des moteurs) : on s'arrête en rouge.
    if (!clock_init()) {
        led_red_on();
        for (;;) {
        }
    }

    log_boot();

    xTaskCreateStatic(blink_task, "blink", BLINK_STACK, NULL, BLINK_PRIORITY, blink_stack, &blink_tcb);
    xTaskCreateStatic(gps_task, "gps", GPS_STACK, NULL, BLINK_PRIORITY, gps_stack, &gps_tcb);
    xTaskCreateStatic(imu_task, "imu", IMU_STACK, NULL, BLINK_PRIORITY, imu_stack, &imu_tcb);
    vTaskStartScheduler();

    for (;;) { // atteint seulement si le noyau n'a pas pu démarrer
    }
}

// ---------------------------------------------------------------------------------------------
// Erreurs fatales : LED rouge allumée, interruptions coupées. Un débogueur trouve l'état intact.
// ---------------------------------------------------------------------------------------------

static void fatal(void)
{
    __disable_irq();
    led_red_on();
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

void HardFault_Handler(void)
{
    fatal();
}

// Appelés par __libc_init_array dans le startup ; vides car on lie sans crti/crtn (-nostartfiles).
void _init(void) {}
void _fini(void) {}
