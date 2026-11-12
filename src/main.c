#include "FreeRTOS.h"
#include "clock.h"
#include "stm32f4xx.h"
#include "task.h"

// LED d'état de la LithCore, anodes au 3V3 : allumées quand la broche est à 0.
#define LED_BLUE_PIN 0U // PC0
#define LED_RED_PIN  9U // PB9

#define BLINK_PERIOD_MS 500U
#define BLINK_STACK     configMINIMAL_STACK_SIZE
#define BLINK_PRIORITY  (tskIDLE_PRIORITY + 1)

static StackType_t blink_stack[BLINK_STACK];
static StaticTask_t blink_tcb;

static void leds_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    // Éteintes (broche haute) avant de passer en sortie.
    GPIOC->BSRR = 1U << LED_BLUE_PIN;
    GPIOB->BSRR = 1U << LED_RED_PIN;
    GPIOC->MODER = (GPIOC->MODER & ~(3U << (LED_BLUE_PIN * 2))) | (1U << (LED_BLUE_PIN * 2));
    GPIOB->MODER = (GPIOB->MODER & ~(3U << (LED_RED_PIN * 2))) | (1U << (LED_RED_PIN * 2));
}

static void led_red_on(void)
{
    GPIOB->BSRR = 1U << (LED_RED_PIN + 16);
}

static void blink_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        GPIOC->ODR ^= 1U << LED_BLUE_PIN;
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(BLINK_PERIOD_MS));
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

    xTaskCreateStatic(blink_task, "blink", BLINK_STACK, NULL, BLINK_PRIORITY, blink_stack, &blink_tcb);
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
