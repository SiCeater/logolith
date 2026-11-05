#include "board.h"

#include "stm32f4xx.h"

#define HSE_TIMEOUT 100000U

typedef struct {
    GPIO_TypeDef *port;
    uint32_t pin;
} led_pin_t;

// Anodes au 3V3 : la LED s'allume quand la broche est à 0.
static const led_pin_t led_pins[] = {
    [BOARD_LED_RED] = {GPIOB, 9},
    [BOARD_LED_BLUE] = {GPIOC, 0},
};

static bool hse_failed;

// SYSCLK 168 MHz, AHB 168, APB1 42, APB2 84, PLLQ 48 MHz (USB).
static void clock_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    RCC->CR |= RCC_CR_HSEON;
    uint32_t timeout = HSE_TIMEOUT;
    while (!(RCC->CR & RCC_CR_HSERDY) && --timeout) {
    }
    hse_failed = (timeout == 0);

    // VCO = source / M * N = 2 MHz * 168 = 336 MHz ; SYSCLK = VCO / 2 ; USB = VCO / 7.
    uint32_t source, m;
    if (hse_failed) {
        RCC->CR &= ~RCC_CR_HSEON;
        source = RCC_PLLCFGR_PLLSRC_HSI;
        m = HSI_VALUE / 2000000U;
    } else {
        source = RCC_PLLCFGR_PLLSRC_HSE;
        m = HSE_VALUE / 2000000U;
    }
    RCC->PLLCFGR = source
                   | (m << RCC_PLLCFGR_PLLM_Pos)
                   | (168U << RCC_PLLCFGR_PLLN_Pos)
                   | (0U << RCC_PLLCFGR_PLLP_Pos) // /2
                   | (7U << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
    }

    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }

    SystemCoreClockUpdate();
}

static void gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    for (unsigned i = 0; i < sizeof led_pins / sizeof led_pins[0]; i++) {
        const led_pin_t *p = &led_pins[i];
        p->port->BSRR = 1U << p->pin; // éteinte avant de passer en sortie
        p->port->MODER = (p->port->MODER & ~(3U << (p->pin * 2))) | (1U << (p->pin * 2));
    }
}

void board_init(void)
{
    clock_init();
    gpio_init();
}

void board_led_set(board_led_t led, bool on)
{
    const led_pin_t *p = &led_pins[led];
    p->port->BSRR = on ? (1U << (p->pin + 16)) : (1U << p->pin);
}

void board_led_toggle(board_led_t led)
{
    const led_pin_t *p = &led_pins[led];
    board_led_set(led, p->port->ODR & (1U << p->pin)); // broche haute = LED éteinte
}

bool board_hse_failed(void)
{
    return hse_failed;
}

// Appelés par __libc_init_array dans le startup ; vides car on lie sans crti/crtn (-nostartfiles).
void _init(void) {}
void _fini(void) {}
