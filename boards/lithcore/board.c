#include "board.h"

#include "stm32f4xx.h"

#define HSE_TIMEOUT 100000U

#define BOOTLOADER_MAGIC 0xB007D0F0U
#define USB_IRQ_PRIORITY 6U // >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY : peut appeler FreeRTOS

// Fournis par memory.ld et le startup.
extern uint32_t __boot_shared[];
extern uint32_t g_pfnVectors[];

// Zone de RAM que ni le startup ni le bootloader n'initialisent : garde sa valeur à travers un reset.
// Après une mise sous tension elle est aléatoire, mais tomber par hasard sur le magic est négligeable.
#define BOOT_REQUEST (*(volatile uint32_t *)__boot_shared)

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
    (void)RCC->APB1ENR; // errata : attendre que l'horloge du périphérique soit active
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
    // Les bits réservés de PLLCFGR doivent garder leur valeur de reset.
    RCC->PLLCFGR = (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN
                                     | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ))
                   | source
                   | (m << RCC_PLLCFGR_PLLM_Pos)
                   | (168U << RCC_PLLCFGR_PLLN_Pos)
                   | (0U << RCC_PLLCFGR_PLLP_Pos) // /2
                   | (7U << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
    }

    // Ordre imposé par le manuel : diviseurs des bus et attentes de la flash d'abord, vérifiés,
    // et seulement ensuite le passage à 168 MHz. Tout faire d'un coup laisse un instant les bus
    // hors spécification, avec des plantages qui dépendent du timing.
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
                | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    FLASH->ACR = FLASH_ACR_ICRST | FLASH_ACR_DCRST; // caches vidés pendant qu'ils sont désactivés
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS) {
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
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
    // Chaque image pointe VTOR sur sa propre table : on ne se fie pas à ce qu'a laissé l'étage
    // précédent (bootloader maison, ou bootloader ROM de ST qui laisse la ROM projetée en 0).
    SCB->VTOR = (uint32_t)g_pfnVectors;
    __DSB();

    clock_init();
    gpio_init();
}

void board_usb_init(void)
{
    // PA11 (D-) et PA12 (D+) en fonction alternative 10 (OTG_FS), vitesse maximale.
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;
    GPIOA->MODER = (GPIOA->MODER & ~(0xFU << 22)) | (0xAU << 22);
    GPIOA->OSPEEDR |= 0xFU << 22;
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFFU << 12)) | (0xAAU << 12);

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    (void)RCC->AHB2ENR;

    // VBUS n'est pas relié à PA9 sur la LithCore (PA9 pilote un moteur) : détection désactivée.
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
    USB_OTG_FS->GCCFG &= ~(USB_OTG_GCCFG_VBUSBSEN | USB_OTG_GCCFG_VBUSASEN);

    NVIC_SetPriority(OTG_FS_IRQn, USB_IRQ_PRIORITY);
}

void board_reset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void board_reset_to_bootloader(void)
{
    __disable_irq();
    BOOT_REQUEST = BOOTLOADER_MAGIC;
    NVIC_SystemReset();
}

bool board_take_bootloader_request(void)
{
    bool requested = (BOOT_REQUEST == BOOTLOADER_MAGIC);
    BOOT_REQUEST = 0;
    return requested;
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
