#include "clock.h"

#include "stm32f4xx.h"

// Démarrage du quartz : quelques ms en pratique. Le manuel de ST prévoit 100 ms au maximum.
#define HSE_TIMEOUT_MS 100U

// PLL (RM0090, section 6.3.2) : VCO = HSE / M * N, SYSCLK = VCO / P, USB = VCO / Q.
// 8 MHz / 4 = 2 MHz (entrée PLL entre 1 et 2 MHz), * 168 = 336 MHz, / 2 = 168 MHz, / 7 = 48 MHz.
#define PLL_M 4U
#define PLL_N 168U
#define PLL_P 2U
#define PLL_Q 7U

bool clock_init(void)
{
    // Compteur de cycles du cœur, pour le délai d'attente du quartz (HSI à 16 MHz à ce stade).
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) {
        if (DWT->CYCCNT > HSI_VALUE / 1000U * HSE_TIMEOUT_MS) {
            RCC->CR &= ~RCC_CR_HSEON;
            return false;
        }
    }

    // Régulateur en mode 1, requis au-delà de 144 MHz. Relire le registre d'horloge laisse le temps
    // au périphérique PWR de démarrer (errata ES0182, 2.1.6).
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;
    PWR->CR |= PWR_CR_VOS;

    // Les bits réservés de PLLCFGR doivent garder leur valeur de reset.
    RCC->PLLCFGR =
        (RCC->PLLCFGR &
         ~(RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ)) |
        RCC_PLLCFGR_PLLSRC_HSE | (PLL_M << RCC_PLLCFGR_PLLM_Pos) | (PLL_N << RCC_PLLCFGR_PLLN_Pos) |
        ((PLL_P / 2U - 1U) << RCC_PLLCFGR_PLLP_Pos) | (PLL_Q << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
    }

    // Ordre imposé : diviseurs des bus et attentes de la flash d'abord, vérifiés, puis bascule à
    // 168 MHz. Tout écrire d'un coup laisse un instant les bus hors spécification (APB1 max 42 MHz),
    // avec des plantages qui dépendent du timing.
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) | RCC_CFGR_HPRE_DIV1 |
                RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

    // 5 cycles d'attente pour 168 MHz sous 2,7 à 3,6 V (RM0090, tableau 10). Caches vidés avant activation.
    FLASH->ACR = FLASH_ACR_ICRST | FLASH_ACR_DCRST;
    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS) {
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }

    SystemCoreClockUpdate();
    return true;
}
