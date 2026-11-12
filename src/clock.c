#include "clock.h"

#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"

// Démarrage du quartz : quelques ms en pratique. Le manuel de ST prévoit 100 ms au maximum.
#define HSE_TIMEOUT_MS 100U

// PLL (RM0090, section 6.3.2) : VCO = HSE / M * N, SYSCLK = VCO / P, USB = VCO / Q.
// 8 MHz / 4 = 2 MHz (entrée PLL entre 1 et 2 MHz), * 168 = 336 MHz, / 2 = 168 MHz, / 7 = 48 MHz.
#define PLL_M LL_RCC_PLLM_DIV_4
#define PLL_N 168U
#define PLL_P LL_RCC_PLLP_DIV_2
#define PLL_Q LL_RCC_PLLQ_DIV_7

bool clock_init(void)
{
    // Compteur de cycles du cœur, pour le délai d'attente du quartz (HSI à 16 MHz à ce stade).
    // Le DWT fait partie du cœur Cortex-M4, pas des périphériques de ST : la LL ne le couvre pas, d'où CMSIS.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    LL_RCC_HSE_Enable();
    while (!LL_RCC_HSE_IsReady()) {
        if (DWT->CYCCNT > HSI_VALUE / 1000U * HSE_TIMEOUT_MS) {
            LL_RCC_HSE_Disable();
            return false;
        }
    }

    // Régulateur en mode 1, requis au-delà de 144 MHz. LL_APB1_GRP1_EnableClock relit le registre
    // d'horloge, ce qui laisse au périphérique PWR le temps de démarrer (errata ES0182, 2.1.6).
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    // Les deux fonctions ne modifient que leurs champs de PLLCFGR : les bits réservés gardent leur valeur
    // de reset, comme l'exige le manuel. M et N sont communs aux sorties système (P) et USB (Q).
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, PLL_M, PLL_N, PLL_P);
    LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, PLL_M, PLL_N, PLL_Q);
    LL_RCC_PLL_Enable();
    while (!LL_RCC_PLL_IsReady()) {
    }

    // Ordre imposé : diviseurs des bus et attentes de la flash d'abord, vérifiés, puis bascule à
    // 168 MHz. Tout écrire d'un coup laisse un instant les bus hors spécification (APB1 max 42 MHz),
    // avec des plantages qui dépendent du timing.
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

    // 5 cycles d'attente pour 168 MHz sous 2,7 à 3,6 V (RM0090, tableau 10). Caches vidés avant activation.
    LL_FLASH_EnableInstCacheReset();
    LL_FLASH_DisableInstCacheReset();
    LL_FLASH_EnableDataCacheReset();
    LL_FLASH_DisableDataCacheReset();
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5) {
    }
    LL_FLASH_EnablePrefetch();
    LL_FLASH_EnableInstCache();
    LL_FLASH_EnableDataCache();

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }

    SystemCoreClockUpdate(); // relit les registres RCC pour que SystemCoreClock vaille 168 MHz
    return true;
}
