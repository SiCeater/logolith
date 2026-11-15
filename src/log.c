#include "log.h"
#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "manifest.h"
#include "stm32f4xx_ll_rcc.h"
#include "task.h"

const char *reset_cause(void)
{
    if (LL_RCC_IsActiveFlag_LPWRRST())
        return "low power: woke from standby/stop mode, with reset-on wake enabled.";
    if (LL_RCC_IsActiveFlag_WWDGRST())
        return "watchdog (WWDG): window watchdog expired, code ran too early or too late.";
    if (LL_RCC_IsActiveFlag_IWDGRST())
        return "watchdog (IWDG): independent watchdog expired.";
    if (LL_RCC_IsActiveFlag_SFTRST())
        return "software: the program requested reset.";
    if (LL_RCC_IsActiveFlag_BORRST())
        return "brown-out: supply voltage dropped below the minimum.";
    if (LL_RCC_IsActiveFlag_PORRST())
        return "power-on: initial power-up.";
    if (LL_RCC_IsActiveFlag_PINRST())
        return "reset pin (NRST): reset button pressed or debugger reset.";
    return "unknown";
}

void log_info(log_level_t log_level, const char *str)
{
    if (str == NULL) {
        return;
    }

    const char *prefix = "";
    const char *color = "";

    switch (log_level) {
    case LOG_IMPORTANT:
        prefix = "[IMPORTANT]";
        color = RTT_CTRL_TEXT_BRIGHT_RED;
        break;

    case LOG_NORMAL:
        prefix = "[NORMAL]";
        color = RTT_CTRL_TEXT_BRIGHT_GREEN;
        break;

    case LOG_CANARY:
        prefix = "[CANARY]";
        color = RTT_CTRL_TEXT_BRIGHT_YELLOW;
        break;

    case LOG_OTHER:
        prefix = "[OTHER]";
        color = RTT_CTRL_BG_BLUE;
        break;

    default:
        prefix = "[UNKNOWN]";
        color = RTT_CTRL_TEXT_BRIGHT_MAGENTA;
        break;
    }

    SEGGER_RTT_printf(0, "%s%s[%d]%s%s\r\n", color, prefix, xTaskGetTickCount(), str, RTT_CTRL_RESET);
}

void log_boot()
{
    LL_RCC_ClocksTypeDef clock;
    LL_RCC_GetSystemClocksFreq(&clock);

    const char *cause = reset_cause();

    SEGGER_RTT_printf(0,
                      RTT_CTRL_TEXT_BRIGHT_GREEN
                      "project  : " RTT_CTRL_RESET "%s\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN
                      "firmware : " RTT_CTRL_RESET "%s\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN
                      "MCU      : " RTT_CTRL_RESET "%s\r\n" RTT_CTRL_TEXT_BRIGHT_YELLOW
                      "reset    : " RTT_CTRL_RESET "%s\r\n" RTT_CTRL_TEXT_BRIGHT_CYAN
                      "SYSCLK   : " RTT_CTRL_RESET "%lu Hz\r\n",
                      PROJECT_NAME, FW_VERSION, BOARD_MCU, cause, clock.SYSCLK_Frequency);

    LL_RCC_ClearResetFlags();
}
