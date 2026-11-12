#include "log.h"
#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "task.h"

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

    default:
        prefix = "[UNKNOWN]";
        color = RTT_CTRL_TEXT_BRIGHT_MAGENTA;
        break;
    }

    SEGGER_RTT_printf(0, "%s%s[%d]%s%s\r\n", color, prefix, xTaskGetTickCount(), str, RTT_CTRL_RESET);
}
