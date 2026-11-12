#pragma once

typedef enum {
    LOG_IMPORTANT = 1,
    LOG_NORMAL = 2,
    LOG_CANARY = 3
} log_level_t;


void log_info(log_level_t log_level, const char* str);
