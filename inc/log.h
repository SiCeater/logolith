#pragma once

typedef enum {
    LOG_IMPORTANT = 1,
    LOG_NORMAL = 2,
    LOG_CANARY = 3,
    LOG_OTHER = 4
} log_level_t;

const char* reset_cause(void);

void log_info(log_level_t log_level, const char* str);

void log_boot();
