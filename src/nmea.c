#include "nmea.h"
#include <stdint.h>
#include <stdlib.h>

int nmea_checksum_ok(const char *line)
{
    int index = 0;
    uint8_t checksum = 0;
    if (line[0] == '$') {
        index++;
        while (line[index] != '*') {
            checksum ^= line[index];
            index++;
        }

        char hex_str[3] = {line[index + 1], line[index + 2], '\0'};
        uint8_t expected = (uint8_t)strtol(hex_str, NULL, 16);
        if (checksum == expected) {
            return 1;
        } else {
            return 0;
        }
    } else {
        return 0;
    }
}
