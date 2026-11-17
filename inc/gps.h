#pragma once

#include <stdint.h>


#define RX_BUFFER_SIZE 64

extern volatile uint8_t rx_buffer[RX_BUFFER_SIZE]; // circular buffer for received data
extern volatile uint8_t rx_head;               // index of the next byte to write
extern volatile uint8_t rx_tail;               // index of the next byte to read

void UART4_LL_IT_INIT(void);

void UART4_IRQHandler(void);

uint8_t UART4_Available(void);

uint8_t UART4_Read(void);
