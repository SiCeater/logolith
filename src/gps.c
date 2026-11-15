#include "gps.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;

void UART4_LL_IT_INIT(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_USART_InitTypeDef UART_InitStruct = {0};

    /* Enable clocks */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);

    /* Configure PA0 (TX) and PA1 (RX) */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure UART4 9600 8N1 */
    UART_InitStruct.BaudRate = 9600;
    UART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    UART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    UART_InitStruct.Parity = LL_USART_PARITY_NONE;
    UART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    UART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

    LL_USART_Init(UART4, &UART_InitStruct);
    LL_USART_ConfigAsyncMode(UART4);

    /* Configure NVIC interrupt for UART4 */
    NVIC_SetPriority(UART4_IRQn, 1);
    NVIC_EnableIRQ(UART4_IRQn);

    /* Enable RX interrupt RXNE and peripheral */
    LL_USART_EnableIT_RXNE(UART4);
    LL_USART_Enable(UART4);
}

/* Interrupt routine */
void UART4_IRQHandler(void)
{
    /* check if interrupt comes from data RX */
    if (LL_USART_IsActiveFlag_RXNE(UART4) && LL_USART_IsEnabledIT_RXNE(UART4)) {
        uint8_t received_byte = LL_USART_ReceiveData8(UART4); // read and clear RXNE flag

        uint8_t next_head = (uint8_t)((rx_head + 1) % RX_BUFFER_SIZE);
        if (next_head != rx_tail) { // avoid erasing unread data
            rx_buffer[rx_head] = received_byte;
            rx_head = next_head;
        }
    }

    if (LL_USART_IsActiveFlag_ORE(UART4)) {
        LL_USART_ClearFlag_ORE(UART4);
    }
}

uint8_t UART4_Available(void)
{
    return (rx_head != rx_tail);
}

uint8_t UART4_Read(void)
{
    if (rx_head == rx_tail) {
        return 0;
    }
    uint8_t data = rx_buffer[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1) % RX_BUFFER_SIZE);
    return data;
}
