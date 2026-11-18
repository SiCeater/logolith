#include "icm42688.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_system.h"

#define IMU_CS_LOW()  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_2)
#define IMU_CS_HIGH() LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_2)

volatile uint8_t icm_whoami_value;

void IMU_SPI1_LL_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_SPI_InitTypeDef SPI_InitStruct = {0};
    LL_EXTI_InitTypeDef EXTI_InitStruct = {0};

    // activation des clocks GPIOA GPIOC SPI1 SYSCFG
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
    // the interrupt uses SYSCFG, so we need to enable its clock as well
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

    // configure CS pin PC2 push-pull output, high by default (push pull meaning it can drive both high and
    // low, as opposed to open drain which can only drive low)
    IMU_CS_HIGH(); // this is the default state, the chip is not selected
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    // this means no pull-up / pull-down is used, the pin is floating when not driven.
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // SPI1 : PA5 SCK, PA6 MISO, PA7 MOSI
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    // alternate function means the pin is controlled by a peripheral (SPI1 in this case)
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_5; // AF5 is the alternate function for SPI1 on these pins
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // INT2 PC4 input
    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE4);
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&EXTI_InitStruct);

    NVIC_SetPriority(EXTI4_IRQn, 5);
    NVIC_EnableIRQ(EXTI4_IRQn);

    // config du périphérique SPI1 (Mode 0 24 MHz)
    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    LL_SPI_Init(SPI1, &SPI_InitStruct);

    LL_SPI_Enable(SPI1);
}

// SPI = bus partagé où CS est actif à l'état bas (0 = capteur sélectionné, 1 = capteur ignoré)
//

void icm_init()
{
    /*
    ICM42688P câblé sur SPI1
    INT2 sur PC4
    CS sur PC2
    MISO sur PA6
    MOSI sur PA7
    SCK PA5
    */
}

void icm_whoami()
{

    uint8_t whoami = 0;

    // selectionner IMU
    IMU_CS_LOW();

    // enovyer l'@ en lecture (@ : 0x75  | R : 1 W : 0 0x80 à 0xF5)
    while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {
    }
    LL_SPI_TransmitData8(SPI1, 0xF5);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {
    }
    (void)LL_SPI_ReceiveData8(SPI1);

    // enovyer dummy byte (0x00) pour lire la réponse
    while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {
    }
    LL_SPI_TransmitData8(SPI1, 0x00);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {
    }
    whoami = LL_SPI_ReceiveData8(SPI1); // doit valoir 0x47

    // attendre la fin physique du xfer et relâcher CS
    while (LL_SPI_IsActiveFlag_BSY(SPI1)) {
    }
    IMU_CS_HIGH();

    icm_whoami_value = whoami;
}
