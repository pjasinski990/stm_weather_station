#include "stm32l4xx_hal.h"

#include "epaper.h"
#include "logger.h"

#define EP_CS_PIN_Pin GPIO_PIN_2
#define EP_CS_PIN_GPIO_Port GPIOD

static SPI_HandleTypeDef spi;

void epaper_init() {
    log_write("initalizing epaper");
    // enable gpio ports
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // spi init
    // miso C7; mosi C3; clk B10
    log_write("initalizing spi");
    spi.Instance = SPI3;
    spi.Init.Mode = SPI_MODE_MASTER;
    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.DataSize = SPI_DATASIZE_4BIT;
    spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.Init.CRCPolynomial = 7;
    spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    spi.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&spi) != HAL_OK)
    {
        log_write("in %s - Error initializing SPI", __func__);
        while (1)
            ;
    }
    log_write("epaper SPI OK");

    // cs C7
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_GPIO_WritePin(EP_CS_PIN_GPIO_Port, EP_CS_PIN_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = EP_CS_PIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EP_CS_PIN_GPIO_Port, &GPIO_InitStruct);
}