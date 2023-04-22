#include "stm32l4xx_hal.h"

#include "epaper.h"
#include "logger.h"

SPI_HandleTypeDef epaper_spi_handle;

void epaper_init() {
    log_write("initalizing epaper");
    // enable gpio ports
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // spi init
    // miso C11; mosi C12; clk C10
    log_write("initalizing epaper spi");
    epaper_spi_handle.Instance = SPI3;
    epaper_spi_handle.Init.Mode = SPI_MODE_MASTER;
    epaper_spi_handle.Init.Direction = SPI_DIRECTION_2LINES;
    epaper_spi_handle.Init.DataSize = SPI_DATASIZE_8BIT;
    epaper_spi_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    epaper_spi_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    epaper_spi_handle.Init.NSS = SPI_NSS_SOFT;
    epaper_spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    epaper_spi_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    epaper_spi_handle.Init.TIMode = SPI_TIMODE_DISABLE;
    epaper_spi_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    epaper_spi_handle.Init.CRCPolynomial = 7;
    epaper_spi_handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    epaper_spi_handle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&epaper_spi_handle) != HAL_OK)
    {
        log_write("in %s - Error initializing spi", __func__);
        while (1)
            ;
    }
    log_write("epaper spi OK");

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // epaper busy C0
    HAL_GPIO_WritePin(EPAPER_BUSY_GPIO_Port, EPAPER_BUSY_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = EPAPER_BUSY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPAPER_BUSY_GPIO_Port, &GPIO_InitStruct);

    // epaper reset C1
    HAL_GPIO_WritePin(EPAPER_RST_GPIO_Port, EPAPER_RST_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = EPAPER_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPAPER_RST_GPIO_Port, &GPIO_InitStruct);

    // epaper data/command B0
    HAL_GPIO_WritePin(EPAPER_DC_GPIO_Port, EPAPER_DC_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = EPAPER_DC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPAPER_DC_GPIO_Port, &GPIO_InitStruct);

    // epaper cs D2
    HAL_GPIO_WritePin(EPAPER_CS_GPIO_Port, EPAPER_CS_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = EPAPER_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPAPER_CS_GPIO_Port, &GPIO_InitStruct);

    DEV_Module_Init();
    EPD_1IN54_V2_Init();
    EPD_1IN54_V2_Clear();
    osDelay(500);
}