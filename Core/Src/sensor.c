#include <string.h>

#include "stm32l4xx_hal.h"

#include "logger.h"
#include "sensor.h"
#include "bme68x.h"
#include "bsec_interface.h"

#define SENSOR_CS_PIN_Pin GPIO_PIN_7
#define SENSOR_CS_PIN_GPIO_Port GPIOC
#define SPI_TIMEOUT 1000U

static SPI_HandleTypeDef spi;
static struct bme68x_dev sensor_dev;

BME68X_INTF_RET_TYPE app_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    SPI_HandleTypeDef *app_spi = (SPI_HandleTypeDef*)intf_ptr;
    uint8_t command = reg_addr | 0x80;

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef res;
    res = HAL_SPI_Transmit(app_spi, &command, 1, SPI_TIMEOUT);
    while (app_spi->State == HAL_SPI_STATE_BUSY)
        ;
    res = HAL_SPI_Receive(app_spi, reg_data, length, SPI_TIMEOUT);
    while (app_spi->State == HAL_SPI_STATE_BUSY)
        ;

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_SET);
    return res != HAL_OK;
}

BME68X_INTF_RET_TYPE app_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    SPI_HandleTypeDef *app_spi = (SPI_HandleTypeDef*)intf_ptr;
    uint8_t command = reg_addr & 0x7F;

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef res;
    res = HAL_SPI_Transmit(app_spi, &command, 1, SPI_TIMEOUT);
    res = HAL_SPI_Transmit(app_spi, (uint8_t *)reg_data, length, SPI_TIMEOUT);

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_SET);
    return res != HAL_OK;
}

void app_delay(uint32_t period, void *intf_ptr)
{
    HAL_Delay(period);
}


void sensor_init()
{
    // enable gpio ports
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // spi init
    // miso C7
    // mosi C3
    // clk B10
    log_write("Initializing SPI...");
    spi.Instance = SPI2;
    spi.Init.Mode = SPI_MODE_MASTER;
    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.DataSize = SPI_DATASIZE_8BIT;
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
    log_write("OK");

    // cs C7
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = SENSOR_CS_PIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SENSOR_CS_PIN_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_SET);

    // sensor init
    log_write("Initializing sensor...");
    bsec_init();

    sensor_dev.variant_id = 0;
    sensor_dev.intf = BME68X_SPI_INTF;
    sensor_dev.intf_ptr = &spi;
    sensor_dev.read = app_spi_read;
    sensor_dev.write =  app_spi_write;
    sensor_dev.delay_us = app_delay;
    sensor_dev.amb_temp = 24u;

    int8_t res = bme68x_init(&sensor_dev);

    if (res != BME68X_OK)
    {
        log_write("in %s - Error initializing sensor (%d)", __func__, res);
        while (1)
            ;
    }
    log_write("OK");
}
