#include "stm32l4xx_hal.h"

#include "logger.h"
#include "sensor.h"
#include "bme68x.h"

#define SENSOR_CS_PIN_Pin GPIO_PIN_7
#define SENSOR_CS_PIN_GPIO_Port GPIOC
#define SPI_TIMEOUT 1000U

static SPI_HandleTypeDef spi;
static struct bme68x_dev sensor_dev;

BME68X_INTF_RET_TYPE app_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    if (HAL_I2C_Master_Transmit(&hi2c4, (dev_id << 1), &reg_addr, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        result = -1;
    }
    else if (HAL_I2C_Master_Receive(&hi2c4, (dev_id << 1) | 0x01, reg_data, len, HAL_MAX_DELAY) != HAL_OK)
    {
        result = -1;
    }
    else
    {
        result = 0;
    }

  return result;

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_RESET);

    SPI_HandleTypeDef *app_spi = (SPI_HandleTypeDef*)intf_ptr;
    log_write("before receive: %d", *reg_data);
    HAL_StatusTypeDef res = HAL_SPI_Receive(app_spi, length, SPI_TIMEOUT);
    log_write("received, read res: %d", res != HAL_OK);
    log_write("after receive: %d", *reg_data);

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_SET);
    return res != HAL_OK;
}

BME68X_INTF_RET_TYPE app_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_RESET);

    SPI_HandleTypeDef *app_spi = (SPI_HandleTypeDef*)intf_ptr;
    log_write("before transmit: %d", *reg_data);
    HAL_StatusTypeDef res = HAL_SPI_Transmit(app_spi, reg_data, length, SPI_TIMEOUT);
    log_write("transmitted, write res: %d", res != HAL_OK);
    log_write("after transmit: %d", *reg_data);

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_SET);
    return res != HAL_OK;
}

void app_delay(uint32_t period, void *intf_ptr)
{
    HAL_Delay(period);
}


void sensor_init()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = SENSOR_CS_PIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SENSOR_CS_PIN_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_SET);

    spi.Instance = SPI1;
    spi.Init.Mode = SPI_MODE_MASTER;
    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.DataSize = SPI_DATASIZE_4BIT;
    spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.Init.CRCPolynomial = 7;
    spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    spi.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;

    // sensor_dev.chip_id = 0x61;
    sensor_dev.variant_id = 0;
    sensor_dev.intf = BME68X_SPI_INTF;
    sensor_dev.intf_ptr = &spi;
    sensor_dev.read = app_spi_read;
    sensor_dev.write = app_spi_write;
    sensor_dev.delay_us = app_delay;
    sensor_dev.amb_temp = 24u;


    if (HAL_SPI_Init(&spi) != HAL_OK)
    {
        log_write("in %s - Error initializing SPI", __func__);
        while (1)
            ;
    }

    int8_t res = bme68x_init(&sensor_dev);
    if (res != BME68X_OK)
    {
        log_write("in %s - Error initializing sensor (%d)", __func__, res);
        while (1)
            ;
    }
}
