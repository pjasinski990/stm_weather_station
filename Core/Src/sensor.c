#include <string.h>

#include "stm32l4xx_hal.h"

#include "logger.h"
#include "sensor.h"
#include "bme68x.h"

#define SENSOR_CS_PIN_Pin GPIO_PIN_7
#define SENSOR_CS_PIN_GPIO_Port GPIOC
#define SPI_TIMEOUT 1000U

static SPI_HandleTypeDef spi;
static struct bme68x_dev sensor_dev;

uint8_t GTXBuffer[512], GRXBuffer[512];
int8_t SensorAPI_SPIx_Read(uint8_t subaddress, uint8_t *pBuffer, uint16_t ReadNumbr, void *intf_ptr)
{
    GTXBuffer[0] = subaddress | 0x80;

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_RESET);

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, ReadNumbr+1, SPI_TIMEOUT); // timeout 1000msec;
    log_write("before receive: %d", *pBuffer);
    HAL_StatusTypeDef res = HAL_SPI_TransmitReceive(&spi, GTXBuffer, GRXBuffer, ReadNumbr+1, SPI_TIMEOUT); // timeout 1000msec;
    while(spi.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
    log_write("after receive: %d", GRXBuffer[0]);
    log_write("after receive: %d", GRXBuffer[1]);
    log_write("");

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_SET);
    memcpy(pBuffer, GRXBuffer+1, ReadNumbr);

    return res != HAL_OK;
}

int8_t SensorAPI_SPIx_Write(uint8_t subaddress, uint8_t *pBuffer, uint16_t WriteNumbr, void *intf_ptr)
{
    GTXBuffer[0] = subaddress & 0x7F;
    memcpy(&GTXBuffer[1], pBuffer, WriteNumbr);

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_RESET);

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, WriteNumbr+1, SPI_TIMEOUT); // send register address + write data
    HAL_StatusTypeDef res = HAL_SPI_Transmit(&spi, GTXBuffer, WriteNumbr+1, SPI_TIMEOUT); // send register address + write data
    while(spi.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_SET);

    return res != HAL_OK;
}


BME68X_INTF_RET_TYPE app_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    SPI_HandleTypeDef *app_spi = (SPI_HandleTypeDef*)intf_ptr;
    log_write("before receive: %d, reg addr is: %d", *reg_data, reg_addr);
    // uint8_t command = reg_addr << 1 | 1;

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_RESET);
    // HAL_Delay(100);

    HAL_StatusTypeDef res = HAL_SPI_Transmit(app_spi, &reg_addr, 1, SPI_TIMEOUT);
    while(app_spi->State == HAL_SPI_STATE_BUSY);

    res = HAL_SPI_Receive(app_spi, reg_data, length, SPI_TIMEOUT);
    while(app_spi->State == HAL_SPI_STATE_BUSY);

    log_write("received, read res: %d", res != HAL_OK);
    log_write("after receive: %d", *reg_data);

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_SET);
    return res != HAL_OK;
}

BME68X_INTF_RET_TYPE app_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    SPI_HandleTypeDef *app_spi = (SPI_HandleTypeDef*)intf_ptr;
    log_write("before transmit: %d", *reg_data);
    // uint8_t command = reg_addr << 1 | 0;

    HAL_GPIO_WritePin(SENSOR_CS_PIN_GPIO_Port, SENSOR_CS_PIN_Pin, GPIO_PIN_RESET);
    // HAL_Delay(100);

    HAL_StatusTypeDef res = HAL_SPI_Transmit(app_spi, &reg_addr, 1, SPI_TIMEOUT);
    res = HAL_SPI_Transmit(app_spi, (uint8_t *)reg_data, length, SPI_TIMEOUT);
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
    spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLED;


    sensor_dev.variant_id = 0;
    sensor_dev.intf = BME68X_SPI_INTF;
    sensor_dev.intf_ptr = &spi;
    sensor_dev.read = app_spi_read;
    sensor_dev.write =  app_spi_write;
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
