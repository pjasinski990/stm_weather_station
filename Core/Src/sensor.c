#include <string.h>

#include "stm32l4xx_hal.h"

#include "cmsis_os2.h"
#include "logger.h"
#include "sensor.h"
#include "bme68x.h"
#include "bsec_interface.h"
#include "bsec_integration.h"
#include "generic_18v_300s_4d/bsec_serialized_configurations_iaq.h"

SPI_HandleTypeDef sensor_spi_handle;
osMessageQueueId_t sensor_data_q;
const float sensor_temperature_offset = 0.0f;

int64_t get_timestamp_us();
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, float gas_percentage, bsec_library_return_t bsec_status,
     float static_iaq, float stabStatus, float runInStatus, float co2_equivalent, float breath_voc_equivalent);
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer);
void state_save(const uint8_t *state_buffer, uint32_t length);
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer);

BME68X_INTF_RET_TYPE app_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    SPI_HandleTypeDef *app_spi = &sensor_spi_handle;
    uint8_t command = reg_addr | 0x80;

    HAL_GPIO_WritePin(SENSOR_CS_GPIO_Port, SENSOR_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef res;
    res = HAL_SPI_Transmit(app_spi, &command, 1, SPI_TIMEOUT);
    while (app_spi->State == HAL_SPI_STATE_BUSY)
        ;
    res = HAL_SPI_Receive(app_spi, reg_data, length, SPI_TIMEOUT);
    while (app_spi->State == HAL_SPI_STATE_BUSY)
        ;

    HAL_GPIO_WritePin(SENSOR_CS_GPIO_Port, SENSOR_CS_Pin, GPIO_PIN_SET);
    return res != HAL_OK;
}

BME68X_INTF_RET_TYPE app_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    SPI_HandleTypeDef *app_spi = &sensor_spi_handle;
    uint8_t command = reg_addr & 0x7F;

    HAL_GPIO_WritePin(SENSOR_CS_GPIO_Port, SENSOR_CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef res;
    res = HAL_SPI_Transmit(app_spi, &command, 1, SPI_TIMEOUT);
    res = HAL_SPI_Transmit(app_spi, (uint8_t *)reg_data, length, SPI_TIMEOUT);

    HAL_GPIO_WritePin(SENSOR_CS_GPIO_Port, SENSOR_CS_Pin, GPIO_PIN_SET);
    return res != HAL_OK;
}

void app_delay(uint32_t period_us, void *intf_ptr)
{
    uint32_t period_millis = period_us / 1000u;
    if (period_millis > 0) {
        osDelay(period_millis);
    }
}


void sensor_init()
{
    // enable gpio ports
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // spi init
    // miso C7; mosi C3; clk B10
    log_write("initializing sensor spi");
    sensor_spi_handle.Instance = SPI2;
    sensor_spi_handle.Init.Mode = SPI_MODE_MASTER;
    sensor_spi_handle.Init.Direction = SPI_DIRECTION_2LINES;
    sensor_spi_handle.Init.DataSize = SPI_DATASIZE_8BIT;
    sensor_spi_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    sensor_spi_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    sensor_spi_handle.Init.NSS = SPI_NSS_SOFT;
    sensor_spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    sensor_spi_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    sensor_spi_handle.Init.TIMode = SPI_TIMODE_DISABLE;
    sensor_spi_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    sensor_spi_handle.Init.CRCPolynomial = 7;
    sensor_spi_handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    sensor_spi_handle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&sensor_spi_handle) != HAL_OK)
    {
        log_write("in %s - Error initializing spi", __func__);
        while (1)
            ;
    }
    log_write("sensor spi OK");

    // cs C7
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_GPIO_WritePin(SENSOR_CS_GPIO_Port, SENSOR_CS_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = SENSOR_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SENSOR_CS_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SENSOR_CS_GPIO_Port, SENSOR_CS_Pin, GPIO_PIN_SET);

    // sensor init
    log_write("initializing sensor");
    return_values_init ret;
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, sensor_temperature_offset, app_spi_write, app_spi_read, app_delay, state_load, config_load);
    if (ret.bme68x_status)
    {
        log_write("in %s - Error initializing BME device (%d)", __func__, ret.bme68x_status);
        while (1)
            ;
    }
    else if (ret.bsec_status)
    {
        log_write("in %s - Error initializing BSEC driver (%d)", __func__, ret.bsec_status);
        while (1)
            ;
    }

    osMessageQueueAttr_t sensor_data_q_attr = {
        .name = "sensor_data_queue"
    };
    sensor_data_q = osMessageQueueNew(16, sizeof(sensor_reading_t), &sensor_data_q_attr);
    if (sensor_data_q == NULL) {
        log_write("ERROR: sensor data queue could not be created");
        while (1);
    }

    log_write("sensor OK");
    log_write("");
}

int64_t get_timestamp_us()
{
    return (int64_t)(HAL_GetTick() * 1e3);
}

void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
                  float pressure, float raw_temperature, float raw_humidity, float gas, float gas_percentage, bsec_library_return_t bsec_status,
                  float static_iaq, float stabStatus, float runInStatus, float co2_equivalent, float breath_voc_equivalent)
{
    UNUSED(iaq_accuracy);
    UNUSED(raw_temperature);
    UNUSED(raw_humidity);
    UNUSED(gas);
    UNUSED(gas_percentage);
    UNUSED(static_iaq);
    UNUSED(stabStatus);
    UNUSED(runInStatus);
    UNUSED(co2_equivalent);
    UNUSED(breath_voc_equivalent);
    if (bsec_status != BSEC_OK) {
        log_write("ERROR - bsec status is %d", bsec_status);
        while (1)
            ;
    }
    log_write("[%.3f sec] - STATS:", (float)timestamp / 1e9);
    log_write("\t\t* IAQ %.2f", iaq);
    log_write("\t\t* TEMPERATURE %.2f", temperature);
    log_write("\t\t* HUMIDITY: %.2f", humidity);
    log_write("\t\t* PRESSURE: %.2f", pressure);
    log_write("\t\t* GAS: %.2f", gas);
    log_write("\t\t* CO2: %.2f", co2_equivalent);
    log_write("\t\t* BREATH_VOC: %.2f", breath_voc_equivalent);

    sensor_reading_t new_reading = {
        .timestamp = timestamp,
        .iaq = iaq,
        .temperature = temperature,
        .humidity = humidity,
        .pressure = pressure
    };
    store_new_reading(new_reading);
}

/*!
 * @brief           Load previous library state from non-volatile memory
 *
 * @param[in,out]   state_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to state_buffer
 */
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer)
{
    // ...
    // Load a previous library state from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no state was available, 
    // otherwise return length of loaded state string.
    // ...
    return 0;
}

/*!
 * @brief           Save library state to non-volatile memory
 *
 * @param[in]       state_buffer    buffer holding the state to be stored
 * @param[in]       length          length of the state string to be stored
 *
 * @return          none
 */
void state_save(const uint8_t *state_buffer, uint32_t length)
{
    // ...
    // Save the string some form of non-volatile memory, if possible.
    // ...
}
 
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    memcpy(config_buffer, bsec_config_iaq, sizeof(bsec_config_iaq));
    return sizeof(bsec_config_iaq);
}

void start_sensor_loop_task(void *arg) {
    uint32_t save_intvl = 5u;
    log_write("starting sensor loop:");
    bsec_iot_loop(app_delay, get_timestamp_us, output_ready, state_save, save_intvl);
}

void store_new_reading(sensor_reading_t data) {
    osStatus_t status = osMessageQueuePut(sensor_data_q, &data, 0, 0);
    if (status != osOK) {
        log_write("WARNING: sensor data queue store error");
    }
}

osStatus_t get_new_reading(sensor_reading_t *data_out) {
    osStatus_t status = osMessageQueueGet(sensor_data_q, data_out, NULL, 0);
    return status;
}
