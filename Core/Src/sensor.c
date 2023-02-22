#include <string.h>

#include "stm32l4xx_hal.h"

#include "logger.h"
#include "sensor.h"
#include "bme68x.h"
#include "bsec_interface.h"
#include "bsec_integration.h"

#define SENSOR_CS_PIN_Pin GPIO_PIN_7
#define SENSOR_CS_PIN_GPIO_Port GPIOC
#define SPI_TIMEOUT 1000U

static SPI_HandleTypeDef spi;
static struct bme68x_dev sensor_dev;
// static float bme68x_temperature_offset_g = 0.0f;


int64_t get_timestamp_us();
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, float gas_percentage, bsec_library_return_t bsec_status,
     float static_iaq, float stabStatus, float runInStatus, float co2_equivalent, float breath_voc_equivalent);
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer);
void state_save(const uint8_t *state_buffer, uint32_t length);
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer);


BME68X_INTF_RET_TYPE app_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    SPI_HandleTypeDef *app_spi = (SPI_HandleTypeDef*)intf_ptr;
    app_spi = &spi;
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
    app_spi = &spi;
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
    // sensor_dev.variant_id = 0;
    // sensor_dev.intf = BME68X_SPI_INTF;
    // sensor_dev.intf_ptr = &spi;
    // sensor_dev.read = app_spi_read;
    // sensor_dev.write =  app_spi_write;
    // sensor_dev.delay_us = app_delay;
    // sensor_dev.amb_temp = 24u;

    // int8_t res = bme68x_init(&sensor_dev);

    // if (res != BME68X_OK)
    // {
    //     log_write("in %s - Error initializing BME device (%d)", __func__, res);
    //     while (1)
    //         ;
    // }

    // res = bsec_init();
    // if (res != BSEC_OK)
    // {
    //     log_write("in %s - Error initializing BSEC driver (%d)", __func__, res);
    //     while (1)
    //         ;
    // }

    return_values_init ret;
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, app_spi_write, app_spi_read, app_delay, state_load, config_load);
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
    log_write("OK");

    log_write("Looping:");
    int64_t time_stamp = 0;
    int64_t time_stamp_interval_ms = 0;
    uint32_t save_intvl = 5u;
    
    /* Allocate enough memory for up to BSEC_MAX_PHYSICAL_SENSOR physical inputs*/
    bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];
    
    /* Number of inputs to BSEC */
    uint8_t num_bsec_inputs = 0;
    
    /* BSEC sensor settings struct */
    bsec_bme_settings_t sensor_settings;
    
    /* Save state variables */
    uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
    uint32_t bsec_state_len = 0;
    uint32_t n_samples = 0;
    
    bsec_library_return_t bsec_status = BSEC_OK;

    bsec_iot_loop(app_delay, get_timestamp_us, output_ready, state_save, /* save intvl */ 5);
}

/*!
 * @brief           Capture the system time in microseconds
 *
 * @return          system_current_time    current system timestamp in microseconds
 */
int64_t get_timestamp_us()
{
    int64_t system_current_time = 0;
    // ...
    // Please insert system specific function to retrieve a timestamp (in microseconds)
    // ...
    return system_current_time;
}

/*!
 * @brief           Handling of the ready outputs
 *
 * @param[in]       timestamp               time in milliseconds
 * @param[in]       iaq                     IAQ
 * @param[in]       iaq_accuracy            IAQ accuracy
 * @param[in]       static_iaq              static IAQ
 * @param[in]       temperature             raw temperature
 * @param[in]       humidity                raw humidity
 * @param[in]       temperature             temperature
 * @param[in]       humidity                humidity
 * @param[in]       pressure                pressure
 * @param[in]       gas                     raw gas
 * @param[in]       gas_percentage          gas percentage
 * @param[in]       co2_equivalent          CO2 equivalent
 * @param[in]       breath_voc_equivalent   breath VOC equivalent
 * @param[in]       stabStatus              stabilization status
 * @param[in]       runInStatus             run in status
 * @param[in]       bsec_status             value returned by the bsec_do_steps() call
 *
 * @return          none
 */
void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
                  float pressure, float raw_temperature, float raw_humidity, float gas, float gas_percentage, bsec_library_return_t bsec_status,
                  float static_iaq, float stabStatus, float runInStatus, float co2_equivalent, float breath_voc_equivalent)
{
    log_write("iaq %f, temp %f, humidity: %f", iaq, temperature, humidity);
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
 
/*!
 * @brief           Load library config from non-volatile memory
 *
 * @param[in,out]   config_buffer    buffer to hold the loaded state string
 * @param[in]       n_buffer        size of the allocated state buffer
 *
 * @return          number of bytes copied to config_buffer
 */
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer)
{
    // ...
    // Load a library config from non-volatile memory, if available.
    //
    // Return zero if loading was unsuccessful or no config was available, 
    // otherwise return length of loaded config string.
    // ...
    return 0;
}