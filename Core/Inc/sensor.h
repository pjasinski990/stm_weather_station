#ifndef SENSOR_H
#define SENSOR_H

#define SENSOR_CS_Pin GPIO_PIN_7
#define SENSOR_CS_GPIO_Port GPIOC
#define SPI_TIMEOUT 1000U

extern SPI_HandleTypeDef sensor_spi_handle;
extern const float sensor_temperature_offset;

typedef struct
{
    float timestamp;
    float iaq;
    float temperature;
    float humidity;
    float pressure;
} sensor_reading_t;

void sensor_init();
void start_sensor_loop_task(void *arg);

void store_new_reading(sensor_reading_t data);
osStatus_t get_new_reading(sensor_reading_t *data_out);

#endif
