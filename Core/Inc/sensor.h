#ifndef SENSOR_H
#define SENSOR_H

#define SENSOR_CS_Pin GPIO_PIN_7
#define SENSOR_CS_GPIO_Port GPIOC
#define SPI_TIMEOUT 1000U

extern SPI_HandleTypeDef sensor_spi_handle;
extern const float sensor_temperature_offset;

void sensor_init();

void start_sensor_loop_task(void *arg);

#endif
