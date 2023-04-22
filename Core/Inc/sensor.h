#ifndef SENSOR_H
#define SENSOR_H

#define SENSOR_CS_Pin GPIO_PIN_7
#define SENSOR_CS_GPIO_Port GPIOC
#define SPI_TIMEOUT 1000U

static SPI_HandleTypeDef sensor_spi_handle;
static const float sensor_temperature_offset = 0.0f;

void sensor_init();
void start_sensor_loop_task(void *arg);

#endif
