#ifndef EPAPER_H
#define EPAPER_H

#define EPAPER_BUSY_GPIO_Port GPIOC
#define EPAPER_RST_GPIO_Port GPIOC
#define EPAPER_DC_GPIO_Port GPIOB
#define EPAPER_CS_GPIO_Port GPIOD

#define EPAPER_BUSY_Pin GPIO_PIN_0
#define EPAPER_RST_Pin GPIO_PIN_1
#define EPAPER_DC_Pin GPIO_PIN_0
#define EPAPER_CS_Pin GPIO_PIN_2

extern SPI_HandleTypeDef epaper_spi_handle;

void epaper_init();

#endif
