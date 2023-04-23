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

#include "fonts.h"

extern SPI_HandleTypeDef epaper_spi_handle;

void epaper_init();
void epaper_deinit();
void epaper_begin();
void epaper_begin_partial();
void epaper_end();

void epaper_draw_text(uint8_t xbegin, uint8_t ybegin, sFONT *font, const char *text);

#endif
