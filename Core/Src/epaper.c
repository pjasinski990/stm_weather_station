#include "stm32l4xx_hal.h"

#include <stdlib.h>

#include "cmsis_os2.h"
#include "epaper.h"
#include "sensor.h"
#include "logger.h"
#include "EPD_1in54_V2.h"
#include "GUI_Paint.h"

#define EPAPER_N_PIXELS (((EPD_1IN54_V2_WIDTH % 8 == 0)? (EPD_1IN54_V2_WIDTH / 8 ): (EPD_1IN54_V2_WIDTH / 8 + 1)) * EPD_1IN54_V2_HEIGHT)
#define EPAPER_REFRESH_DELAY 20000

SPI_HandleTypeDef epaper_spi_handle;

static UBYTE *frame_buffer;

void epaper_init() {
    log_write("initalizing epaper");
    // enable gpio ports
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // spi init
    // miso C11; mosi C12; clk C10
    log_write("initalizing epaper spi");
    epaper_spi_handle.Instance = SPI3;
    epaper_spi_handle.Init.Mode = SPI_MODE_MASTER;
    epaper_spi_handle.Init.Direction = SPI_DIRECTION_2LINES;
    epaper_spi_handle.Init.DataSize = SPI_DATASIZE_8BIT;
    epaper_spi_handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    epaper_spi_handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    epaper_spi_handle.Init.NSS = SPI_NSS_SOFT;
    epaper_spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    epaper_spi_handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    epaper_spi_handle.Init.TIMode = SPI_TIMODE_DISABLE;
    epaper_spi_handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    epaper_spi_handle.Init.CRCPolynomial = 7;
    epaper_spi_handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    epaper_spi_handle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&epaper_spi_handle) != HAL_OK)
    {
        log_write("in %s - Error initializing spi", __func__);
        while (1)
            ;
    }
    log_write("epaper spi OK");

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // epaper busy C0
    HAL_GPIO_WritePin(EPAPER_BUSY_GPIO_Port, EPAPER_BUSY_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = EPAPER_BUSY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPAPER_BUSY_GPIO_Port, &GPIO_InitStruct);

    // epaper reset C1
    HAL_GPIO_WritePin(EPAPER_RST_GPIO_Port, EPAPER_RST_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = EPAPER_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPAPER_RST_GPIO_Port, &GPIO_InitStruct);

    // epaper data/command B0
    HAL_GPIO_WritePin(EPAPER_DC_GPIO_Port, EPAPER_DC_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = EPAPER_DC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPAPER_DC_GPIO_Port, &GPIO_InitStruct);

    // epaper cs D2
    HAL_GPIO_WritePin(EPAPER_CS_GPIO_Port, EPAPER_CS_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = EPAPER_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPAPER_CS_GPIO_Port, &GPIO_InitStruct);

    frame_buffer = malloc(sizeof(UBYTE) * EPAPER_N_PIXELS);
    if (frame_buffer == NULL) {
        log_write("error allocating %d bytes for epaper frame buffer", sizeof(UBYTE) * EPAPER_N_PIXELS);
        while (1)
            ;
    }
    // create, select and clear frame buffer
    Paint_NewImage(frame_buffer, EPD_1IN54_V2_WIDTH, EPD_1IN54_V2_HEIGHT, 270, WHITE);
    Paint_SelectImage(frame_buffer);
    Paint_Clear(WHITE);
    DEV_Module_Init();

    log_write("epaper OK");
    log_write("");
}

void epaper_deinit() {
    free(frame_buffer);
    frame_buffer = NULL;
}

// wake epaper for writing
void epaper_begin() {
    log_write("epaper init for writing");
    EPD_1IN54_V2_Init();
}

void epaper_begin_partial() {
    log_write("epaper partial display activate");
    EPD_1IN54_V2_DisplayPartBaseImage(frame_buffer);
    EPD_1IN54_V2_Init_Partial();
}

void epaper_clear() {
    log_write("epaper clear");
    EPD_1IN54_V2_Clear();
    DEV_Delay_ms(500);
}

// put epaper to sleep and power off
void epaper_end() {
    log_write("epaper sleep");
    EPD_1IN54_V2_Sleep();
    log_write("close vcc, epaper entering low power");
    DEV_Module_Exit();
}

void epaper_update() {
    log_write("%s", "refreshing epaper");
    EPD_1IN54_V2_Display(frame_buffer);
}

void epaper_draw_text(uint8_t xbegin, uint8_t ybegin, sFONT *font, const char *text) {
    log_write("display text: %s", text);
    Paint_DrawString_EN(xbegin, ybegin, text, font, WHITE, BLACK);
}

#include "weather_icons_32x32.h"
void start_epaper_loop_task(void *arg) {
    log_write("starting epaper task");

    int border = 0;
    int n_icons = 4;
    int icon_dim = 32;
    int icon_padding_right = 0;
    int y_space = (EPD_1IN54_V2_HEIGHT - 2*border - icon_dim*(n_icons)) / (n_icons - 1) + icon_dim;
    int values_x = border + icon_dim + icon_padding_right;
    sFONT font = Font12;

    while (1) {
        epaper_begin();
        Paint_Clear(WHITE);

        // draw icons
        Paint_DrawBitMap_Paste(epd_bitmap_temperature_32x32, border, border + 0 * y_space, icon_dim, icon_dim, 0);
        Paint_DrawBitMap_Paste(epd_bitmap_humidity_32x32,    border, border + 1 * y_space, icon_dim, icon_dim, 0);
        Paint_DrawBitMap_Paste(epd_bitmap_pressure_32x32,    border, border + 2 * y_space, icon_dim, icon_dim, 0);
        Paint_DrawBitMap_Paste(epd_bitmap_iaq_32x32,         border, border + 3 * y_space, icon_dim, icon_dim, 0);

        uint8_t has_new_data = 0;
        sensor_reading_t data;
        osStatus_t status = get_new_reading(&data);
        while (status == osOK) {
            status = get_new_reading(&data);
            has_new_data = 1;
        }
        if (has_new_data) {
            log_write("new data from sensor");
            // write sensor data
            Paint_DrawNumDecimals(values_x, icon_dim - font.Height + border + 0 * y_space, data.temperature, &Font12, 1, BLACK, WHITE);
            Paint_DrawNumDecimals(values_x, icon_dim - font.Height + border + 1 * y_space, data.humidity, &Font12, 1, BLACK, WHITE);
            Paint_DrawNumDecimals(values_x, icon_dim - font.Height + border + 2 * y_space, data.pressure / 100, &Font12, 1, BLACK, WHITE);
            Paint_DrawNumDecimals(values_x, icon_dim - font.Height + border + 3 * y_space, data.iaq, &Font12, 1, BLACK, WHITE);
        }

        epaper_update();
        epaper_end();
        osDelay(EPAPER_REFRESH_DELAY);
    }
}
