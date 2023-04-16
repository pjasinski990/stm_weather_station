#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "stm32l4xx_hal.h"
#include "cmsis_os2.h"

#include "logger.h"

#define OUT_UART USART2
#define OUT_BUFFER_SIZE 129U
#define TIMEOUT 1000U
#define BAUDRATE 115200U

static UART_HandleTypeDef uart;
static RTC_HandleTypeDef rtc;

// static osMutexId_t uart_mutex_id;
// static const osMutexAttr_t uart_mutex_attributes = {
//     "uart_mutex",
//     osMutexRobust,
//     NULL,
//     0U
// };

static char output_buffer[OUT_BUFFER_SIZE];

void log_init()
{
    uart.Instance = OUT_UART;
    uart.Init.BaudRate = BAUDRATE;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
    uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&uart) != HAL_OK)
    {
        while (1)
            ;
    }

    rtc.Instance = RTC;
    rtc.Init.HourFormat = RTC_HOURFORMAT_24;
    rtc.Init.AsynchPrediv = 127;
    rtc.Init.SynchPrediv = 255;
    rtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    rtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    rtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    rtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&rtc) != HAL_OK)
    {
        while (1)
            ;
    }

    // uart_mutex_id = osMutexNew(NULL);
    // if (uart_mutex_id == NULL)
    // {
    //     log_write("error creating uart_mutex");
    // }
    HAL_UART_Abort(&uart);
}

void log_write(const char *format, ...)
{
    // osMutexAcquire(uart_mutex_id, osWaitForever);
    memset(output_buffer, '\0', OUT_BUFFER_SIZE);

    RTC_TimeTypeDef rtc_time;
    HAL_RTC_GetTime(&rtc, &rtc_time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&rtc, NULL, RTC_FORMAT_BIN); // must be called to unlock time values

    uint16_t millis = (uint16_t)(999.0 - 1000.0 / rtc_time.SecondFraction * rtc_time.SubSeconds);
    snprintf(output_buffer, OUT_BUFFER_SIZE, "[%02d:%02d:%02d:%03d] ", rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, millis);

    va_list argp;
    va_start(argp, format);
    char *after_date_q = output_buffer + strlen(output_buffer);
    uint32_t bytes_left = OUT_BUFFER_SIZE - strlen(output_buffer) - 1;
    vsnprintf(after_date_q, bytes_left, format, argp);
    va_end(argp);

    // buffer not filled completely, add newline at the end of the string
    if (strlen(output_buffer) < OUT_BUFFER_SIZE - 3)
    {
        output_buffer[strlen(output_buffer)] = '\r';
        output_buffer[strlen(output_buffer) + 1] = '\n';
    }
    // buffer overflowing, add newline at the end of the buffer
    else
    {
        output_buffer[OUT_BUFFER_SIZE - 3] = '\r';
        output_buffer[OUT_BUFFER_SIZE - 2] = '\n';
        output_buffer[OUT_BUFFER_SIZE - 1] = '\0';
    }

    HAL_UART_Transmit(&uart, (uint8_t *)output_buffer, OUT_BUFFER_SIZE, TIMEOUT);
    // osMutexRelease(uart_mutex_id);
}
