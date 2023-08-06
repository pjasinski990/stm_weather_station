#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "stm32l4xx_hal.h"
#include "cmsis_os2.h"

#include "logger.h"

#define OUT_UART USART2
#define OUT_BUFFER_SIZE 255U
#define TIMEOUT 1000U
#define BAUDRATE 115200U

UART_HandleTypeDef uart;
RTC_HandleTypeDef rtc;
char output_buffer[OUT_BUFFER_SIZE];

osMutexId_t uart_mutex_id;

int _write(int file, char *data, int len)
{
    if (file != 1 && file != 2)
    {
        return -1;
    }
    if (osMutexAcquire(uart_mutex_id, osWaitForever) == osOK)
    {
        HAL_UART_Transmit(&uart, (uint8_t *)data, len, TIMEOUT);
        osMutexRelease(uart_mutex_id);
    }
    return len;
}

static void uart_clear_terminal()
{
    char buff[] = {27, '[', '2', 'J'};
    _write(1, buff, 4);
}

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

    uart_mutex_id = osMutexNew(NULL);
    if (uart_mutex_id == NULL)
    {
        log_write("error creating uart_mutex");
        while (1)
            ;
    }
    uart_clear_terminal();
    log_write("uart init ok, handle is %d", uart);
}

void log_write(const char *format, ...)
{
    if (osMutexAcquire(uart_mutex_id, osWaitForever) == osOK)
    {
        RTC_TimeTypeDef rtc_time;
        HAL_RTC_GetTime(&rtc, &rtc_time, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&rtc, NULL, RTC_FORMAT_BIN); // must be called to unlock time values

        uint16_t millis = (uint16_t)(999.0 - 1000.0 / rtc_time.SecondFraction * rtc_time.SubSeconds);
        int prefix_len = snprintf(output_buffer, OUT_BUFFER_SIZE, "[%02d:%02d:%02d:%03d] ", rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds, millis);

        va_list argp;
        va_start(argp, format);
        int message_len = vsnprintf(output_buffer + prefix_len, OUT_BUFFER_SIZE - prefix_len, format, argp);
        va_end(argp);

        // ensure we don't overflow the buffer
        int total_len = prefix_len + message_len;
        if (total_len > OUT_BUFFER_SIZE - 3)
        {
            total_len = OUT_BUFFER_SIZE - 3;
        }
        output_buffer[total_len] = '\r';
        output_buffer[total_len + 1] = '\n';
        output_buffer[total_len + 2] = '\0';

        HAL_UART_Transmit(&uart, (uint8_t *)output_buffer, total_len + 3, TIMEOUT);
        osMutexRelease(uart_mutex_id);
    }
}
