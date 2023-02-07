#include <stdarg.h>
#include <stdio.h>

#include "stm32l4xx_hal.h"

#include "logger.h"

#define OUT_UART USART2
#define OUT_BUFFER_SIZE 256
#define BAUDRATE 115200

static UART_HandleTypeDef uart;
static uint8_t output_buffer[OUT_BUFFER_SIZE];

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
    }
}

void log_write(const char *format, ...) {
    va_list argp;
    va_start(argp, format);
    vsnprintf(output_buffer, OUT_BUFFER_SIZE, format, argp);
    HAL_UART_Transmit(&uart, output_buffer, OUT_BUFFER_SIZE, 1);
    va_end(argp);
}
