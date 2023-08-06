#include "main.h"
#include "cmsis_os2.h"

#include "logger.h"
#include "sensor.h"
#include "epaper.h"
#include "EPD_Test.h"
#ifdef USE_FREERTOS
#include "FreeRTOSConfig.h"
#endif

void SystemClock_Config(void);

osThreadId_t main_task_handle;
const osThreadAttr_t main_task_attributes = {
    .name = "main_task",
    .stack_size = 512 * 2,
    .priority = (osPriority_t)osPriorityNormal,
};
osThreadId_t sensor_task_handle;
const osThreadAttr_t sensor_task_attributes = {
    .name = "sensor_task",
    .stack_size = 512 * 8,
    .priority = (osPriority_t)osPriorityNormal,
};
osThreadId_t epaper_task_handle;
const osThreadAttr_t epaper_task_attributes = {
    .name = "epaper_task",
    .stack_size = 512 * 2,
    .priority = (osPriority_t)osPriorityHigh,
};

void main_task(void *arg) {
    log_init();
    log_write("***********************************");
    log_write("|        Device starting up       |");
    log_write("***********************************");
    log_write("");

    sensor_task_handle = osThreadNew(start_sensor_loop_task, NULL, &sensor_task_attributes);
    if (sensor_task_handle == NULL) {
        log_write("Error creating sensor task!");
    }
    epaper_task_handle = osThreadNew(start_epaper_loop_task, NULL, &epaper_task_attributes);
    if (sensor_task_handle == NULL) {
        log_write("Error creating epaper task!");
    }

    while(1) {
        osDelay(osWaitForever);
    }
}

int main(void)
{
    // NVIC_SetPriority(SVCall_IRQn, 7U);
    // NVIC_SetPriority(SysTick_IRQn, 7U);
    // NVIC_SetPriority(PendSV_IRQn, 6U);

    HAL_Init();

    SystemClock_Config();
    osKernelInitialize();

    main_task_handle = osThreadNew(main_task, NULL, &main_task_attributes);
    osKernelStart();
    while(1) {
        osDelay(osWaitForever);
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_IncTick();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    log_write("Wrong parameters value: file %s on line %d\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */
