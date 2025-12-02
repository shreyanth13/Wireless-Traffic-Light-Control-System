#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#define RX_BUFFER_SIZE 256

uint8_t rxByte;
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint16_t rxIndex = 0;
volatile uint8_t packetReady = 0;

volatile uint8_t gScheduleValid = 0;
uint8_t gStageNum = 0;
uint32_t gStageTimes_ms[8] = {0};
uint32_t gStagesPattern[8] = {0};
volatile uint8_t gCurrentStageIdx = 0;

SemaphoreHandle_t xUART_Mutex = NULL;
SemaphoreHandle_t xData_Mutex = NULL;

osThreadId_t packetTaskHandle = NULL;
osThreadId_t ledTaskHandle    = NULL;

extern UART_HandleTypeDef huart6;
extern void StartPacketProcessor(void *argument);
extern void StartLEDController(void *argument);

extern void InitStageTimer(void);

void SystemClock_Config(void);
static void PrintUART(const char *msg);
void LED_Pins_Init(void);
void ApplyStageToLEDs(uint32_t pattern);

static void UART_IRQ_Priority_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART6_UART_Init();
    LED_Pins_Init();

    UART_IRQ_Priority_Config();

    osKernelInitialize();
    xUART_Mutex = xSemaphoreCreateMutex();
    xData_Mutex = xSemaphoreCreateMutex();

    PrintUART("\r\nSTM32 Traffic Light Packet Decoder + LED Controller \r\n");

    HAL_UART_Receive_IT(&huart6, &rxByte, 1);

    const osThreadAttr_t packetTask_attributes = {
        .name = "packetTask",
        .stack_size = 4096,
        .priority = (osPriority_t) osPriorityHigh
    };
    const osThreadAttr_t ledTask_attributes = {
        .name = "ledTask",
        .stack_size = 2048,
        .priority = (osPriority_t) osPriorityNormal
    };

    packetTaskHandle = osThreadNew(StartPacketProcessor, NULL, &packetTask_attributes);
    ledTaskHandle    = osThreadNew(StartLEDController, NULL, &ledTask_attributes);

    InitStageTimer();

    osKernelStart();

    while (1) { }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        if (rxIndex < RX_BUFFER_SIZE)
        {
            rxBuffer[rxIndex++] = rxByte;

            if (rxIndex >= 3 && rxBuffer[rxIndex - 3] == 0x45 && rxBuffer[rxIndex - 2] == 0x4F && rxBuffer[rxIndex - 1] == 0x46)
            {
                packetReady = 1;
            }
        }

        HAL_UART_Receive_IT(&huart6, &rxByte, 1);
    }
}

static void PrintUART(const char *msg)
{
    if (xUART_Mutex != NULL)
    {
        if (xSemaphoreTake(xUART_Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            xSemaphoreGive(xUART_Mutex);
            return;
        }
    }
    HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void LED_Pins_Init(void)
{
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,GPIO_PIN_RESET);
}

void ApplyStageToLEDs(uint32_t p)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, ((p >> 11) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, ((p >> 10) & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, ((p >> 9)  & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, ((p >> 8)  & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, ((p >> 7)  & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, ((p >> 6)  & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void UART_IRQ_Priority_Config(void)
{
    HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 12;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
