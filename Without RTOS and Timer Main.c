#include "main.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

#define RX_BUFFER_SIZE 256



static uint8_t  rxByte;
static uint8_t  rxBuffer[RX_BUFFER_SIZE];
static uint16_t rxIndex = 0;
static volatile uint8_t packetReady = 0;



static volatile uint8_t  gScheduleValid = 0;
static uint8_t  gStageNum = 0;
static uint32_t gStageTimes_ms[8] = {0};
static uint32_t gStagesPattern[8] = {0};
static uint8_t  gCurrentStageIdx = 0;



void SystemClock_Config(void);
void ProcessPacket(void);
uint32_t bytes_to_uint32(uint8_t *data);
static void LED_Pins_Init(void);
static void ApplyStageToLEDs(uint32_t stagePattern);


int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  LED_Pins_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  const char *banner = "\r\nSTM32 Traffic Light Packet Decoder + LED Controller\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)banner, strlen(banner), HAL_MAX_DELAY);

  HAL_UART_Receive_IT(&huart1, &rxByte, 1);

  while (1)
  {

    if (packetReady)
    {
      packetReady = 0;
      ProcessPacket();
      rxIndex = 0;
      memset(rxBuffer, 0, RX_BUFFER_SIZE);
      HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }


    if (gScheduleValid && gStageNum > 0)
    {
      char msg[64];
      sprintf(msg, "Running Stage %d, delay = %.2f s\r\n",
              gCurrentStageIdx + 1,
              (float)gStageTimes_ms[gCurrentStageIdx]/1000.0f);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

      ApplyStageToLEDs(gStagesPattern[gCurrentStageIdx]);
      HAL_Delay(gStageTimes_ms[gCurrentStageIdx]);

      gCurrentStageIdx++;
      if (gCurrentStageIdx >= gStageNum)
        gCurrentStageIdx = 0;
    }
    else
    {
      HAL_Delay(10);
    }
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (rxIndex < RX_BUFFER_SIZE)
    {
      rxBuffer[rxIndex++] = rxByte;

      // Detect EOF ("EOF")
      if (rxIndex >= 3)
      {
        if (rxBuffer[rxIndex - 3] == 0x45 &&
            rxBuffer[rxIndex - 2] == 0x4F &&
            rxBuffer[rxIndex - 1] == 0x46)
        {
          packetReady = 1;
        }
      }
    }
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
  }
}


uint32_t bytes_to_uint32(uint8_t *data)
{
  uint32_t value = 0;
  value |= ((uint32_t)data[0] << 24);
  value |= ((uint32_t)data[1] << 16);
  value |= ((uint32_t)data[2] << 8);
  value |= ((uint32_t)data[3]);
  return value;
}


static void LED_Pins_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 |
                        GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 |
                           GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                    GPIO_PIN_RESET);
}

/* Apply first 6 bits (bits 11..6) to LEDs on PA0, PA1, PA4, PA5, PA6, PA7 */
static void ApplyStageToLEDs(uint32_t stagePattern)
{
  uint8_t b0 = (stagePattern >> 11) & 0x1;
  uint8_t b1 = (stagePattern >> 10) & 0x1;
  uint8_t b2 = (stagePattern >> 9 ) & 0x1;
  uint8_t b3 = (stagePattern >> 8 ) & 0x1;
  uint8_t b4 = (stagePattern >> 7 ) & 0x1;
  uint8_t b5 = (stagePattern >> 6 ) & 0x1;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, b0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, b1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, b2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, b3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, b4 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, b5 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void ProcessPacket(void)
{
  char msg[256];
  uint16_t idx = 0;


  for (uint16_t i = 0; i + 2 < rxIndex; i++)
  {
    if (rxBuffer[i] == 0x53 && rxBuffer[i + 1] == 0x4F && rxBuffer[i + 2] == 0x46)
    {
      idx = i + 3;
      break;
    }
  }

  if (idx == 0)
  {
    const char *err = "No SOF found\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)err, strlen(err), HAL_MAX_DELAY);
    return;
  }

  uint16_t length = (rxBuffer[idx] << 8) | rxBuffer[idx + 1];
  idx += 2;

  uint8_t StageNum = rxBuffer[idx++];
  uint8_t MaxLight = rxBuffer[idx++];

  uint32_t StageTimes_raw[8] = {0};
  for (int i = 0; i < 8; i++)
  {
    StageTimes_raw[i] = bytes_to_uint32(&rxBuffer[idx]);
    idx += 4;
  }

  uint32_t Stages[8] = {0};
  for (int i = 0; i < 8; i++)
  {
    Stages[i] = bytes_to_uint32(&rxBuffer[idx]);
    idx += 4;
  }

  uint8_t green_Ext = rxBuffer[idx++];
  uint8_t Interrupt = rxBuffer[idx++];


  sprintf(msg, "\r\nNEW PACKET RECEIVED\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  sprintf(msg, "StageNum = %d\r\n", StageNum);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  sprintf(msg, "MaxLight = %d\r\n", MaxLight);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  sprintf(msg, "StageTimes = [");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  for (int i = 0; i < 8; i++)
  {
    float t = (float)StageTimes_raw[i] / 1000.0f;
    sprintf(msg, "%.2f", t);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    sprintf(msg, (i < 7) ? ", " : "]\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  }


  sprintf(msg, "\r\nStages:\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  for (int i = 0; i < 8; i++)
  {
    sprintf(msg, "Stage %d: [", i + 1);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    for (int bit = 11; bit >= 0; bit--)
    {
      uint8_t val = (Stages[i] >> bit) & 0x01;
      sprintf(msg, "%d", val);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
      if (bit != 0) {
        sprintf(msg, ", ");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
      }
    }
    sprintf(msg, "]\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  }

  sprintf(msg, "green_Ext = %d\r\nInterrupt = %d\r\n\r\n", green_Ext, Interrupt);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


  gStageNum = StageNum;
  for (int i = 0; i < 8; i++)
  {
    gStagesPattern[i] = Stages[i];
    uint32_t ms = StageTimes_raw[i];
    if (ms == 0) ms = 1;
    gStageTimes_ms[i] = ms;
  }
  gScheduleValid = 1;
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}



void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
