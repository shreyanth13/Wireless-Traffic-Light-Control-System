#include "cmsis_os.h"
#include "main.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#define RX_BUFFER_SIZE 256

extern uint8_t rxBuffer[];
extern uint16_t rxIndex;
extern volatile uint8_t packetReady;

extern SemaphoreHandle_t xUART_Mutex;
extern SemaphoreHandle_t xData_Mutex;

extern UART_HandleTypeDef huart6;

extern uint8_t gStageNum;
extern uint32_t gStageTimes_ms[8];
extern uint32_t gStagesPattern[8];
extern volatile uint8_t gCurrentStageIdx;
extern volatile uint8_t gScheduleValid;

extern osThreadId_t ledTaskHandle;

extern void ApplyStageToLEDs(uint32_t pattern);

static uint8_t lastPacketBuf[RX_BUFFER_SIZE];
static uint16_t lastPacketLen = 0;
static volatile uint8_t lastPacketAvailable = 0;

static TimerHandle_t xStageTimer = NULL;

static void PrintUART_Local(const char *buf);
static void PrintStoredPacketOnce(void);
void StartPacketProcessor(void *argument);
void StartLEDController(void *argument);
void InitStageTimer(void);
static void vStageTimerCallback(TimerHandle_t xTimer);

static void PrintUART_Local(const char *buf)
{
    if (xUART_Mutex != NULL)
    {
        if (xSemaphoreTake(xUART_Mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            HAL_UART_Transmit(&huart6, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
            xSemaphoreGive(xUART_Mutex);
            return;
        }
    }
    HAL_UART_Transmit(&huart6, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

static void vStageTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (ledTaskHandle != NULL)
    {
        xTaskNotifyGive((TaskHandle_t)ledTaskHandle);
    }
}

void InitStageTimer(void)
{
    if (xStageTimer == NULL)
    {
        xStageTimer = xTimerCreate("StageTimer",pdMS_TO_TICKS(1),pdFALSE,NULL,vStageTimerCallback);
    }
}

static void PrintStoredPacketOnce(void)
{
    if (!lastPacketAvailable || lastPacketLen == 0) return;

    uint8_t localBuf[RX_BUFFER_SIZE];
    uint16_t len = 0;

    taskENTER_CRITICAL();
    len = lastPacketLen;
    if (len > RX_BUFFER_SIZE) len = RX_BUFFER_SIZE;
    memcpy(localBuf, lastPacketBuf, len);
    lastPacketAvailable = 0;
    taskEXIT_CRITICAL();

    uint16_t idx = 0;
    for (uint16_t i = 0; i + 2 < len; i++)
    {
        if (localBuf[i] == 0x53 && localBuf[i+1] == 0x4F && localBuf[i+2] == 0x46)
        {
            idx = i + 3;
            break;
        }
    }

    if (idx == 0)
    {
        PrintUART_Local("\r\nNEW PACKET RECEIVED\r\n");
        PrintUART_Local("No SOF found\r\n");
        return;
    }

    char msg[256];

    if (idx + 2 <= len) idx += 2;

    if (idx >= len)
    {
        PrintUART_Local("\r\nNEW PACKET RECEIVED\r\n");
        PrintUART_Local("Truncated packet\r\n");
        return;
    }

    uint8_t StageNum = (idx < len) ? localBuf[idx++] : 0;
    uint8_t MaxLight = (idx < len) ? localBuf[idx++] : 0;

    uint32_t StageTimes_raw[8] = {0};
    for (int i = 0; i < 8; i++)
    {
        if (idx + 3 < len)
        {
            StageTimes_raw[i] = ((uint32_t)localBuf[idx]   << 24) |((uint32_t)localBuf[idx+1] << 16) |((uint32_t)localBuf[idx+2] << 8)  |(uint32_t)localBuf[idx+3];
        }
        else
        {
            StageTimes_raw[i] = 0;
        }
        idx += 4;
    }

    uint32_t Stages[8] = {0};
    for (int i = 0; i < 8; i++)
    {
        if (idx + 3 < len)
        {
            Stages[i] = ((uint32_t)localBuf[idx]   << 24) |((uint32_t)localBuf[idx+1] << 16) |((uint32_t)localBuf[idx+2] << 8)  |(uint32_t)localBuf[idx+3];
        }
        else
        {
            Stages[i] = 0;
        }
        idx += 4;
    }

    uint8_t green_Ext = 0;
    uint8_t Interrupt = 0;
    if (idx < len) green_Ext = localBuf[idx++];
    if (idx < len) Interrupt = localBuf[idx++];

    PrintUART_Local("\r\nNEW PACKET RECEIVED\r\n");
    snprintf(msg, sizeof(msg), "StageNum = %d\r\n", StageNum); PrintUART_Local(msg);
    snprintf(msg, sizeof(msg), "MaxLight = %d\r\n", MaxLight); PrintUART_Local(msg);

    PrintUART_Local("StageTimes = [");
    for (int i = 0; i < 8; i++)
    {
        snprintf(msg, sizeof(msg), "%.2f", (float)StageTimes_raw[i] / 1000.0f);
        PrintUART_Local(msg);
        if (i < 7) PrintUART_Local(", ");
    }
    PrintUART_Local("]\r\n");

    PrintUART_Local("\r\nStages:\r\n");
    for (int i = 0; i < 8; i++)
    {
        snprintf(msg, sizeof(msg), "Stage %d: [", i + 1); PrintUART_Local(msg);
        for (int bit = 11; bit >= 0; bit--)
        {
            int val = (int)((Stages[i] >> bit) & 0x01);
            snprintf(msg, sizeof(msg), "%d", val);
            PrintUART_Local(msg);
            if (bit != 0) PrintUART_Local(", ");
        }
        PrintUART_Local("]\r\n");
    }

    snprintf(msg, sizeof(msg),"green_Ext = %d\r\nInterrupt = %d\r\n\r\n",green_Ext, Interrupt);
    PrintUART_Local(msg);
}

void StartPacketProcessor(void *argument)
{
    (void) argument;
    uint8_t localBuf[RX_BUFFER_SIZE];

    for (;;)
    {
        if (packetReady)
        {
            taskENTER_CRITICAL();
            uint16_t len = rxIndex;
            if (len > RX_BUFFER_SIZE) len = RX_BUFFER_SIZE;
            memcpy(localBuf, rxBuffer, len);
            rxIndex = 0;
            packetReady = 0;
            taskEXIT_CRITICAL();

            uint16_t idx = 0;
            for (uint16_t i = 0; i + 2 < len; i++)
            {
                if (localBuf[i] == 0x53 && localBuf[i+1] == 0x4F && localBuf[i+2] == 0x46)
                {
                    idx = i + 3;
                    break;
                }
            }

            taskENTER_CRITICAL();
            lastPacketLen = len;
            if (lastPacketLen > RX_BUFFER_SIZE) lastPacketLen = RX_BUFFER_SIZE;
            memcpy(lastPacketBuf, localBuf, lastPacketLen);
            lastPacketAvailable = 1;
            taskEXIT_CRITICAL();

            if (idx != 0)
            {
                idx += 2;

                if (idx + 1 < len)
                {
                    uint8_t StageNum = localBuf[idx++];
                    uint8_t MaxLight = localBuf[idx++];

                    uint32_t StageTimes_raw[8] = {0};
                    for (int i = 0; i < 8 && idx + 3 < len; i++)
                    {
                        StageTimes_raw[i] = ((uint32_t)localBuf[idx] << 24) |((uint32_t)localBuf[idx+1] << 16) |((uint32_t)localBuf[idx+2] << 8)|(uint32_t)localBuf[idx+3];
                        idx += 4;
                    }

                    uint32_t Stages[8] = {0};
                    for (int i = 0; i < 8 && idx + 3 < len; i++)
                    {
                        Stages[i] = ((uint32_t)localBuf[idx] << 24) | ((uint32_t)localBuf[idx+1] << 16) | ((uint32_t)localBuf[idx+2] << 8)| (uint32_t)localBuf[idx+3];
                        idx += 4;
                    }

                    if (xData_Mutex != NULL)
                    {
                        if (xSemaphoreTake(xData_Mutex, portMAX_DELAY) == pdTRUE)
                        {
                            gStageNum = StageNum;
                            for (int i = 0; i < 8; i++)
                            {
                                gStagesPattern[i] = Stages[i];
                                uint32_t ms = StageTimes_raw[i];
                                if (ms == 0) ms = 1;
                                gStageTimes_ms[i] = ms;
                            }
                            gScheduleValid = 1;
                            xSemaphoreGive(xData_Mutex);
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void StartLEDController(void *argument)
{
    (void) argument;
    for (;;)
    {
        if (gScheduleValid && gStageNum > 0)
        {
            uint8_t  localIdx      = 0;
            uint32_t localDelayMs  = 1;
            uint32_t localPattern  = 0;

            if (xData_Mutex != NULL)
            {
                if (xSemaphoreTake(xData_Mutex, portMAX_DELAY) == pdTRUE)
                {
                    localIdx     = gCurrentStageIdx;
                    localDelayMs = gStageTimes_ms[localIdx];
                    localPattern = gStagesPattern[localIdx];
                    xSemaphoreGive(xData_Mutex);
                }
            }

            if (lastPacketAvailable)
            {
                PrintStoredPacketOnce();
            }

            char msg[64];
            snprintf(msg, sizeof(msg),"Running Stage %d, delay = %.2f s\r\n",localIdx + 1, (float)localDelayMs / 1000.0f);
            PrintUART_Local(msg);

            ApplyStageToLEDs(localPattern);

            if (localDelayMs == 0) localDelayMs = 1;

            if (xStageTimer != NULL)
            {
                xTimerChangePeriod(xStageTimer,pdMS_TO_TICKS(localDelayMs),portMAX_DELAY);
                xTimerStart(xStageTimer, portMAX_DELAY);
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(localDelayMs));
            }

            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            if (xData_Mutex != NULL)
            {
                if (xSemaphoreTake(xData_Mutex, portMAX_DELAY) == pdTRUE)
                {
                    gCurrentStageIdx++;
                    if (gCurrentStageIdx >= gStageNum)
                        gCurrentStageIdx = 0;
                    xSemaphoreGive(xData_Mutex);
                }
            }
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
