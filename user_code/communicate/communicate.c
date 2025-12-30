#include "communicate.h"
#include "protocol.h"
#include "main.h"
#include "cmsis_os2.h"
#include <sys/_intsup.h>
#include "usart.h"

// 队列句柄
extern osMessageQueueId_t usart1RxQueueHandle;

// 双缓冲区
extern uint8_t g_usart1_rx_buffer_1[RX_BUFFER_SIZE];
extern uint8_t g_usart1_rx_buffer_2[RX_BUFFER_SIZE];

void communicate()
{
    RxDataChunk_t local_chunk;
    osStatus_t status;
    uint8_t *pBuffer;

    while (1)
    {
        status = osMessageQueueGet(usart1RxQueueHandle, &local_chunk, NULL, osWaitForever);
        if (status == osOK)
        {
            // 根据缓冲区索引选择对应的缓冲区
            if (local_chunk.buffer_index == 0)
            {
                pBuffer = g_usart1_rx_buffer_1;
            }
            else
            {
                pBuffer = g_usart1_rx_buffer_2;
            }

            // 从指定缓冲区解析数据
            Protocol_Parse_Chunk(pBuffer, local_chunk.len);
        }
        osDelay(1);
    }
}
