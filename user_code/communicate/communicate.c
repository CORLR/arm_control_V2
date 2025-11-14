#include "communicate.h"
#include "protocol.h"
#include "main.h"
#include "cmsis_os2.h"
#include <sys/_intsup.h>
#include "usart.h"

// 队列句柄
extern osMessageQueueId_t usart1RxQueueHandle;
extern uint8_t g_usart1_rx_buffer[RX_BUFFER_SIZE];


void communicate()
{
    RxDataChunk_t local_chunk;
    osStatus_t status;
    
    while(1)
    {
        status = osMessageQueueGet(usart1RxQueueHandle, &local_chunk, NULL, osWaitForever);
        if (status == osOK)
        {
            Protocol_Parse_Chunk(local_chunk.buffer, local_chunk.len);
        }
        osDelay(1);
    }
    
}



