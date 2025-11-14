#include "protocol.h"
#include "stdbool.h" 
#include "stdio.h"   
#include "stdint.h"
#include "arm_control.h"

#define CONTROL_MODE_ABSOLUTE 0// 绝对控制模式
#define CONTROL_MODE_RELATIVE 1// 相对控制模式

#define CURRENT_CONTROL_MODE CONTROL_MODE_RELATIVE // 当前控制模式

// 帧头
#define FRAME_HEADER 0x55 

// 功能码
#define FUNCTION_CODE 0x02

// 数据长度 (7 个编码器 * 2 字节/编码器)
#define FRAME_DATA_LENGTH 14 


typedef enum {
    STATE_WAIT_HEADER,      // 状态1: 等待 0x55
    STATE_READ_FUNCTION,    // 状态2: 等待 0x01
    STATE_READ_DATA         // 状态3: 接收 14 字节数据
} ParseState_t;


// 当前状态
static ParseState_t g_parse_state = STATE_WAIT_HEADER;

// 临时缓冲区，用于拼装 14 字节的数据
static uint8_t g_frame_buffer[FRAME_DATA_LENGTH]; 

// 当前已接收的数据字节计数器
static uint8_t g_byte_counter = 0;

// 记录初始角度，用于相对控制模式
static float g_initial_host_angle[8] = {0.0f};

// 记录相对控制模式初始化标志((用于将第一帧数据作为零点))
static bool g_relative_mode_initialized = false;


/**
 * @brief 协议解析，逐字节处理数据
 * 
 * @param byte 
 */
static void Protocol_Parse_Byte(uint8_t byte)
{
    switch (g_parse_state)
    {
        case STATE_WAIT_HEADER:
            if (byte == FRAME_HEADER)
            {
                g_parse_state = STATE_READ_FUNCTION;
            }
            break;

        case STATE_READ_FUNCTION:
            if (byte == FUNCTION_CODE)
            {
                g_byte_counter = 0; // 清零数据计数器
                g_parse_state = STATE_READ_DATA;
            }
            else
            {
                if (byte == FRAME_HEADER)
                {
                }
                else
                {
                    g_parse_state = STATE_WAIT_HEADER;
                }
            }
            break;

        case STATE_READ_DATA:
            g_frame_buffer[g_byte_counter] = byte;
            g_byte_counter++;

            if (g_byte_counter >= FRAME_DATA_LENGTH)
            {
                process_encoder_data(g_frame_buffer, FRAME_DATA_LENGTH);
                g_parse_state = STATE_WAIT_HEADER;
            }
            break;
            
        default:
            g_parse_state = STATE_WAIT_HEADER;
            break;
    }
}


/**
 * @brief 协议解析入口函数
 * 
 * @param chunk 
 * @param len 
 */
void Protocol_Parse_Chunk(uint8_t* chunk, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        Protocol_Parse_Byte(chunk[i]);
    }
}


/**
 * @brief 处理编码器数据
 * 
 * @param pData 
 * @param len 
 */
void process_encoder_data(uint8_t* pData, uint32_t len)
{

    if (len != FRAME_DATA_LENGTH)
    {
        // 理论上不会发生
        return; 
    }

    #if (CURRENT_CONTROL_MODE == CONTROL_MODE_ABSOLUTE)

    /************************************************/
    /* 模式 0: 绝对位置控制                            */
    /************************************************/

    if(osMutexAcquire(setJointAngleMutexHandle, 0) == osOK)
    {
        set_joint_angle[1] = -((float)((pData[0] << 8 | pData[1]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_1);
        set_joint_angle[2] = (float)((pData[2] << 8 | pData[3]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_2;
        set_joint_angle[3] = (float)((pData[4] << 8 | pData[5]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_3;
        set_joint_angle[4] = -((float)((pData[6] << 8 | pData[7]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_4);
        set_joint_angle[5] = (float)((pData[8] << 8 | pData[9]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_5;
        set_joint_angle[6] = (float)((pData[10] << 8 | pData[11]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_6;
        set_joint_angle[7] = (float)((pData[12] << 8 | pData[13]) - 8192) / 16384 * 2 * PI - ANGLE_ERROR_7;
        
        for(int i = 1; i <= 7; i++)
        {
            if(set_joint_angle[i] > PI) set_joint_angle[i] -= 2 * PI;
            if(set_joint_angle[i] < -PI) set_joint_angle[i] += 2 * PI;
        }
        osMutexRelease(setJointAngleMutexHandle);
    }

    #elif (CURRENT_CONTROL_MODE == CONTROL_MODE_RELATIVE)

    /************************************************/
    /* 模式 1: 相对位置控制                            */
    /************************************************/
    
    float current_host_angle[8];

    current_host_angle[1] = -((float)((pData[0] << 8 | pData[1]) - 8192) / 16384 * 2 * PI) ;
    current_host_angle[2] = (float)((pData[2] << 8 | pData[3]) - 8192) / 16384 * 2 * PI;
    current_host_angle[3] = (float)((pData[4] << 8 | pData[5]) - 8192) / 16384 * 2 * PI;
    current_host_angle[4] = -((float)((pData[6] << 8 | pData[7]) - 8192) / 16384 * 2 * PI);
    current_host_angle[5] = (float)((pData[8] << 8 | pData[9]) - 8192) / 16384 * 2 * PI;
    current_host_angle[6] = (float)((pData[10] << 8 | pData[11]) - 8192) / 16384 * 2 * PI;
    current_host_angle[7] = (float)((pData[12] << 8 | pData[13]) - 8192) / 16384 * 2 * PI;

    
    for(int i = 1; i <= 7; i++)
    {
        if(current_host_angle[i] > PI) current_host_angle[i] -= 2 * PI;
        if(current_host_angle[i] < -PI) current_host_angle[i] += 2 * PI;
    }

    if(osMutexAcquire(setJointAngleMutexHandle, 0) == osOK)
    {
        if (!g_relative_mode_initialized)
        {
            // 首次接收数据，记录初始角度作为零点
            for(int i = 1; i <= 7; i++)
            {
                g_initial_host_angle[i] = current_host_angle[i];
            }
            g_relative_mode_initialized = true;
        }
        else
        {
            // 已经初始化，计算 "当前角度" 与 "初始零点" 的差值
            for(int i = 1; i <= 7; i++)
            {
                float relative_angle = current_host_angle[i] - g_initial_host_angle[i];

                if (relative_angle > PI) 
                {
                    relative_angle -= 2 * PI;
                } 
                else if (relative_angle < -PI) 
                {
                    relative_angle += 2 * PI;
                }
                
                set_joint_angle[i] = relative_angle;
            }
        }
        
        osMutexRelease(setJointAngleMutexHandle);
    }

    #else
        #error "未定义有效的 CURRENT_CONTROL_MODE !!"
    #endif
    
}