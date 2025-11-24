#include "protocol.h"
#include "stdbool.h" 
#include "stdio.h"   
#include "stdint.h"
#include "string.h" 
#include "arm_control.h" // 包含 set_joint_angle, setJointAngleMutexHandle 等定义

// ==============================================================================
// 配置宏定义
// ==============================================================================

// 控制模式定义
#define CONTROL_MODE_ABSOLUTE 0 // 绝对控制模式 (用于位置闭环)
#define CONTROL_MODE_RELATIVE 1 // 相对控制模式 (用于增量控制)

// **当前控制模式选择**
#define CURRENT_CONTROL_MODE CONTROL_MODE_RELATIVE 

// --- 滤波配置 ---
#define ENABLE_MEDIAN_FILTER  1   // 1: 开启中值滤波, 0: 关闭
#define FILTER_WINDOW_SIZE    5   // 窗口大小 (建议 3, 5, 7)

// --- 协议常量 ---
#define FRAME_HEADER      0x55 
#define FUNCTION_CODE     0x02
#define FRAME_DATA_LENGTH 14      // 7个关节 * 2字节
#define PI                3.1415926f

// ==============================================================================
// 数据结构与静态变量
// ==============================================================================

typedef enum {
    STATE_WAIT_HEADER,      // 状态1: 等待 0x55
    STATE_READ_FUNCTION,    // 状态2: 等待 功能码
    STATE_READ_DATA         // 状态3: 接收 数据
} ParseState_t;

// 协议解析状态
static ParseState_t g_parse_state = STATE_WAIT_HEADER;
static uint8_t g_frame_buffer[FRAME_DATA_LENGTH]; 
static uint8_t g_byte_counter = 0;

// 相对模式专用：记录上电时的初始原始角度
static float g_initial_raw_angle[8] = {0.0f};
static bool g_relative_mode_initialized = false;

// 滤波专用变量
#if ENABLE_MEDIAN_FILTER
// 滤波窗口缓冲区 [关节索引 1-7][窗口数据]
static float g_filter_buffer[8][FILTER_WINDOW_SIZE]; 
static uint8_t g_filter_idx = 0; 
static bool g_filter_initialized = false;
#endif

// ==============================================================================
// 内部辅助函数声明
// ==============================================================================
static void apply_median_filter(float *raw_angles);
static float normalize_angle(float angle);
static void bubble_sort(float *arr, int n);

// ==============================================================================
// 核心逻辑实现
// ==============================================================================

/**
 * @brief 简单的冒泡排序，用于求中值
 */
static void bubble_sort(float *arr, int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                float temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

/**
 * @brief 对原始上位机数据进行中值滤波
 * 输入/输出都是 host_raw_angle 数组
 */
static void apply_median_filter(float *raw_angles)
{
#if ENABLE_MEDIAN_FILTER
    // 1. 初始化：如果是第一次运行，将窗口填满当前值，防止从0突变
    if (!g_filter_initialized) {
        for (int j = 1; j <= 7; j++) {
            for (int k = 0; k < FILTER_WINDOW_SIZE; k++) {
                g_filter_buffer[j][k] = raw_angles[j];
            }
        }
        g_filter_initialized = true;
    } else {
        // 2. 正常运行：写入新数据到环形缓冲区
        for (int j = 1; j <= 7; j++) {
            g_filter_buffer[j][g_filter_idx] = raw_angles[j];
        }
        
        // 更新索引
        g_filter_idx++;
        if (g_filter_idx >= FILTER_WINDOW_SIZE) {
            g_filter_idx = 0;
        }
    }

    // 3. 计算中值
    float temp_window[FILTER_WINDOW_SIZE];
    for (int j = 1; j <= 7; j++) {
        // 复制数据到临时数组进行排序，不破坏历史记录的顺序
        memcpy(temp_window, g_filter_buffer[j], sizeof(temp_window));
        bubble_sort(temp_window, FILTER_WINDOW_SIZE);
        
        // 取中间值更新当前角度
        raw_angles[j] = temp_window[FILTER_WINDOW_SIZE / 2];
    }
#endif
}

/**
 * @brief 角度归一化到 -PI ~ PI
 */
static float normalize_angle(float angle)
{
    while (angle > PI)  angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

/**
 * @brief 处理编码器数据 (核心业务逻辑)
 */
void process_encoder_data(uint8_t* pData, uint32_t len)
{
    if (len != FRAME_DATA_LENGTH) return;

    // ---------------------------------------------------------
    // 1. 数据解析：获取“上位机原始角度” (Host Raw Angle)
    //    这里只做 数值->弧度 的转换，不处理方向，也不减误差。
    //    范围: -PI ~ PI
    // ---------------------------------------------------------
    float host_raw_angle[8];
    
    // 协议公式: (val - 8192) / 16384 * 2PI
    host_raw_angle[1] = (float)((pData[0] << 8 | pData[1]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[2] = (float)((pData[2] << 8 | pData[3]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[3] = (float)((pData[4] << 8 | pData[5]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[4] = (float)((pData[6] << 8 | pData[7]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[5] = (float)((pData[8] << 8 | pData[9]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[6] = (float)((pData[10] << 8 | pData[11]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[7] = (float)((pData[12] << 8 | pData[13]) - 8192) / 16384.0f * 2.0f * PI;

    // ---------------------------------------------------------
    // 2. 滤波处理
    //    对原始数据进行滤波，保证后续计算平滑
    // ---------------------------------------------------------
    apply_median_filter(host_raw_angle);

    // ---------------------------------------------------------
    // 3. 业务逻辑与坐标系转换
    //    公式: Arm_Angle = Direction * (Host_Raw - Zero_Offset)
    // ---------------------------------------------------------
    if(osMutexAcquire(setJointAngleMutexHandle, 0) == osOK)
    {
        #if (CURRENT_CONTROL_MODE == CONTROL_MODE_ABSOLUTE)
            // 计算偏差值 (Host - Error)
            float diff[8];
            diff[1] = host_raw_angle[1] - ANGLE_ERROR_1;
            diff[2] = host_raw_angle[2] - ANGLE_ERROR_2;
            diff[3] = host_raw_angle[3] - ANGLE_ERROR_3;
            diff[4] = host_raw_angle[4] - ANGLE_ERROR_4;
            diff[5] = host_raw_angle[5] - ANGLE_ERROR_5;
            diff[6] = host_raw_angle[6] - ANGLE_ERROR_6;
            diff[7] = host_raw_angle[7] - ANGLE_ERROR_7;

            set_joint_angle[1] = -diff[1];
            set_joint_angle[2] =  diff[2];
            set_joint_angle[3] =  diff[3];
            set_joint_angle[4] = -diff[4];
            set_joint_angle[5] =  diff[5];
            set_joint_angle[6] =  diff[6];
            set_joint_angle[7] = -diff[7];

        #elif (CURRENT_CONTROL_MODE == CONTROL_MODE_RELATIVE)
            if (!g_relative_mode_initialized)
            {
                // 首次运行，记录当前原始角度作为“零点”
                for(int i = 1; i <= 7; i++) {
                    g_initial_raw_angle[i] = host_raw_angle[i];
                }
                g_relative_mode_initialized = true;
                
                // 初始时刻角度设为0
                for(int i = 1; i <= 7; i++) set_joint_angle[i] = 0.0f;
            }
            else
            {
                // 计算与初始位置的差值 (Delta)
                float delta[8];
                for(int i = 1; i <= 7; i++) {
                    delta[i] = host_raw_angle[i] - g_initial_raw_angle[i];
                    // 处理跨越边界的情况 (例如从 PI 变到 -PI，实际只变了一点点)
                    delta[i] = normalize_angle(delta[i]);
                }

                set_joint_angle[1] = -delta[1];
                set_joint_angle[2] =  delta[2];
                set_joint_angle[3] =  delta[3];
                set_joint_angle[4] = -delta[4];
                set_joint_angle[5] =  delta[5];
                set_joint_angle[6] =  delta[6];
                set_joint_angle[7] = -delta[7];
            }

        #else
            #error "未定义有效的 CURRENT_CONTROL_MODE"
        #endif

        // 最终安全归一化，确保输出在 -PI ~ PI 范围内
        for(int i = 1; i <= 7; i++) {
            set_joint_angle[i] = normalize_angle(set_joint_angle[i]);
        }

        osMutexRelease(setJointAngleMutexHandle);
    }
}

/**
 * @brief 协议解析，逐字节处理
 */
static void Protocol_Parse_Byte(uint8_t byte)
{
    switch (g_parse_state)
    {
        case STATE_WAIT_HEADER:
            if (byte == FRAME_HEADER) {
                g_parse_state = STATE_READ_FUNCTION;
            }
            break;

        case STATE_READ_FUNCTION:
            if (byte == FUNCTION_CODE) {
                g_byte_counter = 0; 
                g_parse_state = STATE_READ_DATA;
            }
            else {
                // 容错：如果字节是 Header，重新进入 Header 状态
                if (byte == FRAME_HEADER) g_parse_state = STATE_READ_FUNCTION;
                else g_parse_state = STATE_WAIT_HEADER;
            }
            break;

        case STATE_READ_DATA:
            g_frame_buffer[g_byte_counter] = byte;
            g_byte_counter++;

            if (g_byte_counter >= FRAME_DATA_LENGTH) {
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
 * @brief 协议解析入口 (外部调用)
 */
void Protocol_Parse_Chunk(uint8_t* chunk, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        Protocol_Parse_Byte(chunk[i]);
    }
}