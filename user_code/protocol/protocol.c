#include "protocol.h"
#include "stdbool.h" 
#include "stdio.h"   
#include "stdint.h"
#include "string.h" 
#include "math.h"
#include "arm_control.h" 
#include "kinematics.h"

// ==============================================================================
//  功能开关配置 (修改此处来切换模式)
// ==============================================================================

/**
 * @brief GMR 重定向算法开关
 * 1: 启用 GMR (骨骼比例缩放 + IK逆解) -> 输出 g_TargetPose 给 arm_control
 * 0: 启用 原有逻辑 (直接关节映射 + 相对/绝对模式) -> 直接修改 set_joint_angle
 */
#define ENABLE_GMR_MAPPING  1

// ==============================================================================
//  通用配置 (两模式共用)
// ==============================================================================

#define ENABLE_MEDIAN_FILTER  1   // 1: 开启中值滤波
#define FILTER_WINDOW_SIZE    5   // 滤波窗口
#define FRAME_HEADER      0x55 
#define FUNCTION_CODE     0x02
#define FRAME_DATA_LENGTH 14      
#define PI                3.1415926f

// 接口变量声明 (arm_control.c 定义)
extern float master_joint_angle[8]; 
extern Matrix4x4 g_TargetPose;      
extern uint8_t g_ik_enable;         

// ==============================================================================
//  模式专用配置
// ==============================================================================

#if ENABLE_GMR_MAPPING
    // --- GMR 模式参数 ---
    // 机器人骨骼 (参考 URDF)
    #define ROBOT_UPPER_ARM_LEN  0.2000f 
    #define ROBOT_FORE_ARM_LEN   0.3575f 
    // 人体骨骼 (参考实际测量)
    #define HUMAN_UPPER_ARM_LEN  0.22154f  
    #define HUMAN_FORE_ARM_LEN   0.22850f

#else
    // --- 原有逻辑参数 ---
    #define CONTROL_MODE_ABSOLUTE 0 
    #define CONTROL_MODE_RELATIVE 1 
    #define CURRENT_CONTROL_MODE  CONTROL_MODE_RELATIVE // 当前使用的控制模式

    // 假设这些宏在 arm_control.h 中定义，若无则需在此补全
    #ifndef ANGLE_ERROR_1
        #define ANGLE_ERROR_1 0.0f
        #define ANGLE_ERROR_2 0.0f
        #define ANGLE_ERROR_3 0.0f
        #define ANGLE_ERROR_4 0.0f
        #define ANGLE_ERROR_5 0.0f
        #define ANGLE_ERROR_6 0.0f
        #define ANGLE_ERROR_7 0.0f
    #endif
#endif

// ==============================================================================
//  静态变量与状态
// ==============================================================================

// 协议解析状态
typedef enum { STATE_WAIT_HEADER, STATE_READ_FUNCTION, STATE_READ_DATA } ParseState_t;
static ParseState_t g_parse_state = STATE_WAIT_HEADER;
static uint8_t g_frame_buffer[FRAME_DATA_LENGTH]; 
static uint8_t g_byte_counter = 0;

// 滤波缓冲
#if ENABLE_MEDIAN_FILTER
static float g_filter_buffer[8][FILTER_WINDOW_SIZE]; 
static uint8_t g_filter_idx = 0; 
static bool g_filter_initialized = false;
#endif

#if ENABLE_GMR_MAPPING
    // GMR 专用状态
    static float DH_HUMAN[JOINT_COUNT][4]; 
    static float g_pos_m0[3], g_pos_s0[3]; 
    static bool g_mapping_init = false;
#else
    // 原有逻辑专用状态
    static float g_initial_raw_angle[8] = {0.0f};
    static bool g_relative_mode_initialized = false;
    // 用于 FK 监控
    static float current_joints[7]; 
    static Matrix4x4 current_pose;
#endif

// ==============================================================================
//  内部辅助函数
// ==============================================================================

// 冒泡排序
static void bubble_sort(float *arr, int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                float temp = arr[j]; arr[j] = arr[j + 1]; arr[j + 1] = temp;
            }
        }
    }
}

// 中值滤波
static void apply_median_filter(float *raw_angles) {
#if ENABLE_MEDIAN_FILTER
    if (!g_filter_initialized) {
        for (int j = 1; j <= 7; j++) {
            for (int k = 0; k < FILTER_WINDOW_SIZE; k++) g_filter_buffer[j][k] = raw_angles[j];
        }
        g_filter_initialized = true;
    } else {
        for (int j = 1; j <= 7; j++) g_filter_buffer[j][g_filter_idx] = raw_angles[j];
        g_filter_idx = (g_filter_idx + 1) % FILTER_WINDOW_SIZE;
    }
    float temp_window[FILTER_WINDOW_SIZE];
    for (int j = 1; j <= 7; j++) {
        memcpy(temp_window, g_filter_buffer[j], sizeof(temp_window));
        bubble_sort(temp_window, FILTER_WINDOW_SIZE);
        raw_angles[j] = temp_window[FILTER_WINDOW_SIZE / 2];
    }
#endif
}

// 角度归一化 (-PI ~ PI)
static float normalize_angle(float angle) {
    while (angle > PI)  angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

#if ENABLE_GMR_MAPPING
// 初始化人体 DH 参数
static void Init_Human_Model(void) {
    float base_off = 0.085f;
    DH_HUMAN[0][0]=0.0f;    DH_HUMAN[0][1]=base_off;            DH_HUMAN[0][2]=0.0f; DH_HUMAN[0][3]=1.5708f;
    // J2 (Shoulder Pitch)
    DH_HUMAN[1][0]=-1.57f;  DH_HUMAN[1][1]=0.0f;                DH_HUMAN[1][2]=0.0f; DH_HUMAN[1][3]=1.5708f;
    // J3 (Shoulder Roll -> Elbow) - 上臂
    DH_HUMAN[2][0]=0.0f;    DH_HUMAN[2][1]=0.0f;                DH_HUMAN[2][2]=HUMAN_UPPER_ARM_LEN; DH_HUMAN[2][3]=-1.5708f;
    // J4 (Elbow Pitch)
    DH_HUMAN[3][0]=0.0f;    DH_HUMAN[3][1]=0.0f;                DH_HUMAN[3][2]=0.0f; DH_HUMAN[3][3]=1.5708f;
    // J5 (Forearm Roll -> Wrist) - 前臂
    DH_HUMAN[4][0]=0.0f;    DH_HUMAN[4][1]=0.0f;                DH_HUMAN[4][2]=HUMAN_FORE_ARM_LEN;  DH_HUMAN[4][3]=1.5708f;
    // J6 (Wrist Pitch)
    DH_HUMAN[5][0]=-1.57f;  DH_HUMAN[5][1]=0.0f;                DH_HUMAN[5][2]=0.0f; DH_HUMAN[5][3]=-1.5708f;
    // J7 (Wrist Roll)
    DH_HUMAN[6][0]=0.0f;    DH_HUMAN[6][1]=0.05f;               DH_HUMAN[6][2]=0.0f; DH_HUMAN[6][3]=0.0f;
}
#endif

// ==============================================================================
//  核心处理函数
// ==============================================================================

void process_encoder_data(uint8_t* pData, uint32_t len)
{
    if (len != FRAME_DATA_LENGTH) return;

    // 1. 数据解析 (Common)
    float host_raw_angle[8];
    host_raw_angle[1] = (float)((pData[0] << 8 | pData[1]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[2] = (float)((pData[2] << 8 | pData[3]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[3] = (float)((pData[4] << 8 | pData[5]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[4] = (float)((pData[6] << 8 | pData[7]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[5] = (float)((pData[8] << 8 | pData[9]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[6] = (float)((pData[10] << 8 | pData[11]) - 8192) / 16384.0f * 2.0f * PI;
    host_raw_angle[7] = (float)((pData[12] << 8 | pData[13]) - 8192) / 16384.0f * 2.0f * PI;

    // 2. 滤波 (Common)
    apply_median_filter(host_raw_angle);

    // 3. 业务逻辑分支
    if(osMutexAcquire(setJointAngleMutexHandle, 0) == osOK)
    {
#if ENABLE_GMR_MAPPING
        // ============================================================
        // 方案 A: GMR 重定向 + IK (计算目标位姿)
        // ============================================================
        
        // 准备数据: 将 1-7 (协议索引) 转为 0-6 (算法索引)
        float human_joints[JOINT_COUNT];
        for(int i=0; i<JOINT_COUNT; i++) human_joints[i] = host_raw_angle[i+1];

        // A. 初始化 (仅一次)
        if (!g_mapping_init) {
            Init_Human_Model();
            
            // Calc Master Init Pos
            Matrix4x4 T_m;
            Calculate_FK_Custom(human_joints, DH_HUMAN, JOINT_COUNT, &T_m);
            g_pos_m0[0] = T_m.data[3]; g_pos_m0[1] = T_m.data[7]; g_pos_m0[2] = T_m.data[11];

            // Calc Slave Init Pos (假设零位启动)
            float zero_joints[JOINT_COUNT] = {0};
            Matrix4x4 T_s;
            Calculate_FK(zero_joints, &T_s); 
            g_pos_s0[0] = T_s.data[3]; g_pos_s0[1] = T_s.data[7]; g_pos_s0[2] = T_s.data[11];

            g_mapping_init = true;
        }

        // B. Master FK
        Matrix4x4 T_m_curr;
        Calculate_FK_Custom(human_joints, DH_HUMAN, JOINT_COUNT, &T_m_curr);

        // C. Scaling (基于骨骼长度比)
        float scale_ratio = (ROBOT_UPPER_ARM_LEN + ROBOT_FORE_ARM_LEN) / 
                            (HUMAN_UPPER_ARM_LEN + HUMAN_FORE_ARM_LEN);
        
        float dx = (T_m_curr.data[3] - g_pos_m0[0]) * scale_ratio;
        float dy = (T_m_curr.data[7] - g_pos_m0[1]) * scale_ratio;
        float dz = (T_m_curr.data[11] - g_pos_m0[2]) * scale_ratio;

        // D. 更新 TargetPose
        Mat4_SetIdentity(&g_TargetPose); 
        
        // 复制旋转 (假设基座对齐)
        g_TargetPose.data[0] = T_m_curr.data[0]; g_TargetPose.data[1] = T_m_curr.data[1]; g_TargetPose.data[2] = T_m_curr.data[2];
        g_TargetPose.data[4] = T_m_curr.data[4]; g_TargetPose.data[5] = T_m_curr.data[5]; g_TargetPose.data[6] = T_m_curr.data[6];
        g_TargetPose.data[8] = T_m_curr.data[8]; g_TargetPose.data[9] = T_m_curr.data[9]; g_TargetPose.data[10] = T_m_curr.data[10];
        
        // 应用位置
        g_TargetPose.data[3]  = g_pos_s0[0] + dx;
        g_TargetPose.data[7]  = g_pos_s0[1] + dy;
        g_TargetPose.data[11] = g_pos_s0[2] + dz;

        // E. 更新 Master 角度 (用于 IK 姿态引导)
        for(int i=1; i<=7; i++) master_joint_angle[i] = host_raw_angle[i];

        // 告知 arm_control 开启 IK
        g_ik_enable = 1;

#else
        // ============================================================
        // 方案 B: 原有逻辑 (直接关节映射)
        // ============================================================
        
        // 告知 arm_control 关闭 IK (直接使用 set_joint_angle)
        g_ik_enable = 0;

    #if (CURRENT_CONTROL_MODE == CONTROL_MODE_ABSOLUTE)
        // 绝对模式: 减去零偏
        float diff[8];
        diff[1] = host_raw_angle[1] - ANGLE_ERROR_1;
        diff[2] = host_raw_angle[2] - ANGLE_ERROR_2;
        diff[3] = host_raw_angle[3] - ANGLE_ERROR_3;
        diff[4] = host_raw_angle[4] - ANGLE_ERROR_4;
        diff[5] = host_raw_angle[5] - ANGLE_ERROR_5;
        diff[6] = host_raw_angle[6] - ANGLE_ERROR_6;
        diff[7] = host_raw_angle[7] - ANGLE_ERROR_7;

        // 符号修正 (根据实际电机安装方向调整)
        set_joint_angle[1] = -diff[1];
        set_joint_angle[2] =  diff[2];
        set_joint_angle[3] =  diff[3];
        set_joint_angle[4] = -diff[4];
        set_joint_angle[5] =  diff[5];
        set_joint_angle[6] =  diff[6];
        set_joint_angle[7] = -diff[7];

    #elif (CURRENT_CONTROL_MODE == CONTROL_MODE_RELATIVE)
        // 相对模式: 记录上电位置为零点
        if (!g_relative_mode_initialized) {
            for(int i = 1; i <= 7; i++) g_initial_raw_angle[i] = host_raw_angle[i];
            g_relative_mode_initialized = true;
            for(int i = 1; i <= 7; i++) set_joint_angle[i] = 0.0f;
        } else {
            float delta[8];
            for(int i = 1; i <= 7; i++) {
                delta[i] = host_raw_angle[i] - g_initial_raw_angle[i];
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
    #endif

        // 安全归一化
        for(int i = 1; i <= 7; i++) {
            set_joint_angle[i] = normalize_angle(set_joint_angle[i]);
        }

        // 计算 FK 用于监控 (可选)
        for(int i=0; i<7; i++) current_joints[i] = set_joint_angle[i+1];
        Calculate_FK(current_joints, &current_pose);

#endif // End of ENABLE_GMR_MAPPING

        osMutexRelease(setJointAngleMutexHandle);
    }
}

// ==============================================================================
//  协议解析层 (保持不变)
// ==============================================================================

static void Protocol_Parse_Byte(uint8_t byte)
{
    switch (g_parse_state) {
        case STATE_WAIT_HEADER:
            if (byte == FRAME_HEADER) g_parse_state = STATE_READ_FUNCTION;
            break;
        case STATE_READ_FUNCTION:
            if (byte == FUNCTION_CODE) {
                g_byte_counter = 0; 
                g_parse_state = STATE_READ_DATA;
            } else {
                if (byte == FRAME_HEADER) g_parse_state = STATE_READ_FUNCTION;
                else g_parse_state = STATE_WAIT_HEADER;
            }
            break;
        case STATE_READ_DATA:
            g_frame_buffer[g_byte_counter++] = byte;
            if (g_byte_counter >= FRAME_DATA_LENGTH) {
                process_encoder_data(g_frame_buffer, FRAME_DATA_LENGTH);
                g_parse_state = STATE_WAIT_HEADER;
            }
            break;
        default: g_parse_state = STATE_WAIT_HEADER; break;
    }
}

void Protocol_Parse_Chunk(uint8_t* chunk, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        Protocol_Parse_Byte(chunk[i]);
    }
}