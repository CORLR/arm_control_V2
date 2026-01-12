#include "arm_control.h"
#include "fdcan.h"
#include "tim.h"
#include "CO_app_STM32.h"
#include "OD.h"
#include "stdint.h"
#include "cmsis_os2.h"
#include "main.h"
#include "arm_math.h"
#include "kinematics.h"

/* ============================================================================== */
/* 宏定义与常量 (FPU 深度优化)                                                   */
/* ============================================================================== */

/* CANopen 对象字典索引 */
#define OD_IDX_CONTROL_WORD         0x6040
#define OD_IDX_STATUS_WORD          0x6041
#define OD_IDX_MODE_OF_OP           0x6060
#define OD_IDX_POS_ACTUAL           0x6064
#define OD_IDX_VEL_ACTUAL           0x606C
#define OD_IDX_POS_TARGET           0x607A
#define OD_IDX_PROFILE_VEL          0x6081
#define OD_IDX_PROFILE_ACC          0x6083
#define OD_IDX_PROFILE_DEC          0x6084

/* 减速比与转换常量 - 强制使用 float 后缀 */
#define RATIO_NODE_4_5              121.0f
#define RATIO_NODE_6                81.0f
#define RATIO_NODE_7                51.0f

#define POS_RESOLUTION              65536.0f
/* 预计算倒数，将运行时除法优化为乘法 */
#define POS_RESOLUTION_INV          (1.0f / POS_RESOLUTION) 

#define PI_2                        (2.0f * PI)
#define PI_2_INV                    (1.0f / PI_2)

/* 硬件限制 */
#define LIMIT_HC_NODE3_MAX          10000
#define LIMIT_HC_NODE3_MIN          -3000000
#define LIMIT_HC_OTHER_MAX          900000
#define LIMIT_HC_OTHER_MIN          -400000
#define LIMIT_TH_NODE4_MAX          2325000
#define LIMIT_TH_NODE4_MIN          -1275000
#define LIMIT_TH_NODE5_MAX          100000
#define LIMIT_TH_NODE5_MIN          -2200000

/* ============================================================================== */
/* 全局变量                                                                      */
/* ============================================================================== */

int16_t dianliu_kp[4] = {0}; 
int16_t dianliu_ki[4] = {0};
int16_t dianliu_kd[4] = {0};
int16_t weizhi_kp[4]  = {0}; 
int16_t weizhi_ki[4]  = {0}; 
int16_t weizhi_kd[4]  = {0}; 

/** * @brief 存储从 protocol 接收到的 Master(人手) 原始角度 
 * 用于 IK 的初始猜测 (Posture Guidance)
 * 索引 1-7 对应 Joint 1-7
 */
float master_joint_angle[8]; 

/** * @brief 存储由 protocol 计算出的末端目标位姿 T_target
 * IK 求解器将努力让机械臂末端到达这个位姿
 */
Matrix4x4 g_TargetPose;      

/** * @brief IK 使能标志位
 * 1: 开启 IK 模式 (由 protocol.c 根据宏定义自动置位)
 * 0: 关闭 IK 模式 (直接透传角度)
 */
uint8_t g_ik_enable;

/* 存放各电机实际位置，下标=节点ID；motor_angle[0]未用 */
float motor_angle[8];       
int32_t motor_pos[8];       
int32_t set_pos[8];         
float set_motor[8];         
float set_joint_angle[8];   
float joint_angle[8];       

extern TIM_HandleTypeDef htim17;
extern FDCAN_HandleTypeDef hfdcan2;
extern CO_t* CO;

/* 缓冲区与标志位 */
uint8_t g_ucTempBuf[20];    
uint8_t g_reTempBuf[20];    
uint32_t g_uiReadSize;

uint32_t speed = SPEED;
uint32_t accelerated = ACCELERATED;
int32_t xianwei;
uint32_t flag;

uint8_t data_error[64];        
uint8_t data_jiaodu[64];        
uint8_t read = 0;              
uint8_t ucStatus = 0;           
uint8_t len = 0;                
uint8_t ucCount = 0;            
uint8_t motor_init = 0;         

CANopenNodeSTM32 canOpenNodeSTM32;      
uint8_t canopen_init_flag = 0;  

/* ============================================================================== */
/* 静态辅助函数 (保留原有打包方式)                                                */
/* ============================================================================== */

/**
 * @brief 将int32数据打包进缓冲区 (Little Endian)
 */
static inline void pack_int32(uint8_t *buf, int32_t data)
{
    buf[0] = (uint8_t)(data & 0xFF);
    buf[1] = (uint8_t)((data >> 8) & 0xFF);
    buf[2] = (uint8_t)((data >> 16) & 0xFF);
    buf[3] = (uint8_t)((data >> 24) & 0xFF);
}

/**
 * @brief 将uint16数据打包进缓冲区 (Little Endian)
 */
static inline void pack_uint16(uint8_t *buf, uint16_t data)
{
    buf[0] = (uint8_t)(data & 0xFF);
    buf[1] = (uint8_t)((data >> 8) & 0xFF);
}

/**
 * @brief 从缓冲区解包int32数据 (Little Endian)
 */
static inline int32_t unpack_int32(const uint8_t *buf)
{
    return (int32_t)((uint32_t)buf[0] |
                     ((uint32_t)buf[1] << 8) |
                     ((uint32_t)buf[2] << 16) |
                     ((uint32_t)buf[3] << 24));
}

/**
 * @brief 获取钛虎电机的减速比 (查找表优化)
 */
static inline float get_taihu_ratio(uint8_t node_id)
{
    /* 使用静态数组代替 switch，减少分支跳转 */
    static const float ratios[] = {RATIO_NODE_4_5, RATIO_NODE_4_5, RATIO_NODE_6, RATIO_NODE_7};
    
    if (node_id >= 4 && node_id <= 7) {
        return ratios[node_id - 4];
    }
    return RATIO_NODE_4_5; 
}

/* ============================================================================== */
/* 主控制逻辑                                                                    */
/* ============================================================================== */

void arm_control(void)
{
    /* 初始化 CANopen 节点参数 */
    canOpenNodeSTM32.CANHandle = &hfdcan2;              
    canOpenNodeSTM32.HWInitFunction = MX_FDCAN2_Init;     
    canOpenNodeSTM32.timerHandle = &htim17;             
    canOpenNodeSTM32.desiredNodeID = 26;                
    canOpenNodeSTM32.baudrate = 1000;                   
    canopen_app_init(&canOpenNodeSTM32);

    OD_RAM.x607A_target_position = 0;

    /* 使用 CMSIS-DSP 快速填充/清零 float 数组 */
    arm_fill_f32(0.0f, motor_angle, 8);
    arm_fill_f32(0.0f, set_motor, 8);
    arm_fill_f32(0.0f, set_joint_angle, 8);
    arm_fill_f32(0.0f, joint_angle, 8);
    
    /* Int 数组使用标准 memset 即可 */
    for(int i=0; i<8; i++) set_pos[i] = 0;

    Mat4_Init(&g_TargetPose);
    Mat4_SetIdentity(&g_TargetPose);

    while (1)
    {
        /* CANopen 协议栈周期性处理 */
        canopen_app_process();

        /* 注册错误回调 */
        if (canopen_init_flag == 0)
        {
            CO_EM_initCallbackRx(CO->em, callback_error);
            canopen_init_flag = 1;
        }

        /* 硬件初始化与运动控制循环 */
        if (!motor_init)
        {
            canopen_init();
            motor_init = 1;
        }
        
        if (motor_init)
        {
            // 4. IK 核心逻辑
            if (g_ik_enable)
            {
                float ik_guess[7], ik_result[7];
                
                // 填入猜测值 (从 1~7 转 0~6)
                for(int i=0; i<7; i++) ik_guess[i] = master_joint_angle[i+1];
                
                // 解算
                Calculate_IK(&g_TargetPose, ik_guess, ik_result);
                
                // 赋值 (从 0~6 转 1~7)
                if(osMutexAcquire(setJointAngleMutexHandle, 0) == osOK) {
                    for(int i=0; i<7; i++) set_joint_angle[i+1] = ik_result[i];
                    osMutexRelease(setJointAngleMutexHandle);
                }
            }

            set_all_motor();            // 逆运动学
            set_all_motor_pos();        // 发送指令
            collect_motor_positions();  // 读取反馈
            collect_joint_angles();     // 正运动学
        }
        
        osDelay(1); // 释放CPU权
    }
}

/* ============================================================================== */
/* 电机控制函数                                                                  */
/* ============================================================================== */

static int32_t last_sent_target[4] = {0}; 
static uint8_t has_sent_first[4] = {0};

void hechuan_motor_setpos(uint8_t node_id, int32_t target)
{
    /* 1. 安全限位检查 */
    if(node_id == 3)
    {
        if (target > LIMIT_HC_NODE3_MAX) target = LIMIT_HC_NODE3_MAX;
        else if (target < LIMIT_HC_NODE3_MIN) target = LIMIT_HC_NODE3_MIN;
    }
    else 
    {
        if (target > LIMIT_HC_OTHER_MAX) target = LIMIT_HC_OTHER_MAX;
        else if (target < LIMIT_HC_OTHER_MIN) target = LIMIT_HC_OTHER_MIN;
    }

    /* 2. 冗余发送过滤 */
    if (has_sent_first[node_id] && (target == last_sent_target[node_id]))
    {
        return; 
    }

    /* 3. 状态字检查 */
    size_t readLen;
    if (read_SDO(CO->SDOclient, node_id, OD_IDX_STATUS_WORD, 0x00, g_reTempBuf, sizeof(g_reTempBuf), &readLen) == CO_SDO_AB_NONE)
    {
        uint16_t status = g_reTempBuf[0] | (g_reTempBuf[1] << 8);
        
        // Bit 3: Fault
        if (status & 0x0008) 
        {
            pack_uint16(g_ucTempBuf, 0x0080); // Fault Reset
            write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
            return; 
        }
        
        // Mask 0x0007, Check if 'Operation Enabled' (0x07)
        if ((status & 0x0007) != 0x07)
        {
            pack_uint16(g_ucTempBuf, 0x000F); // Enable
            write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
        }
    }

    /* 4. 写入目标位置 */
    pack_int32(g_ucTempBuf, target);
    if (write_SDO(CO->SDOclient, node_id, OD_IDX_POS_TARGET, 0x00, g_ucTempBuf, 4) != CO_SDO_AB_NONE)
    {
        return; 
    }

    /* 5. 触发运动 */
    pack_uint16(g_ucTempBuf, 0x000F);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);

    pack_uint16(g_ucTempBuf, 0x003F); // New Set-point
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);

    /* 6. 更新缓存 */
    last_sent_target[node_id] = target;
    has_sent_first[node_id] = 1;
}

int32_t hechuan_read_actual_velocity(uint8_t node_id)
{
    int32_t velocity = 0;
    clear_rebuff(); 

    if (read_SDO(CO->SDOclient, node_id, OD_IDX_VEL_ACTUAL, 0x00,
                 g_reTempBuf, sizeof(g_reTempBuf), &g_uiReadSize) == CO_SDO_AB_NONE
        && g_uiReadSize >= 4) 
    {
        velocity = unpack_int32(g_reTempBuf);
    }
    return velocity;
}

void taihu_motor_setpos(uint8_t node_id, int32_t target)
{
    /* 应用特定节点的软件限位 */
    switch (node_id) 
    {
        case 4:
            if (target > LIMIT_TH_NODE4_MAX) target = LIMIT_TH_NODE4_MAX;
            if (target < LIMIT_TH_NODE4_MIN) target = LIMIT_TH_NODE4_MIN;
            OD_RAM.x6066__607A.ID4607A = target;
            CO_TPDOsendRequest(&CO->TPDO[0]);
            break;
        case 5:
            if (target > LIMIT_TH_NODE5_MAX) target = LIMIT_TH_NODE5_MAX;
            if (target < LIMIT_TH_NODE5_MIN) target = LIMIT_TH_NODE5_MIN;
            OD_RAM.x6066__607A.ID5607A = target;
            CO_TPDOsendRequest(&CO->TPDO[1]);
            break;
        case 6:
            OD_RAM.x6066__607A.ID6607A = target;
            CO_TPDOsendRequest(&CO->TPDO[2]);
            break;
        case 7:
            OD_RAM.x6066__607A.ID7607A = target;
            CO_TPDOsendRequest(&CO->TPDO[3]);
            break;
        default:
            return;
    }

    /* 触发立即执行 */
    pack_uint16(g_ucTempBuf, 0x001F);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
}

/* ============================================================================== */
/* 位置与数据采集                                                                */
/* ============================================================================== */

void collect_motor_positions() 
{
    /* 节点1~3 (禾川): 使用 SDO 轮询读取 */
    for (uint8_t id = 1; id <= 3; id++) 
    {
        if (read_SDO(CO->SDOclient, id, OD_IDX_POS_ACTUAL, 0x00,
                     g_reTempBuf, sizeof(g_reTempBuf), &g_uiReadSize) == CO_SDO_AB_NONE
            && g_uiReadSize >= 4) 
        {
            int32_t raw_pos = unpack_int32(g_reTempBuf);
            /* 优化：使用乘法代替除法 */
            motor_angle[id] = (float)raw_pos * POS_RESOLUTION_INV;
            motor_pos[id]   = raw_pos;
        }
        clear_rebuff();
    }

    /* 节点4~7 (钛虎): 使用 PDO 映射直接读取 */
    motor_pos[4] = OD_RAM.x6065__6064.ID46064;
    motor_pos[5] = OD_RAM.x6065__6064.ID56064;
    motor_pos[6] = OD_RAM.x6065__6064.ID66064;
    motor_pos[7] = OD_RAM.x6065__6064.ID76064;

    motor_angle[4] = taihu_pos_to_angle(CANopenSlaveID4, motor_pos[4]);
    motor_angle[5] = taihu_pos_to_angle(CANopenSlaveID5, motor_pos[5]);
    motor_angle[6] = taihu_pos_to_angle(CANopenSlaveID6, motor_pos[6]);
    motor_angle[7] = taihu_pos_to_angle(CANopenSlaveID7, motor_pos[7]);
}

/* ============================================================================== */
/* 单位转换逻辑 (FPU 优化版)                                                     */
/* ============================================================================== */

float taihu_pos_to_angle(uint8_t node_id, int32_t pos)
{
    float ratio = get_taihu_ratio(node_id);
    /* 优化：Angle = Pos * (1/Ratio) * (1/Res) * 2PI 
       将连续除法转换为乘法链，并保持 float 类型 */
    return ((float)pos / ratio) * POS_RESOLUTION_INV * PI_2;
}

int32_t taihu_angle_to_pos(uint8_t node_id, float angle)
{
    float ratio = get_taihu_ratio(node_id);
    /* 优化：Pos = Angle * (1/2PI) * Ratio * Resolution */
    return (int32_t)(angle * PI_2_INV * ratio * POS_RESOLUTION);
}

float hechuan_pos_to_length(int32_t pos)
{
    /* 优化：乘法代替除法 */
    return (float)pos * POS_RESOLUTION_INV;
}

int32_t hechuan_length_to_pos(float length)
{
    return (int32_t)(length * POS_RESOLUTION);
}

/* ============================================================================== */
/* 运动学算法 (FPU/CMSIS-DSP 优化)                                               */
/* ============================================================================== */

void linear_to_joint_angle()
{
    float L1 = motor_angle[1] + LINEAR_ERROR_1; 
    float L2 = motor_angle[2] + LINEAR_ERROR_2; 
    
    /* 使用 f 后缀的标准数学函数，确保 FPU 调用 */
    joint_angle[7] = atan2f((L2 - L1), CENTER_DIS); 

    /* 优化：/2.0f 改为 *0.5f */
    joint_angle[6] = atan2f(((L1 + L2)) * 0.5f, SHAFT_DIS); 

    float L3 = DEFAULT_LONG - motor_angle[3]; 
    float num = (D_1 * D_1) + (D_2 * D_2) - (L3 * L3);
    float den_inv = 1.0f / (2.0f * D_1 * D_2); /* 建议将 1/(2*D1*D2) 提取为常量预编译 */
    float COS_A = num * den_inv;
    
    /* 简单的限幅 */
    if (COS_A > 1.0f) COS_A = 1.0f;
    else if (COS_A < -1.0f) COS_A = -1.0f;
    
    joint_angle[4] = acosf(COS_A) - DEFAULT_ANGLE;
}

void collect_joint_angles()
{
    linear_to_joint_angle();
    
    /* 旋转关节直接映射 */
    joint_angle[1] = motor_angle[4]; 
    joint_angle[2] = motor_angle[5]; 
    joint_angle[3] = motor_angle[6]; 
    joint_angle[5] = motor_angle[7]; 
}

void joint_angle_to_linear()
{
    float roll = set_joint_angle[7];
    float pitch = set_joint_angle[6];
    
    /* FPU: tanf */
    float common_term = SHAFT_DIS * tanf(pitch);
    /* 优化：除以2 改为 乘以0.5 */
    float diff_term   = (CENTER_DIS * 0.5f) * tanf(roll);

    set_motor[1] = -(common_term - diff_term) - LINEAR_ERROR_1;
    set_motor[2] = -(common_term + diff_term) - LINEAR_ERROR_2;

    float COS_A = cosf(set_joint_angle[4] + DEFAULT_ANGLE);
    
    float L3_sq = (D_1 * D_1) + (D_2 * D_2) - (2.0f * D_1 * D_2 * COS_A);
    
    /* 关键优化：使用 arm_sqrt_f32 (CMSIS-DSP) 
       注：sqrtf 也可以，但 arm_sqrt_f32 显式表明意图 */
    float L3;
    arm_sqrt_f32(L3_sq, &L3);
    
    set_motor[3] = DEFAULT_LONG - L3;
}

void set_all_motor()
{
    if(osMutexAcquire(setJointAngleMutexHandle, 0) == osOK)
    {
        joint_angle_to_linear();
        set_motor[4] = set_joint_angle[1];
        set_motor[5] = set_joint_angle[2];
        set_motor[6] = set_joint_angle[3];
        set_motor[7] = set_joint_angle[5];
        osMutexRelease(setJointAngleMutexHandle);
    }
}

void set_all_motor_pos()
{
    /* 转换物理量到编码器数值 */
    set_pos[1] = hechuan_length_to_pos(set_motor[1]);
    set_pos[2] = hechuan_length_to_pos(set_motor[2]);
    set_pos[3] = hechuan_length_to_pos(set_motor[3]);
    set_pos[4] = taihu_angle_to_pos(CANopenSlaveID4, set_motor[4]);
    set_pos[5] = taihu_angle_to_pos(CANopenSlaveID5, set_motor[5]);
    set_pos[6] = taihu_angle_to_pos(CANopenSlaveID6, set_motor[6]);
    set_pos[7] = taihu_angle_to_pos(CANopenSlaveID7, set_motor[7]);

    /* 发送指令 */
    hechuan_motor_setpos(CANopenSlaveID1, set_pos[1]);
    hechuan_motor_setpos(CANopenSlaveID2, set_pos[2]);
    hechuan_motor_setpos(CANopenSlaveID3, set_pos[3]);
    taihu_motor_setpos(CANopenSlaveID4, set_pos[4]);
    taihu_motor_setpos(CANopenSlaveID5, set_pos[5]);
    taihu_motor_setpos(CANopenSlaveID6, set_pos[6]);
    taihu_motor_setpos(CANopenSlaveID7, set_pos[7]);
}

/* ============================================================================== */
/* CANopen 初始化                                                                */
/* ============================================================================== */

void clear_rebuff(void)
{
    for (int i = 0; i < 20; i++) g_reTempBuf[i] = 0;
}

void callback_error(const uint16_t ident, const uint16_t errorCode, const uint8_t errorRegister, const uint8_t errorBit, const uint32_t infoCode)
{
    data_error[0] = 0xFF;
    data_error[1] = 0x05;
    data_error[2] = 0x03;
    data_error[3] = ident & 0xFF;
    pack_uint16(&data_error[4], errorCode);
    data_error[6] = errorRegister & 0xFF;
    data_error[7] = errorBit & 0xFF;
}

void canopen_init()
{
    canopen_init_hechuan(CANopenSlaveID1);
    canopen_init_hechuan(CANopenSlaveID2);
    canopen_init_hechuan(CANopenSlaveID3);
    canopen_init_taihu(CANopenSlaveID4);
    canopen_init_taihu(CANopenSlaveID5);
    canopen_init_taihu(CANopenSlaveID6);
    canopen_init_taihu(CANopenSlaveID7);
}

void canopen_init_taihu(uint8_t node_id) 
{
    /* RPDO1 */
    g_ucTempBuf[0] = node_id; g_ucTempBuf[1] = 0x02; g_ucTempBuf[2] = 0x00; g_ucTempBuf[3] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x1400, 0x01, g_ucTempBuf, 4);

    g_ucTempBuf[0] = 0x20; g_ucTempBuf[1] = 0x00; g_ucTempBuf[2] = 0x7A; g_ucTempBuf[3] = 0x60;
    write_SDO(CO->SDOclient, node_id, 0x1600, 0x01, g_ucTempBuf, 4);

    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1600, 0x00, g_ucTempBuf, 1);

    /* TPDO1 */
    g_ucTempBuf[0] = 0x80 + node_id; g_ucTempBuf[1] = 0x01; g_ucTempBuf[2] = 0x00; g_ucTempBuf[3] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x1800, 0x01, g_ucTempBuf, 4);

    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1800, 0x02, g_ucTempBuf, 1);

    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1800, 0x06, g_ucTempBuf, 1);

    g_ucTempBuf[0] = 0x20; g_ucTempBuf[1] = 0x00; g_ucTempBuf[2] = 0x64; g_ucTempBuf[3] = 0x60;
    write_SDO(CO->SDOclient, node_id, 0x1A00, 0x01, g_ucTempBuf, 4);

    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1A00, 0x00, g_ucTempBuf, 1);

    /* 软件限位 */
    pack_int32(g_ucTempBuf, 0x80000002);
    write_SDO(CO->SDOclient, node_id, 0x607D, 0x01, g_ucTempBuf, 4);

    pack_int32(g_ucTempBuf, 0x7FFFFFFE);
    write_SDO(CO->SDOclient, node_id, 0x607D, 0x02, g_ucTempBuf, 4);

    /* 运动参数 */
    pack_uint16(g_ucTempBuf, (uint16_t)speed);
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_VEL, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, (uint16_t)accelerated);
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_ACC, 0x00, g_ucTempBuf, 2);

    pack_uint16(g_ucTempBuf, (uint16_t)accelerated);
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_DEC, 0x00, g_ucTempBuf, 2);
    
    /* 模式与启动 */
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, OD_IDX_MODE_OF_OP, 0x00, g_ucTempBuf, 1);

    pack_uint16(g_ucTempBuf, 0x0006);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, 0x0007);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, 0x000F);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, 0x001F);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
}

void canopen_init_hechuan(uint8_t node_id) 
{
    int32_t max_limit = 3000000; 
    int32_t fast_acc = 3000000; 
    int32_t speed_val = 2000000;

    pack_int32(g_ucTempBuf, max_limit);
    write_SDO(CO->SDOclient, node_id, 0x607F, 0x00, g_ucTempBuf, 4);
    
    write_SDO(CO->SDOclient, node_id, 0x6080, 0x00, g_ucTempBuf, 4);

    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, OD_IDX_MODE_OF_OP, 0x00, g_ucTempBuf, 1); 

    pack_int32(g_ucTempBuf, fast_acc);
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_ACC, 0x00, g_ucTempBuf, 4); 
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_DEC, 0x00, g_ucTempBuf, 4); 

    pack_int32(g_ucTempBuf, speed_val);
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_VEL, 0x00, g_ucTempBuf, 4);

    pack_uint16(g_ucTempBuf, 0x0006);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, 0x0007);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, 0x000F);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
}

/* ============================================================================== */
/* CANopen SDO 底层                                                              */
/* ============================================================================== */

CO_SDO_abortCode_t write_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                             uint16_t index, uint8_t subIndex,
                             uint8_t *data, size_t dataSize)
{
    CO_SDO_return_t SDO_ret;
    bool_t bufferPartial = false;
 
    SDO_ret = CO_SDOclient_setup(SDO_C,
                                 CO_CAN_ID_SDO_CLI + nodeId,
                                 CO_CAN_ID_SDO_SRV + nodeId,
                                 nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) return -1;
 
    SDO_ret = CO_SDOclientDownloadInitiate(SDO_C, index, subIndex, dataSize, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) return -1;
 
    size_t nWritten = CO_SDOclientDownloadBufWrite(SDO_C, data, dataSize);
    if (nWritten < dataSize) bufferPartial = true;
 
    do {
        uint32_t timeDifference_us = 500;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
 
        SDO_ret = CO_SDOclientDownload(SDO_C, timeDifference_us, false, bufferPartial, &abortCode, NULL, NULL);
        if (SDO_ret < 0) return abortCode;
 
        if (SDO_ret > 0) osDelay(1);
    } while(SDO_ret > 0);
 
    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t read_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                            uint16_t index, uint8_t subIndex,
                            uint8_t *buf, size_t bufSize, size_t *readSize)
{
    CO_SDO_return_t SDO_ret;
 
    SDO_ret = CO_SDOclient_setup(SDO_C,
                                 CO_CAN_ID_SDO_CLI + nodeId,
                                 CO_CAN_ID_SDO_SRV + nodeId,
                                 nodeId); 
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) return CO_SDO_AB_GENERAL;
 
    SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) return CO_SDO_AB_GENERAL;
 
    do {
        uint32_t timeDifference_us = 500;
        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;
 
        SDO_ret = CO_SDOclientUpload(SDO_C, timeDifference_us, false, &abortCode, NULL, NULL, NULL);
        if (SDO_ret < 0) return abortCode;
 
        if (SDO_ret > 0) osDelay(1);
    } while(SDO_ret > 0);
 
    *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);
 
    return CO_SDO_AB_NONE;
}