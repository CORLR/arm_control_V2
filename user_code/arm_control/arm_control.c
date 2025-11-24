#include "arm_control.h"
#include "fdcan.h"
#include "tim.h"
#include "CO_app_STM32.h"
#include "OD.h"
#include "stdint.h"
#include "cmsis_os2.h"
#include "main.h"
#include "math.h"

/* ============================================================================== */
/* 宏定义与常量                                  */
/* ============================================================================== */

/* CANopen 对象字典索引 */
#define OD_IDX_CONTROL_WORD         0x6040  // 控制字
#define OD_IDX_STATUS_WORD          0x6041  // 状态字
#define OD_IDX_MODE_OF_OP           0x6060  // 运行模式
#define OD_IDX_POS_ACTUAL           0x6064  // 实际位置
#define OD_IDX_VEL_ACTUAL           0x606C  // 实际速度
#define OD_IDX_POS_TARGET           0x607A  // 目标位置
#define OD_IDX_PROFILE_VEL          0x6081  // 轮廓速度
#define OD_IDX_PROFILE_ACC          0x6083  // 轮廓加速度
#define OD_IDX_PROFILE_DEC          0x6084  // 轮廓减速度

/* 减速比与转换常量 */
#define RATIO_NODE_4_5              121.0f
#define RATIO_NODE_6                81.0f
#define RATIO_NODE_7                51.0f
#define POS_RESOLUTION              65536.0f // 16-bit 细分
#define PI_2                        (2.0f * PI)

/* 硬件限制 (保护逻辑保持原样) */
#define LIMIT_HC_NODE3_MAX          10000
#define LIMIT_HC_NODE3_MIN          -3000000
#define LIMIT_HC_OTHER_MAX          900000
#define LIMIT_HC_OTHER_MIN          -400000
#define LIMIT_TH_NODE4_MAX          2325000
#define LIMIT_TH_NODE4_MIN          -1275000
#define LIMIT_TH_NODE5_MAX          100000
#define LIMIT_TH_NODE5_MIN          -2200000

/* ============================================================================== */
/* 全局变量                                      */
/* ============================================================================== */

int16_t dianliu_kp[4] = {0}; 
int16_t dianliu_ki[4] = {0};
int16_t dianliu_kd[4] = {0};
int16_t weizhi_kp[4]  = {0}; 
int16_t weizhi_ki[4]  = {0}; 
int16_t weizhi_kd[4]  = {0}; 

/* 存放各电机实际位置，下标=节点ID；motor_angle[0]未用 */
float motor_angle[8];       // 存储当前电机角度(rad)和长度(mm)数据
int32_t motor_pos[8];       // 编码器原始数值
int32_t set_pos[8];         // 目标编码器数值
float set_motor[8];         // 目标物理量 (1-3:长度mm, 4-7:角度rad)
float set_joint_angle[8];   // 设置目标关节角度
float joint_angle[8];       // 存储当前关节角度

extern TIM_HandleTypeDef htim17;
extern FDCAN_HandleTypeDef hfdcan2;
extern CO_t* CO;

/* 缓冲区与标志位 */
uint8_t g_ucTempBuf[20];    // 发送缓冲区
uint8_t g_reTempBuf[20];    // 接收缓冲区
uint32_t g_uiReadSize;

uint32_t speed = SPEED;
uint32_t accelerated = ACCELERATED;
int32_t xianwei;
uint32_t flag;

uint8_t data_error[64];         // 错误信息上报缓冲
uint8_t data_jiaodu[64];        // 角度数据上报缓冲
uint8_t read = 0;               // 串口当前字节
uint8_t ucStatus = 0;           // 协议状态
uint8_t len = 0;                // 长度
uint8_t ucCount = 0;            // 计数
uint8_t motor_init = 0;         // 电机初始化完成标志

CANopenNodeSTM32 canOpenNodeSTM32;      
uint8_t canopen_init_flag = 0;  // CANopen栈初始化完成标志

/* ============================================================================== */
/* 静态辅助函数                                  */
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
 * @brief 获取钛虎电机的减速比
 */
static inline float get_taihu_ratio(uint8_t node_id)
{
    switch(node_id) {
        case 6: return RATIO_NODE_6;
        case 7: return RATIO_NODE_7;
        default: return RATIO_NODE_4_5; // Node 4 and 5
    }
}

/* ============================================================================== */
/* 主控制逻辑                                    */
/* ============================================================================== */

void arm_control(void)
{
    /* 初始化 CANopen 节点参数 */
    canOpenNodeSTM32.CANHandle = &hfdcan2;              
    canOpenNodeSTM32.HWInitFunction = MX_FDCAN2_Init;     
    canOpenNodeSTM32.timerHandle = &htim17;             
    canOpenNodeSTM32.desiredNodeID = 26;                
    canOpenNodeSTM32.baudrate = 1000;                   /* 1MHz */
    canopen_app_init(&canOpenNodeSTM32);

    OD_RAM.x607A_target_position = 0;

    /* 清空所有状态 */
    for (int i = 0; i < 8; i++) 
    {
        motor_angle[i] = 0;
        set_pos[i] = 0;
        set_motor[i] = 0;
        set_joint_angle[i] = 0;
        joint_angle[i] = 0;
    }

    while (1)
    {
        /* CANopen 协议栈周期性处理 */
        canopen_app_process();

        /* 注册错误回调 (仅执行一次) */
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
            set_all_motor();            // 计算目标电机值 (逆运动学)
            set_all_motor_pos();        // 发送控制指令
            collect_motor_positions();  // 读取当前状态
            collect_joint_angles();     // 计算关节角度 (正运动学)
        }
        
        osDelay(1); // 释放CPU权
    }
}

/* ============================================================================== */
/* 电机控制函数                                  */
/* ============================================================================== */

/**
 * @brief 设置禾川电机目标位置 (通过SDO)
 * 包含状态检查：Fault Reset -> Enable -> Set Position -> New Setpoint
 */
static int32_t last_sent_target[4] = {0}; 
static uint8_t has_sent_first[4] = {0};

void hechuan_motor_setpos(uint8_t node_id, int32_t target)
{
    /* 1. 安全限位检查 */
    if(node_id == 3)
    {
        if (target > LIMIT_HC_NODE3_MAX) target = LIMIT_HC_NODE3_MAX;
        if (target < LIMIT_HC_NODE3_MIN) target = LIMIT_HC_NODE3_MIN;
    }
    else 
    {
        if (target > LIMIT_HC_OTHER_MAX) target = LIMIT_HC_OTHER_MAX;
        if (target < LIMIT_HC_OTHER_MIN) target = LIMIT_HC_OTHER_MIN;
    }

    /* 2. 冗余发送过滤 */
    if (has_sent_first[node_id] && (target == last_sent_target[node_id]))
    {
        return; 
    }

    /* 3. 状态字检查 (0x6041) */
    size_t readLen;
    if (read_SDO(CO->SDOclient, node_id, OD_IDX_STATUS_WORD, 0x00, g_reTempBuf, sizeof(g_reTempBuf), &readLen) == CO_SDO_AB_NONE)
    {
        uint16_t status = g_reTempBuf[0] | (g_reTempBuf[1] << 8);
        
        // Bit 3: Fault
        if (status & 0x0008) 
        {
            // 发送 Fault Reset (0x0080)
            pack_uint16(g_ucTempBuf, 0x0080);
            write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
            return; // 正在复位中，暂不发送位置
        }
        
        // Mask 0x0007, Check if 'Operation Enabled' (0x07)
        if ((status & 0x0007) != 0x07)
        {
            // 尝试重新使能 (0x000F)
            pack_uint16(g_ucTempBuf, 0x000F);
            write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
        }
    }

    /* 4. 写入目标位置 (0x607A) */
    pack_int32(g_ucTempBuf, target);
    if (write_SDO(CO->SDOclient, node_id, OD_IDX_POS_TARGET, 0x00, g_ucTempBuf, 4) != CO_SDO_AB_NONE)
    {
        return; // 通讯繁忙
    }

    /* 5. 触发运动 (Control Word toggle) */
    // Enable Operation (0x000F)
    pack_uint16(g_ucTempBuf, 0x000F);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);

    // New Set-point + Enable Operation (0x003F: Bit 4 is New Set-point)
    pack_uint16(g_ucTempBuf, 0x003F);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);

    /* 6. 更新缓存 */
    last_sent_target[node_id] = target;
    has_sent_first[node_id] = 1;
}

/**
 * @brief 读取禾川电机实际速度
 */
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

/**
 * @brief 设置钛虎电机目标位置 (通过 TPDO 映射的 OD_RAM)
 */
void taihu_motor_setpos(uint8_t node_id, int32_t target)
{
    // 应用特定节点的软件限位
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

    // 触发立即执行 (0x001F: Bit 4 New Setpoint in Profile Position Mode often implies immediate update if configured)
    pack_uint16(g_ucTempBuf, 0x001F);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
}

/* ============================================================================== */
/* 位置与数据采集                                */
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
            motor_angle[id] = hechuan_pos_to_length(raw_pos);
            motor_pos[id]   = raw_pos;
        }
        clear_rebuff();
    }

    /* 节点4~7 (钛虎): 使用 PDO 映射直接读取内存 */
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
/* 单位转换逻辑                                  */
/* ============================================================================== */

float taihu_pos_to_angle(uint8_t node_id, int32_t pos)
{
    float ratio = get_taihu_ratio(node_id);
    // Angle = (Pos / Ratio / Resolution) * 2PI
    return (float)pos / ratio / POS_RESOLUTION * PI_2;
}

int32_t taihu_angle_to_pos(uint8_t node_id, float angle)
{
    float ratio = get_taihu_ratio(node_id);
    // Pos = (Angle / 2PI) * Ratio * Resolution
    return (int32_t)(angle / PI_2 * ratio * POS_RESOLUTION);
}

float hechuan_pos_to_length(int32_t pos)
{
    return (float)pos / POS_RESOLUTION;
}

int32_t hechuan_length_to_pos(float length)
{
    return (int32_t)(length * POS_RESOLUTION);
}

/* ============================================================================== */
/* 运动学算法                                    */
/* ============================================================================== */

/**
 * @brief 直线电机长度 -> 关节角度 (正运动学)
 * * 1号和2号电机控制 Pitch 和 Roll：
 * - Roll 由长度差决定
 * - Pitch 由平均长度决定
 * 3号电机控制肘关节：
 * - 余弦定理计算角度
 */
void linear_to_joint_angle()
{
    float L1 = motor_angle[1] + LINEAR_ERROR_1; // 1号电机绝对长度
    float L2 = motor_angle[2] + LINEAR_ERROR_2; // 2号电机绝对长度
    
    // 计算 Roll (atan2((L2-L1), Width))
    joint_angle[7] = atan2f((L2 - L1), CENTER_DIS); 

    // 计算 Pitch (atan2(Avg_L, Height))
    joint_angle[6] = atan2f(((L1 + L2)) / 2.0f, SHAFT_DIS); 

    // 计算 Elbow Angle (余弦定理)
    float L3 = DEFAULT_LONG - motor_angle[3]; 
    float num = (D_1 * D_1) + (D_2 * D_2) - (L3 * L3);
    float den = 2.0f * D_1 * D_2;
    float COS_A = num / den;
    
    // 保护 acos 输入范围
    if (COS_A > 1.0f) COS_A = 1.0f;
    if (COS_A < -1.0f) COS_A = -1.0f;
    
    joint_angle[4] = acosf(COS_A) - DEFAULT_ANGLE;
}

void collect_joint_angles()
{
    linear_to_joint_angle();
    // 旋转关节直接映射
    joint_angle[1] = motor_angle[4]; 
    joint_angle[2] = motor_angle[5]; 
    joint_angle[3] = motor_angle[6]; 
    joint_angle[5] = motor_angle[7]; 
}

/**
 * @brief 关节角度 -> 直线电机长度 (逆运动学)
 * * L1 = H*tan(pitch) - (W/2)*tan(roll)
 * L2 = H*tan(pitch) + (W/2)*tan(roll)
 * L3 = sqrt(a^2 + b^2 - 2ab*cos(theta))
 */
void joint_angle_to_linear()
{
    float roll = set_joint_angle[7];
    float pitch = set_joint_angle[6];
    
    float common_term = SHAFT_DIS * tanf(pitch);
    float diff_term   = (CENTER_DIS / 2.0f) * tanf(roll);

    set_motor[1] = -(common_term - diff_term) - LINEAR_ERROR_1;
    set_motor[2] = -(common_term + diff_term) - LINEAR_ERROR_2;

    float COS_A = cosf(set_joint_angle[4] + DEFAULT_ANGLE);
    float L3 = sqrtf((D_1 * D_1) + (D_2 * D_2) - (2.0f * D_1 * D_2 * COS_A));
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
    set_pos[5] = taihu_angle_to_pos(CANopenSlaveID5, set_motor[5]); // ID5 uses same logic as ID4/5 in function
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
/* CANopen 初始化                                */
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

/**
 * @brief 初始化钛虎电机的CANopen参数
 * 配置 PDO 映射、同步模式、速度/加速度限制以及软件限位
 */
void canopen_init_taihu(uint8_t node_id) 
{
    /* 1. 配置 RPDO1 (Receive PDO 1) */
    // COB-ID (0x1400:01)
    g_ucTempBuf[0] = node_id; g_ucTempBuf[1] = 0x02; g_ucTempBuf[2] = 0x00; g_ucTempBuf[3] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x1400, 0x01, g_ucTempBuf, 4);

    // Mapping (0x1600:01) -> Target Position (0x607A)
    g_ucTempBuf[0] = 0x20; g_ucTempBuf[1] = 0x00; g_ucTempBuf[2] = 0x7A; g_ucTempBuf[3] = 0x60;
    write_SDO(CO->SDOclient, node_id, 0x1600, 0x01, g_ucTempBuf, 4);

    // Number of mapped objects (0x1600:00) = 1
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1600, 0x00, g_ucTempBuf, 1);

    /* 2. 配置 TPDO1 (Transmit PDO 1) */
    // COB-ID (0x1800:01) -> 0x180 + NodeID
    g_ucTempBuf[0] = 0x80 + node_id; g_ucTempBuf[1] = 0x01; g_ucTempBuf[2] = 0x00; g_ucTempBuf[3] = 0x00;
    write_SDO(CO->SDOclient, node_id, 0x1800, 0x01, g_ucTempBuf, 4);

    // Transmission Type (0x1800:02) = 1 (Synchronous per SYNC)
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1800, 0x02, g_ucTempBuf, 1);

    // Event Timer (0x1800:06) = 1ms
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1800, 0x06, g_ucTempBuf, 1);

    // Mapping (0x1A00:01) -> Actual Position (0x6064)
    g_ucTempBuf[0] = 0x20; g_ucTempBuf[1] = 0x00; g_ucTempBuf[2] = 0x64; g_ucTempBuf[3] = 0x60;
    write_SDO(CO->SDOclient, node_id, 0x1A00, 0x01, g_ucTempBuf, 4);

    // Number of mapped objects
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, 0x1A00, 0x00, g_ucTempBuf, 1);

    /* 3. 软件限位 (0x607D) */
    // Min Limit (0x80000002)
    pack_int32(g_ucTempBuf, 0x80000002);
    write_SDO(CO->SDOclient, node_id, 0x607D, 0x01, g_ucTempBuf, 4);

    // Max Limit (0x7FFFFFFE)
    pack_int32(g_ucTempBuf, 0x7FFFFFFE);
    write_SDO(CO->SDOclient, node_id, 0x607D, 0x02, g_ucTempBuf, 4);

    /* 4. 运动参数 (Velocity, Acc, Dec) */
    pack_uint16(g_ucTempBuf, (uint16_t)speed);
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_VEL, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, (uint16_t)accelerated);
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_ACC, 0x00, g_ucTempBuf, 2);

    pack_uint16(g_ucTempBuf, (uint16_t)accelerated);
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_DEC, 0x00, g_ucTempBuf, 2);
    
    /* 5. 设置模式与状态机启动 */
    // Mode = 1 (Profile Position Mode)
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, OD_IDX_MODE_OF_OP, 0x00, g_ucTempBuf, 1);

    // CiA 402 State Machine: Shutdown(06) -> SwitchOn(07) -> EnableOp(0F) -> Start(1F)
    pack_uint16(g_ucTempBuf, 0x0006);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, 0x0007);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, 0x000F);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, 0x001F);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
}

/**
 * @brief 初始化禾川电机的CANopen参数
 */
void canopen_init_hechuan(uint8_t node_id) 
{
    int32_t max_limit = 3000000; 
    int32_t fast_acc = 3000000; 
    int32_t speed_val = 2000000;

    // Max Profile Velocity (0x607F)
    pack_int32(g_ucTempBuf, max_limit);
    write_SDO(CO->SDOclient, node_id, 0x607F, 0x00, g_ucTempBuf, 4);
    
    // Profile Acceleration (0x6080) - 实际上禾川这里可能需要写，但原代码未赋新值仅调用了Write
    write_SDO(CO->SDOclient, node_id, 0x6080, 0x00, g_ucTempBuf, 4);

    // Mode = 1 (Profile Position)
    g_ucTempBuf[0] = 0x01;
    write_SDO(CO->SDOclient, node_id, OD_IDX_MODE_OF_OP, 0x00, g_ucTempBuf, 1); 

    // Acc (0x6083) / Dec (0x6084)
    pack_int32(g_ucTempBuf, fast_acc);
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_ACC, 0x00, g_ucTempBuf, 4); 
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_DEC, 0x00, g_ucTempBuf, 4); 

    // Profile Velocity (0x6081)
    pack_int32(g_ucTempBuf, speed_val);
    write_SDO(CO->SDOclient, node_id, OD_IDX_PROFILE_VEL, 0x00, g_ucTempBuf, 4);

    // State Machine: Shutdown(06) -> SwitchOn(07) -> EnableOp(0F)
    pack_uint16(g_ucTempBuf, 0x0006);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, 0x0007);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
    
    pack_uint16(g_ucTempBuf, 0x000F);
    write_SDO(CO->SDOclient, node_id, OD_IDX_CONTROL_WORD, 0x00, g_ucTempBuf, 2);
}

/* ============================================================================== */
/* CANopen 底层封装                              */
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
 
        osDelay(timeDifference_us / 1000);
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
 
        osDelay(timeDifference_us / 1000);
    } while(SDO_ret > 0);
 
    *readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);
 
    return CO_SDO_AB_NONE;
}