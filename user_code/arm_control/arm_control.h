#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "CO_app_STM32.h"
#include "fdcan.h"
#include "stdint.h"
#include "cmsis_os.h"

/*
*********************************************************************************************************
*	                                           宏定义
*********************************************************************************************************
*/
#define CANopenSlaveID1   0x01  //禾川  行程35mm    推测 位置 = 目标长度 * 65536  长度 = 位置 / 65536 导程2mm，编码器17bit
#define CANopenSlaveID2   0x02  //禾川  行程35mm
#define CANopenSlaveID3   0x03  //禾川  行程70mm
#define CANopenSlaveID4   0x04  //钛虎  121减速比   位置 = 目标角度 / 360 * 121 * 65536 角度 = 位置 / 121 / 65536 * 360
#define CANopenSlaveID5   0x05  //钛虎  121减速比   位置 = 目标角度 / 360 * 121 * 65536 角度 = 位置 / 121 / 65536 * 360
#define CANopenSlaveID6   0x06  //钛虎  81减速比    位置 = 目标角度 / 360 * 81 * 65536  角度 = 位置 / 81 / 65536 * 360
#define CANopenSlaveID7   0x07  //钛虎  51减速比    位置 = 目标角度 / 360 * 51 * 65536  角度 = 位置 / 51 / 65536 * 360


#define pid_dianliu 0x60F6 //电流环PID
#define pid_weizhi  0x60FB //位置环PID
#define pid_sudu    0x60f9 //速度环PID

#define pid_dianliu_4_kp 0x2
#define pid_dianliu_4_ki 0x1
#define pid_dianliu_5_kp 0x1
#define pid_dianliu_5_ki 0x1
#define pid_dianliu_6_kp 0x1
#define pid_dianliu_6_ki 0x1
#define pid_dianliu_7_kp 0x1 
#define pid_dianliu_7_ki 0x1

#define pid_weizhi_4_kp  0x1
#define pid_weizhi_4_ki  0x1
#define pid_weizhi_4_kd  0x1
#define pid_weizhi_5_kp  0x1
#define pid_weizhi_5_ki  0x1
#define pid_weizhi_5_kd  0x1
#define pid_weizhi_6_kp  0x1
#define pid_weizhi_6_ki  0x1
#define pid_weizhi_6_kd  0x1
#define pid_weizhi_7_kp  0x1
#define pid_weizhi_7_ki  0x1 
#define pid_weizhi_7_kd  0x1

#define D_1 289.01                  //连杆AB长度转轴到电机固定端
#define D_2 60.0                    //连杆AC长度转轴到电机活动端
#define DEFAULT_LONG 264.18         //3号电机初始长度
#define DEFAULT_ANGLE 1.05842024         //初始角度补偿
#define CENTER_DIS  48.0            //1号电机和2号电机中点距离
#define DEFAULT_HIGHT  168.7        //固定杆的长度
#define SHAFT_DIS  18.0             //1号电机和2号电机转轴中心平面到旋转轴的距离
#define LINEAR_ERROR_1  1.26f            //直线电机长度误差补偿
#define LINEAR_ERROR_2  0.17f           //直线电机长度误差补偿

#define PI               3.14159265358979f

#define ANGLE_ERROR_1 1.65f
#define ANGLE_ERROR_2 2.22f
#define ANGLE_ERROR_3 0.632f
#define ANGLE_ERROR_4 0.64f
#define ANGLE_ERROR_5 2.87f
#define ANGLE_ERROR_6 -2.22f
#define ANGLE_ERROR_7 -0.35f

#define SPEED 5000
#define ACCELERATED 1000

/*
*********************************************************************************************************
*	                                            变量
*********************************************************************************************************
*/
extern osMutexId_t setJointAngleMutexHandle;//互斥锁


extern int16_t dianliu_kp[4]; 
extern int16_t dianliu_ki[4];
extern int16_t dianliu_kd[4];
extern int16_t weizhi_kp[4] ; 
extern int16_t weizhi_ki[4] ; 
extern int16_t weizhi_kd[4] ; 

/* 存放各电机实际位置，下标=节点ID；motor_angle[0]未用 */
extern float motor_angle[8];       //存储当前电机角度和长度数据，电机禾川电机，压缩时读数变大，伸展时读数变小
extern int32_t motor_pos[8];
extern int32_t set_pos[8];         //pos[1]~pos[7] 设置 1~7号电机编码器目标位置
extern float set_motor[8];         //set_motor[1]~set_motor[7] 设置 1~3号电机目标长度，4~7号电机目标角度
extern float set_joint_angle[8];   //设置目标关节角度
extern float joint_angle[8];       //存储当前关节角度


extern TIM_HandleTypeDef htim17;
extern CANopenNodeSTM32 canOpenNodeSTM32;
// extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern CO_t* CO;
// CO_CANrxMsg_t rcvMsg;

extern uint8_t g_ucTempBuf[20];
extern uint8_t g_reTempBuf[20];
extern uint32_t g_uiReadSize;

extern uint32_t speed;
extern uint32_t accelerated;

extern int32_t xianwei;

extern uint32_t flag;

// uint32_t obj_ledvarID1;
// uint32_t obj_ledvarID2;



/*
*********************************************************************************************************
*	                                            函数
*********************************************************************************************************
*/
void clear_rebuff(void);                         // 清空接收缓冲区
void callback_error(const uint16_t ident, const uint16_t errorCode, const uint8_t errorRegister, const uint8_t errorBit, const uint32_t infoCode); // CANopen错误回调
void taihu_motor_setpos(uint8_t node_id, int32_t target);   // 钛虎电机单独控制目标位置
void hechuan_motor_setpos(uint8_t node_id, int32_t target); // 禾川电机单独控制目标位置
void collect_motor_positions();                // 收集所有电机位置
float taihu_pos_to_angle(uint8_t node_id, int32_t pos);      // 钛虎电机位置转角度
int32_t taihu_angle_to_pos(uint8_t node_id, float angle);   // 钛虎电机角度转位置
float hechuan_pos_to_length(int32_t pos);                    // 禾川电机位置转长度
int32_t hechuan_length_to_pos(float length);           // 禾川电机长度转位置
void linear_to_joint_angle();                                 //直线电机长度转关节角度
void collect_joint_angles();                               //收集所有关节角度
void joint_angle_to_linear();                               //关节角度转直线电机长度
void set_all_motor();                                   //根据关节角度设置所有电机目标角度和长度
void set_all_motor_pos();                                //根据电机目标角度长度设置所有电机编码器位置
void canopen_init();                                      // 电机CANopen初始化
void canopen_init_taihu(uint8_t node_id);                     // 钛虎电机CANopen初始化
void canopen_init_hechuan(uint8_t node_id);                   // 禾川电机CANopen初始化

CO_SDO_abortCode_t write_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                             uint16_t index, uint8_t subIndex,
                             uint8_t *data, size_t dataSize); // SDO写入函数
CO_SDO_abortCode_t read_SDO(CO_SDOclient_t *SDO_C, uint8_t nodeId,
                            uint16_t index, uint8_t subIndex,
                            uint8_t *buf, size_t bufSize, size_t *readSize); // SDO读取函数




extern uint8_t data_error[64];  // 存放错误信息并通过串口上报的缓冲区
extern uint8_t data_jiaodu[64]; // 存放电机角度数据并通过串口上报的缓冲区
extern uint8_t read;              // 串口接收的当前字节
extern uint8_t ucStatus;          // 串口协议解析状态机当前状态
extern uint8_t len;               // 当前接收数据包的长度
extern uint8_t ucCount;           // 当前已接收数据字节计数
extern uint8_t init_flag;         // CANopen初始化完成标志
extern uint8_t motor_init;       // 电机初始化完成标志

#endif