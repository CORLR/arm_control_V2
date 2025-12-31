#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "arm_math.h"
#include "main.h"

// 机械臂自由度 (根据你的URDF是7轴)
#define JOINT_COUNT 7

// 定义一个便于管理的 4x4 矩阵结构体
// 这样可以把数据内存和 CMSIS 实例绑在一起
typedef struct {
    arm_matrix_instance_f32 instance; // CMSIS 矩阵实例句柄
    float32_t data[16];               // 实际存储数据的数组 (4x4 flattened, Row-major)
} Matrix4x4;

/* 基础矩阵运算函数 */
void Mat4_Init(Matrix4x4 *mat);
void Mat4_Multiply(Matrix4x4 *out, Matrix4x4 *A, Matrix4x4 *B);
int Mat4_Inverse(Matrix4x4 *out, Matrix4x4 *in);
void Mat4_FromDH(Matrix4x4 *mat, float theta, float d, float a, float alpha);

/* 核心运动学函数 */
/**
 * @brief 正运动学解算 (FK)
 * @param angles 输入: 7个关节的角度 (单位: 弧度)
 * @param EndEffector 输出: 末端执行器的位姿矩阵 (4x4)
 */
void Calculate_FK(float *angles, Matrix4x4 *EndEffector);

#endif // KINEMATICS_H