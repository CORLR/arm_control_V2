#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "arm_math.h"
#include "main.h" // 包含项目全局定义

// 机械臂自由度
#define JOINT_COUNT 7

// 4x4 矩阵结构体定义
typedef struct {
    arm_matrix_instance_f32 instance;
    float32_t data[16]; // 4x4 Row-major storage
} Matrix4x4;

/* ================== 基础矩阵运算 ================== */
void Mat4_Init(Matrix4x4 *mat);
void Mat4_SetIdentity(Matrix4x4 *mat);
void Mat4_Multiply(Matrix4x4 *out, Matrix4x4 *A, Matrix4x4 *B);
int Mat4_Inverse(Matrix4x4 *out, Matrix4x4 *in);
void Mat4_FromDH(Matrix4x4 *mat, float theta, float d, float a, float alpha);

/* ================== 核心运动学函数 ================== */

/**
 * @brief 通用正运动学 (支持自定义DH表)
 * @details 用于在 protocol.c 中计算人体(Master)的末端位置
 * @param angles 关节角度数组
 * @param dh_table 自定义DH参数表
 * @param joint_cnt 关节数量
 * @param EndEffector 输出末端矩阵
 */
void Calculate_FK_Custom(float *angles, const float dh_table[][4], int joint_cnt, Matrix4x4 *EndEffector);

/**
 * @brief 机器人专用正运动学 (使用内置 DH_ROBOT)
 */
void Calculate_FK(float *angles, Matrix4x4 *EndEffector);

/**
 * @brief 逆运动学解算 (带 HumanMimic 稳定性约束)
 * @param TargetPose 目标位姿 (4x4矩阵)
 * @param InitGuess  初始猜测值 (通常是上一帧的角度)
 * @param OutJoints  输出计算出的关节角度
 * @return int 1=收敛成功, 0=达到最大迭代次数(返回当前最优解)
 */
int Calculate_IK(const Matrix4x4 *TargetPose, const float *InitGuess, float *OutJoints);

#endif // KINEMATICS_H