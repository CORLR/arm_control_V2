#include "kinematics.h"
#include "math.h"
#include "string.h" // 需要 memcpy 和 memset

/* ================== D-H 参数配置 ================== */

/**
 * @brief 机械臂 D-H 参数表
 * * 对应关系:
 * Col 0: theta_offset (关节角偏移量, rad)
 * Col 1: d (连杆偏距, m)
 * Col 2: a (连杆长度, m)
 * Col 3: alpha (连杆扭转角, rad)
 * * !!! 重要提示 !!!
 * 这些值必须与你在 MATLAB 中仿真成功的模型参数完全一致。
 * 如果 MATLAB 使用 importrobot 从 URDF 导入，你可以通过 robot.Bodies{i}.Joint.HomePosition 等属性查看，
 * 或者参考标准的 DH 建模规则将 URDF 转换过来。
 * 下面的值为示例值（基于常见的7轴构型），请务必根据实际情况修改！
 */
const float DH_PARAMS[JOINT_COUNT][4] = {
    // theta_off,              d,           a,              alpha
    {0.0f,          0.085f,     0.0f,       1.5708f},   // Joint 1 (Base Yaw)
    {-3.1416f,     -0.1158f,    0.0f,       1.5708f},   // Joint 2 (Shoulder Pitch) - URDF z=-0.1158
    {-1.5708f,      0.0f,       0.22154f,  -1.5708f},   // Joint 3 (Shoulder Roll) - Link2 Length = 0.22154
    {0.0f,         -0.050f,     0.0f,       1.5708f},   // Joint 4 (Elbow Pitch) - URDF z=-0.05
    {1.5708f,       0.0f,       0.2285f,    1.5708f},   // Joint 5 (Forearm Roll) - Link4 Length = 0.2285
    {0.0f,         -0.0505f,    0.0f,      -1.5708f},   // Joint 6 (Wrist Pitch) - URDF z=-0.0505
    {-3.1416f,      0.0f,       0.0f,       0.0f}       // Joint 7 (Wrist Roll)
};

/* ================== 辅助函数实现 ================== */

void Mat4_Init(Matrix4x4 *mat) {
    // 告诉 CMSIS 这是一个 4行 4列 的矩阵，数据存在 mat->data 中
    arm_mat_init_f32(&mat->instance, 4, 4, mat->data);
}

void Mat4_Multiply(Matrix4x4 *out, Matrix4x4 *A, Matrix4x4 *B) {
    // 这里的 instance 已经在 Mat4_Init 中初始化好了
    arm_mat_mult_f32(&A->instance, &B->instance, &out->instance);
}

int Mat4_Inverse(Matrix4x4 *out, Matrix4x4 *in) {
    arm_status status;
    status = arm_mat_inverse_f32(&in->instance, &out->instance);
    
    if (status == ARM_MATH_SUCCESS) {
        return 0;
    } else {
        return -1; // 矩阵奇异，不可逆
    }
}

void Mat4_FromDH(Matrix4x4 *mat, float theta, float d, float a, float alpha) {
    float ct = cosf(theta);
    float st = sinf(theta);
    float ca = cosf(alpha);
    float sa = sinf(alpha);

    // CMSIS-DSP 是行主序 (Row-major)
    float *p = mat->data;

    // Row 1
    p[0] = ct;     p[1] = -st * ca;   p[2] = st * sa;    p[3] = a * ct;
    // Row 2
    p[4] = st;     p[5] = ct * ca;    p[6] = -ct * sa;   p[7] = a * st;
    // Row 3
    p[8] = 0.0f;   p[9] = sa;         p[10] = ca;        p[11] = d;
    // Row 4
    p[12] = 0.0f;  p[13] = 0.0f;      p[14] = 0.0f;      p[15] = 1.0f;
}

/* ================== 核心功能实现 ================== */

/**
 * @brief 将矩阵设为单位矩阵
 */
static void Mat4_SetIdentity(Matrix4x4 *mat) {
    memset(mat->data, 0, sizeof(mat->data));
    mat->data[0] = 1.0f;
    mat->data[5] = 1.0f;
    mat->data[10] = 1.0f;
    mat->data[15] = 1.0f;
}

/**
 * @brief 正运动学解算 (FK) 实现
 * * 计算公式: T_total = T_1 * T_2 * ... * T_7
 */
void Calculate_FK(float *angles, Matrix4x4 *EndEffector) {
    Matrix4x4 T_total; // 累积变换矩阵 (最终结果)
    Matrix4x4 T_local; // 当前关节的局部变换矩阵
    Matrix4x4 T_next;  // 临时存储乘法结果 (Ping-Pong buffer)

    // 1. 初始化所有矩阵结构体
    Mat4_Init(&T_total);
    Mat4_Init(&T_local);
    Mat4_Init(&T_next);
    Mat4_Init(EndEffector); // 确保输出矩阵也被初始化

    // 2. 将 T_total 初始化为单位矩阵
    Mat4_SetIdentity(&T_total);

    // 3. 循环计算连乘: T_total = T_total * T_local
    for (int i = 0; i < JOINT_COUNT; i++) {
        // 根据 DH 参数和当前关节角度生成局部变换矩阵
        // theta = 当前角度 + 零位偏移
        Mat4_FromDH(&T_local, 
                    angles[i] + DH_PARAMS[i][0], // theta
                    DH_PARAMS[i][1],             // d
                    DH_PARAMS[i][2],             // a
                    DH_PARAMS[i][3]);            // alpha

        // 执行矩阵乘法: T_next = T_total * T_local
        // 注意: arm_mat_mult_f32 的输入和输出不建议重叠
        Mat4_Multiply(&T_next, &T_total, &T_local);

        // 将结果复制回 T_total，为下一次乘法做准备
        memcpy(T_total.data, T_next.data, sizeof(float) * 16);
    }

    // 4. 输出最终结果
    memcpy(EndEffector->data, T_total.data, sizeof(float) * 16);
}