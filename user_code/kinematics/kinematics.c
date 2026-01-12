/**
 * @file kinematics.c
 * @brief 机械臂运动学实现（正/逆运动学）
 *
 * 说明:
 * - 单位: 长度使用米(m)，角度使用弧度(rad)
 * - 矩阵存储: 行主序(Row-major)，4x4 齐次变换矩阵按 mat->data[16] 存储
 * - 依赖: CMSIS-DSP 的 `arm_mat_*` 矩阵操作（通过 Matrix4x4 封装）
 * - 正运动学使用 D-H 参数表进行连乘，逆运动学使用雅可比转置 + 数值差分近似
 */
#include "kinematics.h"
#include "math.h"
#include "string.h"

/* ================== IK 参数配置 ================== */
/**
 * @name IK 配置常量
 * @note 根据机器人特性可调整: 迭代次数、步长、差分步长与容差
 */
#define IK_MAX_ITERATIONS   20      /**< 最大迭代次数 */
#define IK_TOLERANCE_POS    0.005f  /**< 位置容差 (5mm) */
#define IK_TOLERANCE_ROT    0.05f   /**< 姿态容差 (~3度) */
#define IK_STEP_SIZE        0.20f   /**< 学习率/步长 (过大容易震荡) */
#define DELTA_THETA         0.001f  /**< 差分步长，用于数值雅可比估计 */

/* 多目标优化权重 (参考 HumanMimic Eq.2) */
#define WEIGHT_POS          1.0f    /**< 位置追踪权重 */
#define WEIGHT_ROT          0.8f    /**< 姿态追踪权重 */
#define WEIGHT_DISP         0.15f   /**< 最小位移阻尼权重 (稳定性) */

/* ================== 机械臂 D-H 参数表 (单位: m) ================== */
/* 注意：请确保与 URDF 严格一致 */
const float DH_ROBOT[JOINT_COUNT][4] = {
    // theta_off,    d,       a,       alpha
    // Joint 1: Base -> Link1 (z-rot)
    {0.0f,          0.058f,  0.0f,    1.5708f},  // d ≈ Base height
    
    // Joint 2: Shoulder Pitch (Link1 -> Link2)
    {-1.5708f,      0.0f,    0.152f,  0.0f},     // a = 0.152 (Link1->Link2 offset)
    
    // Joint 3: Shoulder Roll (Link2 -> Link3)
    {0.0f,          0.0f,    0.0f,    1.5708f}, 
    
    // Joint 4: Elbow Pitch (Link3 -> Link4)
    {0.0f,          0.200f,  0.0f,    1.5708f},  // d = 0.2 (Upper Arm Length)
    
    // Joint 5: Forearm Roll (Link4 -> Link5)
    {0.0f,          0.0f,    0.1635f, 0.0f},     // a = 0.1635 (Forearm Part 1)
    
    // Joint 6: Wrist Pitch (Link5 -> Link6)
    {-1.5708f,      0.194f,  0.0f,    1.5708f},  // d = 0.194 (Forearm Part 2)
    
    // Joint 7: Wrist Roll (Link6 -> End)
    {0.0f,          0.033f,  0.0f,    0.0f}      // d ≈ End Effector offset
};

/* ================== 辅助函数 ================== */

/**
 * @brief 初始化 4x4 矩阵的 CMSIS 实例
 * @param mat 目标 Matrix4x4
 */
void Mat4_Init(Matrix4x4 *mat) {
    arm_mat_init_f32(&mat->instance, 4, 4, mat->data);
}

/**
 * @brief 将矩阵置为单位矩阵
 * @param mat 目标 Matrix4x4
 */
void Mat4_SetIdentity(Matrix4x4 *mat) {
    memset(mat->data, 0, sizeof(mat->data));
    mat->data[0] = 1.0f; mat->data[5] = 1.0f; mat->data[10] = 1.0f; mat->data[15] = 1.0f;
}

/**
 * @brief 矩阵乘法：out = A * B
 * @param out 输出矩阵
 * @param A 左矩阵
 * @param B 右矩阵
 */
void Mat4_Multiply(Matrix4x4 *out, Matrix4x4 *A, Matrix4x4 *B) {
    arm_mat_mult_f32(&A->instance, &B->instance, &out->instance);
}

/**
 * @brief 计算 4x4 矩阵的逆
 * @param out 输出逆矩阵
 * @param in 输入矩阵
 * @return 0 成功, -1 失败
 */
int Mat4_Inverse(Matrix4x4 *out, Matrix4x4 *in) {
    return (arm_mat_inverse_f32(&in->instance, &out->instance) == ARM_MATH_SUCCESS) ? 0 : -1;
}

/**
 * @brief 根据 D-H 参数生成单个关节的齐次变换矩阵
 * @param mat 输出矩阵
 * @param theta 关节角 (rad)
 * @param d 连杆偏距 (m)
 * @param a 连杆长度 (m)
 * @param alpha 扭转角 (rad)
 */
void Mat4_FromDH(Matrix4x4 *mat, float theta, float d, float a, float alpha) {
    float ct = cosf(theta); float st = sinf(theta);
    float ca = cosf(alpha); float sa = sinf(alpha);
    float *p = mat->data;
    p[0] = ct;    p[1] = -st*ca; p[2] = st*sa;   p[3] = a*ct;
    p[4] = st;    p[5] = ct*ca;  p[6] = -ct*sa;  p[7] = a*st;
    p[8] = 0.0f;  p[9] = sa;     p[10]= ca;      p[11]= d;
    p[12]= 0.0f;  p[13]= 0.0f;   p[14]= 0.0f;    p[15]= 1.0f;
}

static void Get_Position(const Matrix4x4 *T, float *pos) {
    pos[0] = T->data[3]; pos[1] = T->data[7]; pos[2] = T->data[11];
}

static void Vec3_Sub(float *out, float *a, float *b) {
    out[0] = a[0] - b[0]; out[1] = a[1] - b[1]; out[2] = a[2] - b[2];
}

/* 计算姿态误差 (轴向量叉积法近似) */
static void Calc_Orientation_Error(const Matrix4x4 *Curr, const Matrix4x4 *Targ, float *err) {
    float c_ax[3] = {Curr->data[0], Curr->data[4], Curr->data[8]};
    float c_ay[3] = {Curr->data[1], Curr->data[5], Curr->data[9]};
    float c_az[3] = {Curr->data[2], Curr->data[6], Curr->data[10]};
    float t_ax[3] = {Targ->data[0], Targ->data[4], Targ->data[8]};
    float t_ay[3] = {Targ->data[1], Targ->data[5], Targ->data[9]};
    float t_az[3] = {Targ->data[2], Targ->data[6], Targ->data[10]};

    // Error = 0.5 * sum(Current_axis X Target_axis)
    err[0] = 0.5f * ((c_ax[1]*t_ax[2] - c_ax[2]*t_ax[1]) + (c_ay[1]*t_ay[2] - c_ay[2]*t_ay[1]) + (c_az[1]*t_az[2] - c_az[2]*t_az[1]));
    err[1] = 0.5f * ((c_ax[2]*t_ax[0] - c_ax[0]*t_ax[2]) + (c_ay[2]*t_ay[0] - c_ay[0]*t_ay[2]) + (c_az[2]*t_az[0] - c_az[0]*t_az[2]));
    err[2] = 0.5f * ((c_ax[0]*t_ax[1] - c_ax[1]*t_ax[0]) + (c_ay[0]*t_ay[1] - c_ay[1]*t_ay[0]) + (c_az[0]*t_az[1] - c_az[1]*t_az[0]));
}

/* ================== 核心功能实现 ================== */

/**
 * @brief 通用正运动学（支持自定义 D-H 表）
 * @param angles 关节角数组，长度为 `joint_cnt`，单位为弧度
 * @param dh_table D-H 参数表，格式为 [theta_off, d, a, alpha]
 * @param joint_cnt 关节数量
 * @param EndEffector 输出末端位姿矩阵（4x4）
 * @details 按顺序计算 T_total = T_1 * T_2 * ... * T_n，其中每个 T_i 由 `Mat4_FromDH` 生成
 */
void Calculate_FK_Custom(float *angles, const float dh_table[][4], int joint_cnt, Matrix4x4 *EndEffector) {
    Matrix4x4 T_total, T_local, T_next;
    Mat4_Init(&T_total); Mat4_Init(&T_local); Mat4_Init(&T_next); Mat4_Init(EndEffector);
    Mat4_SetIdentity(&T_total);

    for (int i = 0; i < joint_cnt; i++) {
        Mat4_FromDH(&T_local, angles[i] + dh_table[i][0], dh_table[i][1], dh_table[i][2], dh_table[i][3]);
        Mat4_Multiply(&T_next, &T_total, &T_local);
        memcpy(T_total.data, T_next.data, sizeof(float) * 16);
    }
    memcpy(EndEffector->data, T_total.data, sizeof(float) * 16);
}

/**
 * @brief 机器人正运动学（使用内部定义的 `DH_ROBOT` 表）
 * @param angles 输入关节角数组，长度为 `JOINT_COUNT`（弧度）
 * @param EndEffector 输出末端位姿矩阵（4x4）
 */
void Calculate_FK(float *angles, Matrix4x4 *EndEffector) {
    Calculate_FK_Custom(angles, DH_ROBOT, JOINT_COUNT, EndEffector);
}

/**
 * @brief 优化后的逆运动学 (HumanMimic C1+C2+C3 多目标优化)
 * 核心优化：在梯度下降中加入了阻尼项 (damping_grad)，使解偏向于上一帧的角度
 */
int Calculate_IK(const Matrix4x4 *TargetPose, const float *InitGuess, float *OutJoints) {
    float current_joints[JOINT_COUNT];
    float temp_joints[JOINT_COUNT];
    Matrix4x4 current_pose, perturbed_pose;
    float error[6]; // [dx, dy, dz, drx, dry, drz]
    float J_col[6]; // 雅可比列向量

    // 初始化: 继承上一帧角度 (InitGuess)
    memcpy(current_joints, InitGuess, sizeof(float) * JOINT_COUNT);

    for (int iter = 0; iter < IK_MAX_ITERATIONS; iter++) {
        // A. 正解计算当前位姿
        Calculate_FK(current_joints, &current_pose);

        // B. 计算任务空间误差 (Task Error)
        float curr_pos[3], targ_pos[3];
        Get_Position(&current_pose, curr_pos);
        Get_Position(TargetPose, targ_pos);
        Vec3_Sub(&error[0], targ_pos, curr_pos); // 位置误差
        Calc_Orientation_Error(&current_pose, TargetPose, &error[3]); // 姿态误差

        // C. 收敛检查
        float pos_err = sqrtf(error[0]*error[0] + error[1]*error[1] + error[2]*error[2]);
        float rot_err = sqrtf(error[3]*error[3] + error[4]*error[4] + error[5]*error[5]);
        if (pos_err < IK_TOLERANCE_POS && rot_err < IK_TOLERANCE_ROT) {
            memcpy(OutJoints, current_joints, sizeof(float) * JOINT_COUNT);
            return 1; // 收敛成功
        }

        // D. 雅可比转置迭代更新
        for (int j = 0; j < JOINT_COUNT; j++) {
            // 1. 差分法求雅可比列向量 (J_col)
            memcpy(temp_joints, current_joints, sizeof(float) * JOINT_COUNT);
            temp_joints[j] += DELTA_THETA;
            Calculate_FK(temp_joints, &perturbed_pose);
            
            float p_new[3];
            Get_Position(&perturbed_pose, p_new);
            
            // J_pos (位置变化率)
            J_col[0] = (p_new[0] - curr_pos[0]) / DELTA_THETA;
            J_col[1] = (p_new[1] - curr_pos[1]) / DELTA_THETA;
            J_col[2] = (p_new[2] - curr_pos[2]) / DELTA_THETA;
            
            // J_rot (姿态变化率)
            float rot_err_p[3];
            Calc_Orientation_Error(&current_pose, &perturbed_pose, rot_err_p);
            J_col[3] = rot_err_p[0] / DELTA_THETA;
            J_col[4] = rot_err_p[1] / DELTA_THETA;
            J_col[5] = rot_err_p[2] / DELTA_THETA;

            // 2. 任务梯度 (Task Gradient) = Weighted J^T * Error
            float task_grad = 
                WEIGHT_POS * (J_col[0]*error[0] + J_col[1]*error[1] + J_col[2]*error[2]) +
                WEIGHT_ROT * (J_col[3]*error[3] + J_col[4]*error[4] + J_col[5]*error[5]);

            // 3. 【核心优化】阻尼梯度 (Damping Gradient) - HumanMimic C3
            // 目标: min || theta_curr - theta_prev ||^2
            // 导数: 2 * (theta_curr - theta_prev)
            // 负梯度方向更新: -k * (curr - prev)
            float damping_grad = -WEIGHT_DISP * (current_joints[j] - InitGuess[j]);

            // 4. 更新关节
            current_joints[j] += IK_STEP_SIZE * (task_grad + damping_grad);
        }
    }

    memcpy(OutJoints, current_joints, sizeof(float) * JOINT_COUNT);
    return 0; // 达到最大迭代次数 (返回当前最佳解)
}