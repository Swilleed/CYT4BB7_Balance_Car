#include "zf_common_typedef.h"
#include "Madgwick.h"
#include <math.h>

float MADGWICK_BETA = 0.05;

void Madgwick_Init()
{
	
	
}

void Madgwick_Update(MadgwickFilter *mf, float gyro[], float accel[], float dt) 
{

    // 局部变量（减少重复计算，提升效率）
    float q0 = mf->q0, q1 = mf->q1, q2 = mf->q2, q3 = mf->q3;
    float norm, inv_norm;
    float s0, s1, s2, s3;       // 梯度下降误差项
    float qDot1, qDot2, qDot3, qDot4; // 四元数导数
    
    // 辅助变量（减少乘法运算）
    float _2q0, _2q1, _2q2, _2q3, _4q1, _4q2;
    float q1q1, q2q2;

    // ---------------- 步骤1：加速度计数据归一化（消除幅值影响） ----------------
    norm = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (norm < 1e-6) return; // 避免除零（加速度计数据无效时直接返回）
    inv_norm = 1.0f / norm;
    accel[0] *= inv_norm;
    accel[1] *= inv_norm;
    accel[2] *= inv_norm;

    // ---------------- 步骤2：计算辅助变量（减少重复乘法） ----------------
    _2q0 = 2.0 * q0;
    _2q1 = 2.0 * q1;
    _2q2 = 2.0 * q2;
    _2q3 = 2.0 * q3;
    
    _4q1 = 4.0 * q1;
    _4q2 = 4.0 * q2;
    
    
    
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    

    // ---------------- 步骤3：梯度下降算法计算误差项s0-s3 ----------------
    // 参考系（地球坐标系）与载体坐标系的投影误差
   
    s0 = -_2q2 * (2.0f*q1*q3 - _2q0*q2 - accel[0]) + _2q1 * (2.0f*q0*q1 + _2q2*q3 - accel[1]) - _2q3 * (2.0f*(0.5f - q1q1 - q2q2) - accel[2]);
    s1 = _2q3 * (2.0f*q1*q3 - _2q0*q2 - accel[0]) + _2q0 * (2.0f*q0*q1 + _2q2*q3 - accel[1]) - _4q1 * (2.0f*(0.5f - q1q1 - q2q2) - accel[2]);
    s2 = -_2q0 * (2.0f*q1*q3 - _2q0*q2 - accel[0]) + _2q3 * (2.0f*q0*q1 + _2q2*q3 - accel[1]) - _4q2 * (2.0f*(0.5f - q1q1 - q2q2) - accel[2]);
    s3 = _2q1 * (2.0f*q1*q3 - _2q0*q2 - accel[0]) + _2q2 * (2.0f*q0*q1 + _2q2*q3 - accel[1]);

    // ---------------- 步骤4：误差项归一化（保证梯度下降方向正确） ----------------
    norm = sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (norm < 1e-6) inv_norm = 0.0f; // 无误差时无需修正
    else inv_norm = 1.0f / norm;
    s0 *= inv_norm;
    s1 *= inv_norm;
    s2 *= inv_norm;
    s3 *= inv_norm;

    // ---------------- 步骤5：计算四元数导数（陀螺仪积分 + 梯度修正） ----------------
    // 陀螺仪积分项（无误差时的四元数变化）
    qDot1 = 0.5f * (-q1*gyro[0] - q2*gyro[1] - q3*gyro[2]);
    qDot2 = 0.5f * (q0*gyro[0] + q2*gyro[2] - q3*gyro[1]);
    qDot3 = 0.5f * (q0*gyro[1] - q1*gyro[2] + q3*gyro[0]);
    qDot4 = 0.5f * (q0*gyro[2] + q1*gyro[1] - q2*gyro[0]);
    // 梯度下降修正项（用加速度计误差修正陀螺仪漂移）
    qDot1 -= MADGWICK_BETA * s0;
    qDot2 -= MADGWICK_BETA * s1;
    qDot3 -= MADGWICK_BETA * s2;
    qDot4 -= MADGWICK_BETA * s3;

    // ---------------- 步骤6：积分更新四元数 ----------------
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // ---------------- 步骤7：四元数归一化（避免数值漂移） ----------------
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm < 1e-6) norm = 1.0f; // 防止除零
    inv_norm = 1.0f / norm;
    mf->q0 = q0 * inv_norm;
    mf->q1 = q1 * inv_norm;
    mf->q2 = q2 * inv_norm;
    mf->q3 = q3 * inv_norm;
}
float Madgwick_QuatToPitch(MadgwickFilter *mf) {

    return atan2f(2.0f*(mf->q0*mf->q1 + mf->q2*mf->q3), 1.0f - 2.0f*(mf->q1*mf->q1 + mf->q2*mf->q2));
}


float Madgwick_QuatToRoll(MadgwickFilter *mf) {
    
    return asinf(2.0f*(mf->q0*mf->q2 - mf->q1*mf->q3));
}


float Madgwick_QuatToYaw(MadgwickFilter *mf) {
    
    return atan2f(2.0f*(mf->q1*mf->q2 + mf->q0*mf->q3), mf->q0*mf->q0 + mf->q1*mf->q1 - mf->q2*mf->q2 - mf->q3*mf->q3);
}

