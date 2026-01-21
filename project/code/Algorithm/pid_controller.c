#include "pid_controller.h"

/**
 * PID初始化函数
 * @param pid PID结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->Err = 0.0f;
    pid->Err_Last = 0.0f;
    pid->Err_Prev = 0.0f;
    pid->Integral = 0.0f;
    pid->Output = 0.0f;
    pid->OutputMax = 99.0f; // 与 PWM 占空比范围一致
    pid->OutputMin = -99.0f;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

/**
 * PID计算函数
 * @param pid PID结构体指针
 * @param target 目标值
 * @param actual 实际值
 * @return PID输出值
 */
float PID_Calculate(PID_TypeDef *pid, float target, float actual)
{
    pid->TargetValue = target;
    pid->ActualValue = actual;

    // 计算当前误差
    pid->Err = pid->TargetValue - pid->ActualValue;

    // 积分项计算
    // 累加
    pid->Integral += pid->Err;

    // 微分项计算
    float derivative = pid->Err - pid->Err_Last;

    // PID输出计算
    pid->Output = (pid->Kp * pid->Err) + (pid->Ki * pid->Integral) + (pid->Kd * derivative);

    // 输出限幅
    if (pid->Output > pid->OutputMax)
        pid->Output = pid->OutputMax;
    else if (pid->Output < pid->OutputMin)
        pid->Output = pid->OutputMin;

    // 更新误差历史
    pid->Err_Prev = pid->Err_Last;
    pid->Err_Last = pid->Err;

    return pid->Output;
}

/**
 * 平衡车串级pid实现
 * @param outer_pid 外环PID结构体指针（角度控制）
 * @param inner_pid 内环PID结构体指针（角速度控制）
 * @param target_angle 目标角度
 * @param actual_angle 实际角度
 * @param actual_angular_velocity 实际角速度
 * @return 最终PID输出值
 */
float PID_Balance_Calculate(PID_TypeDef *outer_pid, PID_TypeDef *inner_pid,
                            float target_angle, float actual_angle,
                            float actual_angular_velocity)
{
    // 外环PID计算，计算目标角速度
    float target_angular_velocity = PID_Calculate(outer_pid, target_angle, actual_angle);

    // 内环PID计算，计算最终输出
    float output = PID_Calculate(inner_pid, target_angular_velocity, actual_angular_velocity);

    return output;
}
