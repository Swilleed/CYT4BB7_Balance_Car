#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

// PID参数结构体
typedef struct
{
    float TargetValue; // 目标值
    float ActualValue; // 实际值
    float Err;         // 当前误差
    float Err_Last;    // 上次误差
    float Err_Prev;    // 上上次误差
    float Kp, Ki, Kd;  // PID参数
    float Integral;    // 积分值
    float Output;      // 输出值
    float OutputMax;   // 输出限幅
    float OutputMin;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float output_min, float output_max);
float PID_Calculate(PID_TypeDef *pid, float target, float actual);

#endif // __PID_CONTROLLER_H__
