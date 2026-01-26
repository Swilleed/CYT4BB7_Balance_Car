#include "zf_common_headfile.h"
#include "infrared_sense.h"
#include "line_following.h"
#include "motor.h"
#include "pid_controller.h"

PID_TypeDef pid_LineFollow;

void Line_Following_PID_Init(float Kp, float Ki, float Kd, float OutputMax, float OutputMin)
{
    PID_Init(&pid_LineFollow, Kp, Ki, Kd, OutputMax, OutputMin);
}

void Tracking_Control(uint8_t status, int32_t baseSpeed)
{
    const float STABLE_ERROR = 0.0f;

    float coefficient1 = 1.0f;
    float coefficient2 = 1.0f;

    float error = 0.0f;

    switch (status)
    {
    case 0x1B:
    case 0x11:
        error = 0.0f;
        break;
    case 0x19:
    case 0x1D:
        error = -0.5f;
        break;
    case 0x18:
    case 0x1C:
        error = -1.0f;
        break;
    case 0x10:
    case 0x14:
        error = -2.0f;
        break;
    case 0x13:
    case 0x17:
        error = 0.5f;
        break;
    case 0x12:
    case 0x16:
        error = 1.0f;
        break;
    case 0x02:
    case 0x06:
        error = 2.0f;
        break;
    default:
        error = STABLE_ERROR;
        break;
    }

    float abs_error = (error > 0) ? error : -error;
    int32_t finalBaseSpeed = baseSpeed;

    if (baseSpeed == 50)
    {
        pid_LineFollow.Kp = (abs_error <= 1.0f) ? 7.0f : 9.3f;
        pid_LineFollow.Kd = (abs_error <= 1.0f) ? 9.0f : 4.0f;
    }
    else if (baseSpeed == 30)
    {
        pid_LineFollow.Kp = (abs_error <= 1.0f) ? 5.0f : 8.0f;
        pid_LineFollow.Kd = (abs_error <= 1.0f) ? 3.0f : 1.8f;
    }
    else if (baseSpeed == 40)
    {
        pid_LineFollow.Kp = (abs_error <= 1.0f) ? 6.0f : 9.0f;
        pid_LineFollow.Kd = (abs_error <= 1.0f) ? 6.0f : 3.8f;
    }

    // if (error < 0)
    // {
    //     if (baseSpeed == 30)
    //     {
    //         coefficient1 = 1.2f;
    //         coefficient2 = 1.2f;
    //     }
    //     else if (baseSpeed == 40)
    //     {
    //         coefficient1 = 1.3f;
    //         coefficient2 = 4.2f;
    //     }
    //     else if (baseSpeed == 50)
    //     {
    //         if (abs_error <= 1.0f)
    //         {
    //             coefficient1 = 1.1f; // 直道：增强系数
    //             coefficient2 = 2.8f;
    //         }
    //         else
    //         {
    //             coefficient1 = 1.5f; // 外侧：满油加速
    //             coefficient2 = 5.3f; // 内侧：暴力反转
    //         }
    //     }
    // }
    // else if (error > 0)
    // {
    //     if (baseSpeed == 30)
    //     {
    //         coefficient1 = 1.2f;
    //         coefficient2 = 1.2f;
    //     }
    //     else if (baseSpeed == 40)
    //     {
    //         coefficient1 = 4.2f;
    //         coefficient2 = 1.3f;
    //     }
    //     else if (baseSpeed == 50)
    //     {
    //         if (abs_error <= 1.0f)
    //         {
    //             coefficient1 = 2.8f;
    //             coefficient2 = 1.1f;
    //         }
    //         else
    //         {
    //             coefficient1 = 5.3f; // 内侧：暴力反转
    //             coefficient2 = 1.5f; // 外侧：满油加速
    //         }
    //     }
    // }
    float correction = PID_Calculate(&pid_LineFollow, 0.0f, error);
    int32_t speed1 = finalBaseSpeed + coefficient1 * (int32_t)correction; // 左轮速度
    int32_t speed2 = finalBaseSpeed - coefficient2 * (int32_t)correction; // 右轮速度
    Motor_SetDuty(&motorLeft, speed1);
    Motor_SetDuty(&motorRight, speed2);
}