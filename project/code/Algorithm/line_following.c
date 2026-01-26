#include "zf_common_headfile.h"
#include "infrared_sense.h"
#include "line_following.h"
#include "motor.h"
#include "pid_controller.h"

PID_TypeDef pid_LineFollow;

void Line_Following_PID_Init(float Kp, float Ki, float Kd, float OutputMax, float OutputMin)
{
    pid_LineFollow.Kp = Kp;
    pid_LineFollow.Ki = Ki;
    pid_LineFollow.Kd = Kd;
    pid_LineFollow.OutputMax = OutputMax;
    pid_LineFollow.OutputMin = OutputMin;
    pid_LineFollow.Integral = 0.0f;
    pid_LineFollow.Err_Last = 0.0f;
    pid_LineFollow.Err_Prev = 0.0f;
}
