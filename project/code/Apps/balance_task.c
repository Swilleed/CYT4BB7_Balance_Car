#include "zf_common_headfile.h"
#include "PID.h"
#include "balance_task.h"
#include "KF.h"
#include "motor.h"

float Roll, Yaw, Pitch, X_gyro = 0;
static float Last_Roll = 0;

static PID_TypeDef _Angle_Speed = {

    .error0 = 0,
    .error1 = 0,
    .error2 = 0,

    .Kp = 0.3,
    .Ki = 0,
    .Kd = 4,

    .OutMax = 14,
    .OutMin = -14,

    .Target = 0,
    .Out = 0,

};

static float BaseAngle = 1.065;
static PID_TypeDef _Angle = {

    .error0 = 0,
    .error1 = 0,
    .error2 = 0,

    .Kp = 0.9,
    .Ki = 0,
    .Kd = 6,

    .OutMax = 25,
    .OutMin = -25,

    .Target = 0,
    .Out = -0,

};

static float Average_Speed, Delta_Speed, L_Speed, R_Speed;
static PID_TypeDef _Speed = {

    .error0 = 0,
    .error1 = 0,
    .error2 = 0,

    .Kp = 1.2,
    .Ki = -0.01,
    .Kd = 3.0,

    .OutMax = 6,
    .OutMin = -6,

    .Target = 0,
    .Out = 0,

};

static PID_TypeDef _Dir = {

    .error0 = 0,
    .error1 = 0,
    .error2 = 0,

    .Kp = 0.10,
    .Ki = 0.00,
    .Kd = 5,

    .OutMax = 0.8,
    .OutMin = -0.8,

    .Target = 0,
    .Out = 0,

};

static KalmanFilter1D_Speed X_GYRO = {
    .Speed_Hat = 0.0f,
    .P = 1.0f,
    .Q = 0.0005f,
    .R = 0.5f};
static KalmanFilter1D_Speed FORWARD = {
    .Speed_Hat = 0.0f,
    .P = 1.0f,
    .Q = 0.0005f,
    .R = 0.5f};
static KalmanFilter1D_Speed TURN = {
    .Speed_Hat = 0.0f,
    .P = 1.0f,
    .Q = 0.0005f,
    .R = 0.5f};

static int32_t forward_speed = 0, turn_speed = 0;
static PID_TypeDef M1 = {

    .error0 = 0,
    .error1 = 0,
    .error2 = 0,

    .Kp = 20,
    .Ki = 2,
    .Kd = 0,

    .OutMax = 5,
    .OutMin = -5,

    .Target = 0,
    .Out = 0,

};
static PID_TypeDef M2 = {

    .error0 = 0,
    .error1 = 0,
    .error2 = 0,

    .Kp = 20,
    .Ki = 2,
    .Kd = 0,

    .OutMax = 5,
    .OutMin = -5,

    .Target = 0,
    .Out = 0,

};

void Balance_SetMotionCmd(int32_t forward_cmd, int32_t turn_cmd)
{
    forward_speed = forward_cmd;
    turn_speed = turn_cmd;
}

void Balance_Remote_SetSpeed(int32_t forward_cmd, int32_t turn_cmd)
{
    Balance_SetMotionCmd(forward_cmd, turn_cmd);
}

void Balance_Control_Loop(void)
{
    KalmanFilter1D_Speed_Update(&FORWARD, (forward_speed / 15.0));
    KalmanFilter1D_Speed_Update(&TURN, (turn_speed / 50.0));
    _Speed.Target = FORWARD.Speed_Hat;
    _Dir.Target = TURN.Speed_Hat;

    printf("%.2f,%.2f\r\n", _Speed.Target, _Dir.Target);
    IMUcounter = 0;
    _Speed.Actual = (L_Speed + R_Speed) / 2;
    PID_Update_Pos(&_Speed);

    _Dir.Actual = L_Speed - R_Speed;
    _Dir.Actual = _Dir.Actual < (_Dir.Target - 1) ? _Dir.Actual : (_Dir.Actual > (_Dir.Target + 1) ? _Dir.Actual : _Dir.Target);
    PID_Update_Pos(&_Dir);

    _Angle.Target = BaseAngle - _Speed.Out;
    _Angle.Actual = Roll;
    PID_Update_Pos(&_Angle);

    _Angle_Speed.Target = 0.1 * _Angle.Out + 0.9 * _Angle_Speed.Target;
    _Angle_Speed.Actual = X_gyro;
    PID_Update_Pos(&_Angle_Speed);
}

void Balance_Foc_Loop(void)
{
    Motor_SpeedSet(&M1, 1, (_Angle_Speed.Out + 2 * _Dir.Out) * 7.5f);
    Motor_SpeedSet(&M2, 2, (_Angle_Speed.Out - 2 * _Dir.Out) * 7.5f);
}

static void Balance_Attitude_Update(float roll, float yaw, float pitch)
{
    Last_Roll = Roll;
    Roll = roll;
    Yaw = yaw;
    Pitch = pitch;

    KalmanFilter1D_Speed_Update(&X_GYRO, (Roll - Last_Roll) / 0.01f);
    X_gyro = X_GYRO.Speed_Hat > 25.0f ? 25.0f : (X_GYRO.Speed_Hat < -25.0f ? -25.0f : X_GYRO.Speed_Hat);
}

void Balance_Attitude_Update_From_Ipc(uint32 receive_data)
{
    static uint8_t frame_idx = 0;
    static float roll = 0.0f;
    static float yaw = 0.0f;
    static float pitch = 0.0f;
    float angle = ((float)receive_data / 100.0f) - 180.0f;

    if (frame_idx == 0)
    {
        roll = angle;
    }
    else if (frame_idx == 1)
    {
        yaw = angle;
    }
    else
    {
        pitch = angle;
        Balance_Attitude_Update(roll, yaw, pitch);
    }

    frame_idx++;
    if (frame_idx == 3)
    {
        frame_idx = 0;
    }
}