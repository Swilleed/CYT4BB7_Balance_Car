#include "zf_common_headfile.h"
#include "PID.h"
#include "balance_task.h"
#include "KF.h"
#include "motor.h"
#include "odometry.h"
#include <math.h>

#define CHASE_KP_DIST (1.5f)
#define CHASE_KP_YAW (1.0f)
#define CHASE_MAX_FORWARD (300.0f)
#define CHASE_MAX_TURN (120.0f)
#define CHASE_REACH_DIST (8.0f)

/**
 * 浮点限幅工具函数
 * @param value 待限幅值
 * @param min_v 下限
 * @param max_v 上限
 * @return 限幅后的值
 * 为什么这样写：追逐控制中需要统一限制前进/转向指令，避免指令突变导致姿态环过冲。
 * 怎么实现：若 value 小于下限返回下限，若大于上限返回上限，否则原样返回。
 * 怎么调用：仅在本文件内部由 Balance_Chase_Position 调用。
 * 对应效果：所有追逐控制输出都被约束在安全范围内，运动更平滑可控。
 */
static float Balance_Clampf(float value, float min_v, float max_v)
{
    if (value < min_v)
    {
        return min_v;
    }
    if (value > max_v)
    {
        return max_v;
    }
    return value;
}

/**
 * 角度归一化函数（角度制）
 * @param angle_deg 输入角度（单位：度）
 * @return 归一化后的角度，范围 [-180, 180] 度
 * 为什么这样写：目标角与当前角直接相减可能跨越 ±180° 边界，导致转向方向错误。
 * 怎么实现：循环加减 360°，将结果压到 [-180, 180] 区间。
 * 怎么调用：仅在本文件内部由 Balance_Chase_Position 计算航向误差时调用。
 * 对应效果：小车总是按最短角度方向转向，减少原地大幅绕转。
 */
static float Balance_NormalizeAngleDeg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float Roll, Yaw, Pitch, X_gyro = 0;
static float Last_Roll = 0;

// 角速度环PID参数
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

// 基准角度
static float BaseAngle = 1.065;
// 角度环参数
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

// 速度环参数
extern float L_Speed, R_Speed;
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

// 方向环参数
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

// 卡尔曼滤波器参数
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

// 电机控制参数
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

/**
 * 设置平衡小车的运动指令
 * @param forward_cmd 前进速度指令
 * @param turn_cmd 转向速度指令
 */
void Balance_SetMotionCmd(int32_t forward_cmd, int32_t turn_cmd)
{
    forward_speed = forward_cmd;
    turn_speed = turn_cmd;
}

/**
 * 直接设置电机速度（用于远程控制）
 * @param forward_cmd 前进速度指令
 * @param turn_cmd 转向速度指令
 */
void Balance_Remote_SetSpeed(int32_t forward_cmd, int32_t turn_cmd)
{
    Balance_SetMotionCmd(forward_cmd, turn_cmd);
}

/**
 * 追逐目标位置并输出平衡控制速度指令
 * @param target_x 目标点 X 坐标（与 current_pose.x 同单位）
 * @param target_y 目标点 Y 坐标（与 current_pose.y 同单位）
 * @param target_yaw_rad 目标航向角（单位：弧度）
 * 为什么这样写：把“路径点追逐”统一收敛在 balance 层，避免 path/app 层重复拼速度与转向逻辑。
 * 怎么实现：
 * 1) 计算当前位置到目标点的距离误差；
 * 2) 将目标航向从弧度转成角度，并与当前 Yaw 做归一化误差；
 * 3) 距离误差经比例环得到前进指令，航向误差经比例环得到转向指令；
 * 4) 对前进/转向指令做限幅后调用 Balance_SetMotionCmd 下发到原平衡控制链路；
 * 5) 当距离误差小于阈值时返回 1，表示“到点”。
 * 怎么调用：由 Path_Playback_Tick 周期调用，每个路径点都会调用一次直到返回到点。
 * 对应效果：路径复现时小车会持续逼近目标点并修正航向，到点后上层再切到下一个路径点。
 */
uint8_t Balance_Chase_Position(float target_x, float target_y, float target_yaw_rad)
{
    float dx = target_x - current_pose.x;
    float dy = target_y - current_pose.y;
    float dist_error = sqrtf(dx * dx + dy * dy);
    float target_yaw_deg = target_yaw_rad * 180.0f / 3.14159f;
    float yaw_error = Balance_NormalizeAngleDeg(target_yaw_deg - Yaw);
    int32_t forward_cmd = 0;
    int32_t turn_cmd = 0;

    forward_cmd = (int32_t)Balance_Clampf(dist_error * CHASE_KP_DIST, 0.0f, CHASE_MAX_FORWARD);
    turn_cmd = (int32_t)Balance_Clampf(yaw_error * CHASE_KP_YAW, -CHASE_MAX_TURN, CHASE_MAX_TURN);

    Balance_SetMotionCmd(forward_cmd, turn_cmd);

    if (dist_error <= CHASE_REACH_DIST)
    {
        return 1;
    }

    return 0;
}

/**
 * 平衡控制主循环，计算PID输出并更新电机速度
 */
void Balance_Control_Loop(void)
{
    KalmanFilter1D_Speed_Update(&FORWARD, (forward_speed / 15.0));
    KalmanFilter1D_Speed_Update(&TURN, (turn_speed / 50.0));
    _Speed.Target = FORWARD.Speed_Hat;
    _Dir.Target = TURN.Speed_Hat;

    printf("%.2f,%.2f\r\n", _Speed.Target, _Dir.Target);
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

/**
 * FOC控制主循环，根据角速度PID输出设置电机速度
 */
void Balance_Foc_Loop(void)
{
    Motor_SpeedSet(&M1, 1, (_Angle_Speed.Out + 2 * _Dir.Out) * 7.5f);
    Motor_SpeedSet(&M2, 2, (_Angle_Speed.Out - 2 * _Dir.Out) * 7.5f);
}

/**
 * 更新小车姿态信息
 * @param roll 滚转角
 * @param yaw 偏航角
 * @param pitch 俯仰角
 * 内部函数，外部通过IPC调用Balance_Attitude_Update_From_Ipc更新姿态信息
 */
static void Balance_Attitude_Update(float roll, float yaw, float pitch)
{
    Last_Roll = Roll;
    Roll = roll;
    Yaw = yaw;
    Pitch = pitch;

    KalmanFilter1D_Speed_Update(&X_GYRO, (Roll - Last_Roll) / 0.01f);
    X_gyro = X_GYRO.Speed_Hat > 25.0f ? 25.0f : (X_GYRO.Speed_Hat < -25.0f ? -25.0f : X_GYRO.Speed_Hat);
}

/**
 * IPC回调函数，接收来自M7_1的姿态数据并更新小车姿态
 * @param receive_data 从M7_1发送过来的数据，包含roll、yaw、pitch信息
 */
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