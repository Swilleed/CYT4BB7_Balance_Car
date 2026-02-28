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
 * �����޷����ߺ���
 * @param value ���޷�ֵ
 * @param min_v ����
 * @param max_v ����
 * @return �޷�����ֵ
 * Ϊʲô����д��׷����������Ҫͳһ����ǰ��/ת��ָ�����ָ��ͻ�䵼����̬�����塣
 * ��ôʵ�֣��� value С�����޷������ޣ����������޷������ޣ�����ԭ�����ء�
 * ��ô���ã����ڱ��ļ��ڲ��� Balance_Chase_Position ���á�
 * ��ӦЧ��������׷��������������Լ���ڰ�ȫ��Χ�ڣ��˶���ƽ���ɿء�
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
 * �Ƕȹ�һ���������Ƕ��ƣ�
 * @param angle_deg �����Ƕȣ���λ���ȣ�
 * @return ��һ�����ĽǶȣ���Χ [-180, 180] ��
 * Ϊʲô����д��Ŀ�����뵱ǰ��ֱ���������ܿ�Խ ��180�� �߽磬����ת������������
 * ��ôʵ�֣�ѭ���Ӽ� 360�㣬������ѹ�� [-180, 180] ���䡣
 * ��ô���ã����ڱ��ļ��ڲ��� Balance_Chase_Position ���㺽������ʱ���á�
 * ��ӦЧ����С�����ǰ����̽Ƕȷ���ת�򣬼���ԭ�ش�����ת��
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

// ���ٶȻ�PID����
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

// ��׼�Ƕ�
static float BaseAngle = 1.065;
// �ǶȻ�����
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

// �ٶȻ�����
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

// ���򻷲���
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

// �������˲�������
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

// �������Ʋ���
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
 * ����ƽ��С�����˶�ָ��
 * @param forward_cmd ǰ���ٶ�ָ��
 * @param turn_cmd ת���ٶ�ָ��
 */
void Balance_SetMotionCmd(int32_t forward_cmd, int32_t turn_cmd)
{
    forward_speed = forward_cmd;
    turn_speed = turn_cmd;
}

/**
 * ֱ�����õ����ٶȣ�����Զ�̿��ƣ�
 * @param forward_cmd ǰ���ٶ�ָ��
 * @param turn_cmd ת���ٶ�ָ��
 */
void Balance_Remote_SetSpeed(int32_t forward_cmd, int32_t turn_cmd)
{
    Balance_SetMotionCmd(forward_cmd, turn_cmd);
}

/**
 * ׷��Ŀ��λ�ò�����ƽ�������ٶ�ָ��
 * @param target_x Ŀ���� X ���꣨�� current_pose.x ͬ��λ��
 * @param target_y Ŀ���� Y ���꣨�� current_pose.y ͬ��λ��
 * @param target_yaw_rad Ŀ�꺽���ǣ���λ�����ȣ�
 * Ϊʲô����д���ѡ�·����׷����ͳһ������ balance �㣬���� path/app ���ظ�ƴ�ٶ���ת���߼���
 * ��ôʵ�֣�
 * 1) ���㵱ǰλ�õ�Ŀ�����ľ������
 * 2) ��Ŀ�꺽���ӻ���ת�ɽǶȣ����뵱ǰ Yaw ����һ�����
 * 3) ��������������õ�ǰ��ָ���������������õ�ת��ָ�
 * 4) ��ǰ��/ת��ָ�����޷������� Balance_SetMotionCmd �·���ԭƽ��������·��
 * 5) ����������С����ֵʱ���� 1����ʾ�����㡱��
 * ��ô���ã��� Path_Playback_Tick ���ڵ��ã�ÿ��·���㶼������һ��ֱ�����ص��㡣
 * ��ӦЧ����·������ʱС���������ƽ�Ŀ���㲢�������򣬵������ϲ����е���һ��·���㡣
 */
uint8_t Balance_Chase_Position(float target_x, float target_y, float target_yaw_rad)
{
    float dx = target_x - current_pose.x;
    float dy = target_y - current_pose.y;
    float dist_error = sqrtf(dx * dx + dy * dy);
    float target_yaw_deg = target_yaw_rad * 180.0f / 3.14159f;
    float yaw_error = Balance_NormalizeAngleDeg(target_yaw_deg - Yaw);
    float yaw_abs = fabsf(yaw_error);
    float forward_scale = 1.0f;
    int32_t forward_cmd = 0;
    int32_t turn_cmd = 0;

    if (yaw_abs > 120.0f)
    {
        forward_scale = 0.0f;
    }
    else if (yaw_abs > 90.0f)
    {
        forward_scale = 0.15f;
    }
    else if (yaw_abs > 45.0f)
    {
        forward_scale = 0.45f;
    }

    forward_cmd = (int32_t)Balance_Clampf(dist_error * CHASE_KP_DIST * forward_scale, 0.0f, CHASE_MAX_FORWARD);
    turn_cmd = (int32_t)Balance_Clampf(yaw_error * CHASE_KP_YAW, -CHASE_MAX_TURN, CHASE_MAX_TURN);

    Balance_SetMotionCmd(forward_cmd, turn_cmd);

    if (dist_error <= CHASE_REACH_DIST)
    {
        return 1;
    }

    return 0;
}

/**
 * ƽ��������ѭ��������PID���������µ����ٶ�
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
 * FOC������ѭ�������ݽ��ٶ�PID�������õ����ٶ�
 */
void Balance_Foc_Loop(void)
{
    Motor_SpeedSet(&M1, 1, (_Angle_Speed.Out + 2 * _Dir.Out) * 7.5f);
    Motor_SpeedSet(&M2, 2, (_Angle_Speed.Out - 2 * _Dir.Out) * 7.5f);
}

/**
 * ����С����̬��Ϣ
 * @param roll ��ת��
 * @param yaw ƫ����
 * @param pitch ������
 * �ڲ��������ⲿͨ��IPC����Balance_Attitude_Update_From_Ipc������̬��Ϣ
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
 * IPC�ص���������������M7_1����̬���ݲ�����С����̬
 * @param receive_data ��M7_1���͹��������ݣ�����roll��yaw��pitch��Ϣ
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