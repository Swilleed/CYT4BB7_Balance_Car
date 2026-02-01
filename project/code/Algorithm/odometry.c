#include "odometry.h"
#include <math.h>

#ifndef PI
#define PI 3.1415926535f
#endif

Pose_t current_pose = {0.0f, 0.0f, 0.0f};

/**
 * @brief 初始化里程计
 */
void Odometry_Init(void)
{
    Odometry_Reset();
}

/**
 * @brief 重置坐标零点
 */
void Odometry_Reset(void)
{
    current_pose.x = 0.0f;
    current_pose.y = 0.0f;
    current_pose.yaw = 0.0f;
}

/**
 * @brief 更新里程计 (在控制循环中调用)
 * @param left_diff  左轮位移增量
 * @param right_diff 右轮位移增量
 * @param yaw_deg    当前航向角 (角度制)
 */
void Odometry_Update(int16 left_diff, int16 right_diff, float yaw_deg)
{
    float delta_s = (float)(left_diff + right_diff) / 2.0f;

    // 将角度转为弧度
    current_pose.yaw = yaw_deg * PI / 180.0f;

    current_pose.x += delta_s * cosf(current_pose.yaw);
    current_pose.y += delta_s * sinf(current_pose.yaw);
}
