#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "zf_common_typedef.h"

typedef struct
{
    float x;   // X 坐标 (单位：脉冲)
    float y;   // Y 坐标 (单位：脉冲)
    float yaw; // 当前角度 (弧度)
} Pose_t;

extern Pose_t current_pose;

void Odometry_Init(void);
void Odometry_Update(int16 left_diff, int16 right_diff, float yaw_deg);
void Odometry_Reset(void);

#endif
