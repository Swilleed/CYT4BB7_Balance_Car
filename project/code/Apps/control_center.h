#ifndef CONTROL_CENTER_H
#define CONTROL_CENTER_H

#include "zf_common_headfile.h"

// 定义五个发车模式
typedef enum {
    MODE_HALT = 0,             // 紧急停止/待机
    MODE_1_STATIONARY,         // 模式1：原地平衡（锁死位置）
    MODE_2_CLOCKWISE,          // 模式2：顺时针巡迹+盲走
    MODE_3_COUNTER_CLOCKWISE,  // 模式3：逆时针巡迹+盲走
    MODE_4_PATH_REPLAY,        // 模式4：路径录制与回放
    MODE_5_REMOTE              // 模式5：远程控制
} CarMode_enum;

// 全局变量声明
extern CarMode_enum SystemMode;

// 声明全局 PID 结构体，这样菜单模块也能访问它们进行动态调参
extern PID_TypeDef pid_pitch_out; // 角度环
extern PID_TypeDef pid_gyro_in;   // 角速度环

// 函数接口
void Control_Center_Init(void);
void Control_Center_Loop(void);          // 核心控制，建议由10ms定时器触发
void Control_Center_SwitchMode(CarMode_enum new_mode); // 菜单切换模式调用

#endif
