#include "control_center.h"
#include "path_task.h"
#include "line_following.h"
#include "motor.h"
#include "pid_controller.h"

// 1. 物理定义 PID 结构体变量
PID_TypeDef pid_pitch_out;
PID_TypeDef pid_gyro_in;

// 全局运行模式
CarMode_enum SystemMode = MODE_HALT;

// 内部控制变量
static float target_vL = 0.0f;
static float target_vR = 0.0f;

/**
 * @brief 系统初始化
 */
void Control_Center_Init(void) {
    SystemMode = MODE_HALT;

    PID_Init(&pid_pitch_out, 15.0f, 0.0f, 0.5f);  // 初始化角度环
    PID_Init(&pid_gyro_in,   12.0f, 0.1f, 0.2f);  // 初始化角速度环

}

/**
 * @brief 模式切换处理器（处理进入模式时的初始化动作）
 */
void Control_Center_SwitchMode(CarMode_enum new_mode) {
    SystemMode = MODE_HALT; // 先切到待机防止电机猛转
    
    // 清理旧状态，开启新状态所需的零件
    switch (new_mode) {
        case MODE_1_STATIONARY:
            Motor_ResetTotalDistance(&motorLeft);
            Motor_ResetTotalDistance(&motorRight);
            break;
        case MODE_2_CLOCKWISE:
            Path_StartReplay(PATH_ID_MODE2); // 准备模式2盲走数据
            break;
        case MODE_3_COUNTER_CLOCKWISE:
            Path_StartReplay(PATH_ID_MODE3); // 准备模式3盲走数据
            break;
        case MODE_4_PATH_REPLAY:
            Path_StartReplay(PATH_ID_MODE4); // 准备模式4回放
            break;
        default: break;
    }
    
    SystemMode = new_mode;
}

/**
 * @brief 核心控制循环 (中断调用)
 */
void Control_Center_Loop(void) {
    // --- 1. 基础传感器更新 & 零件状态维护 ---
    float pitch = Madgwick_GetPitch(); // 姿态解算
    float gyro_z = gyro_get_z();       // 获取角速度
    Path_Tick_Handler();               // 路径记录/回放下标自动推移

    // 安全保护：角度过大自动停机
    if (pitch > 45.0f || pitch < -45.0f) {
        SystemMode = MODE_HALT;
    }

    if (SystemMode == MODE_HALT) {
        Motor_SetDuty(&motorLeft, 0);
        Motor_SetDuty(&motorRight, 0);
        return;
    }

    // --- 2. 模式决策层：决定 target_vL 和 target_vR ---
    switch (SystemMode) {
        case MODE_1_STATIONARY:
            // 模式1：通过位置环把车锁在原地
            // 利用 Path_GetReplaySpeed 但不自增 index，实现原地锁死
            // 或者直接计算：target = 0 - current_pos
            target_vL = (float)(0 - Motor_GetTotalDistanceCount(&motorLeft)) * 0.5f;
            target_vR = (float)(0 - Motor_GetTotalDistanceCount(&motorRight)) * 0.5f;
            break;

        case MODE_2_CLOCKWISE:
        case MODE_3_COUNTER_CLOCKWISE:
            line_update(); // 更新红外循迹
            if (line_get_state() == TRACK_OK) {
                // 有线：使用红外控制转向
                float turn_offset = (float)line_get_pid_out();
                target_vL = 40.0f - turn_offset; // 40为基础巡迹速度,可调
                target_vR = 40.0f + turn_offset;
            } else {
                // 丢线：自动调用提前录制好的 Flash 盲走路径
                Path_GetReplaySpeed(&target_vL, &target_vR);
            }
            break;

        case MODE_4_PATH_REPLAY:
            // 直接回放录制的轨迹
            Path_GetReplaySpeed(&target_vL, &target_vR);
            break;

        case MODE_5_REMOTE:
            // 速度由蓝牙或遥控器变量决定（这里示例为0）
            break;

        default: break;
    }

    // --- 3. 执行层：串级 PID 计算 & 动力融合 ---
    // 平衡环输出 (角度 -> 角速度)
    float balance_pwm = PID_Balance_Calculate(&pid_pitch_out, &pid_gyro_in, 
                                             0.0f, pitch, gyro_z);

    // 最终电机 PWM = 平衡动力 + 运动动力
    // 这种叠加方式能保证车在移动时依然维持平衡
    Motor_SetDuty(&motorLeft,  balance_pwm + target_vL);
    Motor_SetDuty(&motorRight, balance_pwm + target_vR);
}
