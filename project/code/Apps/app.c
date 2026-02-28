#include "app.h"

#include "path_task.h"
#include "balance_task.h"
#include "flash_logger.h"
#include "odometry.h"

#define APP_TICK_SEC (0.02f)

// =========================
// 预留功能（暂不启用，全部注释）
// 1) 航向误差归一化到 [-180, 180]
// 2) 转向参数化，便于赛前快速调参
//
// #define APP_TURN_KP_EXT (0.8f)
// #define APP_TURN_CMD_LIMIT (120.0f)
//
// static float App_NormalizeAngleDeg(float angle_deg)
// {
//     while (angle_deg > 180.0f)
//     {
//         angle_deg -= 360.0f;
//     }
//     while (angle_deg < -180.0f)
//     {
//         angle_deg += 360.0f;
//     }
//     return angle_deg;
// }
// =========================

static AppModeState_enum app_mode_state = APP_MODE_REMOTE;

extern float Yaw;
extern float L_Speed;
extern float R_Speed;

/**
 * 用电机速度估算本周期里程计增量并更新位姿
 * 为什么这样写：示教录制/回放依赖 current_pose，必须在固定周期持续更新里程计。
 * 怎么实现：
 * 1) 读取平衡模块中的 L_Speed/R_Speed；
 * 2) 按 20ms 周期换算为 left_diff/right_diff；
 * 3) 调用 Odometry_Update(left_diff, right_diff, Yaw) 更新位姿。
 * 怎么调用：由 App_Mode_Tick20ms 每次进入时先调用一次。
 * 对应效果：Path_Record_Tick 录到的是连续轨迹，Path_Playback_Tick 依据实时位姿追点。
 */
static void App_UpdateOdometryFromSpeed(void)
{
    int16 left_diff = 0;
    int16 right_diff = 0;

    left_diff = (int16)(L_Speed * APP_TICK_SEC);
    right_diff = (int16)(R_Speed * APP_TICK_SEC);

    Odometry_Update(left_diff, right_diff, Yaw);

    // TODO: L_Speed/R_Speed 单位与 Odometry_Update 的脉冲单位可能不一致。
    // 后续可在此处增加比例系数做标定，当前先保证示教状态机闭环可运行。
}

/**
 * 模式模块初始化
 * 为什么这样写：把示教模式依赖的资源统一初始化，避免分散在 main 中导致遗漏。
 * 怎么实现：依次初始化 Flash、路径模块、里程计、按键扫描，并将模式置为 REMOTE。
 * 怎么调用：在 main_cm7_0.c 启动阶段调用一次即可。
 * 对应效果：示教模式进入前就具备录制、回放、按键、里程计的基础条件。
 */
void App_Mode_Init(void)
{
    Logger_Init();
    Path_Init();
    Odometry_Init();
    key_init(20);
    app_mode_state = APP_MODE_REMOTE;
}

/**
 * 示教模式 20ms 周期状态机
 * 为什么这样写：将示教流程（录制/回放/退出）做成固定节拍状态机，保证逻辑稳定且低耦合。
 * 怎么实现：
 * 1) 扫描按键并更新里程计；
 * 2) 根据当前状态处理事件与转移：
 *    - REMOTE：可进入录制或回放；
 *    - TEACH_RECORDING：周期录制，支持停止/退出；
 *    - TEACH_IDLE：等待回放或退出；
 *    - TEACH_PLAYBACK：周期追点，支持中断退出；
 * 3) 回放结束或中断时统一下发停车指令。
 * 怎么调用：由主循环每 20ms 调用一次。
 * 对应效果：实现“按钮触发示教、录制路径、保存、回放复现、可随时退出”的完整闭环。
 */
void App_Mode_Tick20ms(void)
{
    key_scanner();
    App_UpdateOdometryFromSpeed();

    switch (app_mode_state)
    {
    case APP_MODE_REMOTE:
        if (key_get_state(KEY_3) == KEY_LONG_PRESS)
        {
            Path_Start_Record();
            app_mode_state = APP_MODE_TEACH_RECORDING;
        }
        else if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            Path_Start_Playback();
            app_mode_state = APP_MODE_TEACH_PLAYBACK;
        }
        break;

    case APP_MODE_TEACH_RECORDING:
        Path_Record_Tick();

        if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            Path_Stop_Record();
            app_mode_state = APP_MODE_TEACH_IDLE;
        }
        else if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
            Path_Stop_Record();
            app_mode_state = APP_MODE_REMOTE;
        }
        break;

    case APP_MODE_TEACH_IDLE:
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            Path_Start_Playback();
            app_mode_state = APP_MODE_TEACH_PLAYBACK;
        }
        else if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
            app_mode_state = APP_MODE_REMOTE;
        }
        break;

    case APP_MODE_TEACH_PLAYBACK:
        Path_Playback_Tick();

        if (path_state != PATH_PLAYBACK)
        {
            Balance_SetMotionCmd(0, 0);
            app_mode_state = APP_MODE_TEACH_IDLE;
            break;
        }

        if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
            Path_Stop_Playback();
            Balance_SetMotionCmd(0, 0);
            app_mode_state = APP_MODE_TEACH_IDLE;
        }
        break;

    default:
        app_mode_state = APP_MODE_REMOTE;
        break;
    }
}

/**
 * 查询示教模式是否处于激活状态
 * 为什么这样写：主循环需要据此做控制源仲裁，避免遥控与示教同时写指令。
 * 怎么实现：只要状态不是 APP_MODE_REMOTE 就返回 1。
 * 怎么调用：main_cm7_0.c 在读取遥控并下发指令前调用。
 * 对应效果：示教模式运行时遥控被屏蔽，不会出现控制冲突。
 */
uint8_t App_Mode_IsTeachActive(void)
{
    return (app_mode_state != APP_MODE_REMOTE);
}

/**
 * 获取当前模式状态
 * 为什么这样写：便于 OLED/串口调试显示当前状态，快速定位流程问题。
 * 怎么实现：直接返回静态状态变量 app_mode_state。
 * 怎么调用：调试逻辑中按需调用。
 * 对应效果：可以实时观察状态机所处阶段，提升调试效率。
 */
AppModeState_enum App_Mode_GetState(void)
{
    return app_mode_state;
}
