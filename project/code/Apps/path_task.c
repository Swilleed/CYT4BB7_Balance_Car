#include "path_task.h"
#include "odometry.h"
#include "flash_logger.h"
#include <math.h>
#include <string.h>

PathState_enum path_state = PATH_IDLE;
uint16 path_node_count = 0;
static uint16 playback_index = 0;
static PathNode_t path_buffer[MAX_PATH_NODES];

#define KP_DIST 1.5f
#define KP_TURN 2.0f

/**
 * 初始化路径任务模块
 */
void Path_Init(void)
{
    path_state = PATH_IDLE;
    path_node_count = 0;
    memset(path_buffer, 0, sizeof(path_buffer));
}

/**
 * 开始路径录制
 */
void Path_Start_Record(void)
{
    path_node_count = 0;
    Odometry_Reset();
    path_state = PATH_RECORDING;
}

/**
 * 停止路径录制并保存到 Flash
 */
void Path_Stop_Record(void)
{
    if (path_state == PATH_RECORDING)
    {
        path_state = PATH_SAVING;
        // 结束录制时一并存入 Flash
        Logger_WriteBlock(PATH_FLASH_PAGE, (void *)path_buffer, sizeof(path_buffer));
        path_state = PATH_IDLE;
    }
}

/**
 * 开始路径复现
 */
void Path_Start_Playback(void)
{
    // 从 Flash 读取到 RAM
    Logger_ReadBlock(PATH_FLASH_PAGE, (void *)path_buffer, sizeof(path_buffer));
    playback_index = 0;
    Odometry_Reset();
    path_state = PATH_PLAYBACK;
}

/**
 * 停止路径复现
 */
void Path_Stop_Playback(void)
{
    path_state = PATH_IDLE;
}

/**
 * 录制一帧数据 (建议 20ms 调用一次)
 */
void Path_Record_Tick(void)
{
    if (path_state != PATH_RECORDING)
        return;

    if (path_node_count < MAX_PATH_NODES)
    {
        path_buffer[path_node_count].x = current_pose.x;
        path_buffer[path_node_count].y = current_pose.y;
        path_buffer[path_node_count].yaw = current_pose.yaw;
        path_node_count++;
    }
    else
    {
        Path_Stop_Record();
    }
}

/**
 * 路径复现处理 (建议 20ms 调用一次)
 * @param target_v 输出目标线速度 (单位：脉冲/周期)
 * @param target_yaw_deg 输出目标航向角 (单位：度)
 */
void Path_Playback_Tick(float *target_v, float *target_yaw_deg)
{
    if (path_state != PATH_PLAYBACK)
        return;

    if (playback_index >= path_node_count)
    {
        Path_Stop_Playback();
        *target_v = 0;
        return;
    }

    PathNode_t *target = &path_buffer[playback_index];

    // 计算轨迹距离误差
    float dx = target->x - current_pose.x;
    float dy = target->y - current_pose.y;
    float dist_error = sqrtf(dx * dx + dy * dy);

    // 追赶距离
    *target_v = dist_error * KP_DIST;

    *target_yaw_deg = target->yaw * 180.0f / 3.14159f;

    playback_index++;
}
