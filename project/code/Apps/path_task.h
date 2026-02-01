#ifndef _PATH_TASK_H_
#define _PATH_TASK_H_

#include "zf_common_typedef.h"

typedef struct
{
    float x;
    float y;
    float yaw;
} PathNode_t;

#define MAX_PATH_NODES 1000
#define PATH_FLASH_PAGE 60

typedef enum
{
    PATH_IDLE,      // 空闲
    PATH_RECORDING, // 录制 (RAM操作)
    PATH_SAVING,    // 持久化 (Flash操作)
    PATH_PLAYBACK   // 复现
} PathState_enum;

extern PathState_enum path_state;
extern uint16 path_node_count;

void Path_Init(void);
void Path_Record_Tick(void);
void Path_Playback_Tick(float *target_v, float *target_yaw);

void Path_Start_Record(void);
void Path_Stop_Record(void);
void Path_Start_Playback(void);
void Path_Stop_Playback(void);

#endif
