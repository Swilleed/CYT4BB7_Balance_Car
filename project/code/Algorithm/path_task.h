#ifndef PATH_TASK_H
#define PATH_TASK_H

#include "zf_common_headfile.h"

// 定义路径点结构体：记录左右轮的累计绝对位移
typedef struct {
    int32 left_pos;   
    int32 right_pos;  
} PathPoint_t;

// Flash 存储区域分配：每个 ID 间隔 100 页（假设每页 2KB，空间绰绰有余）
typedef enum {
    PATH_ID_MODE2 = 100,  // 模式2路径存放在 100 页
    PATH_ID_MODE3 = 200,  // 模式3路径存放在 200 页
    PATH_ID_MODE4 = 300   // 模式4临时路径存放在 300 页
} PathID_t;

#define MAX_POINTS 800    // 最大记录点数（根据 RAM 大小调整）

// 外部调用接口
void Path_Delete(PathAreaID_t area);
void Path_StartRecord(PathID_t id);
void Path_StopRecord(void);
void Path_StartReplay(PathID_t id);
void Path_Tick_Handler(void); // 建议 20ms-40ms 调用一次

// 获取回放时的补偿速度（给 PID 的输入）
void Path_GetReplaySpeed(float *outL, float *outR);

// 获取当前状态（用于 OLED 显示或逻辑判断）
uint8 Path_IsReplaying(void);

#endif
