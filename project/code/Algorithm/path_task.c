#include "path_task.h"
#include "motor.h"
#include "flash_logger.h"

// 内部状态定义
typedef enum {
    PATH_STATE_IDLE = 0,
    PATH_STATE_RECORDING,
    PATH_STATE_REPLAYING
} PathState_t;

static PathState_t currentState = PATH_STATE_IDLE;
static PathPoint_t pathBuffer[MAX_POINTS];
static uint16 pointCount = 0;
static uint16 replayIndex = 0;
static PathID_t currentID;

// 开始录制：指定存入哪个 ID 的 Flash 区域
void Path_StartRecord(PathID_t id) {
    currentID = id;
    pointCount = 0;
    Motor_ResetTotalDistance(&motorLeft);
    Motor_ResetTotalDistance(&motorRight);
    currentState = PATH_STATE_RECORDING;
}

// 停止录制：将内存中的数据批量写入 Flash
void Path_StopRecord(void) {
    if (currentState == PATH_STATE_RECORDING) {
        currentState = PATH_STATE_IDLE;
        // 先存点数，再存数组
        Logger_WriteBlock((uint32)currentID, &pointCount, sizeof(pointCount));
        Logger_WriteBlock((uint32)currentID + 1, pathBuffer, sizeof(PathPoint_t) * pointCount);
    }
}

// 开始回放：从 Flash 读取指定 ID 的数据
void Path_StartReplay(PathID_t id) {
    currentID = id;
    // 从 Flash 加载数据到 RAM
    Logger_ReadBlock((uint32)currentID, &pointCount, sizeof(pointCount));
    
    if (pointCount > 0 && pointCount <= MAX_POINTS) {
        Logger_ReadBlock((uint32)currentID + 1, pathBuffer, sizeof(PathPoint_t) * pointCount);
        replayIndex = 0;
        Motor_ResetTotalDistance(&motorLeft);
        Motor_ResetTotalDistance(&motorRight);
        currentState = PATH_STATE_REPLAYING;
    }
}

// 核心调度：放在定时器中执行 (每XXX ms 调用一次)
void Path_Tick_Handler(void) {
    if (currentState == PATH_STATE_RECORDING) {
        if (pointCount < MAX_POINTS) {
            pathBuffer[pointCount].left_pos = Motor_GetTotalDistanceCount(&motorLeft);
            pathBuffer[pointCount].right_pos = Motor_GetTotalDistanceCount(&motorRight);
            pointCount++;
        } else {
            Path_StopRecord(); // 自动溢出停止
        }
    } 

    else if (currentState == PATH_STATE_REPLAYING) {
        if (replayIndex < pointCount - 1) {
            replayIndex++; 
        } else {
            currentState = PATH_STATE_IDLE; // 路径跑完，自动停下
        }
    }
}

// 计算回放时的目标速度（位置环控制）
void Path_GetReplaySpeed(float *outL, float *outR) {
    if (currentState == PATH_STATE_REPLAYING) {
        // 计算当前位置与录制位置的误差
        int32 errL = pathBuffer[replayIndex].left_pos - Motor_GetTotalDistanceCount(&motorLeft);
        int32 errR = pathBuffer[replayIndex].right_pos - Motor_GetTotalDistanceCount(&motorRight);
        
        // 这里的 0.6f 是位置增益系数，根据需要调整哈
        *outL = (float)errL * 0.6f;
        *outR = (float)errR * 0.6f;
    } else {
        *outL = 0;
        *outR = 0;
    }
}

uint8 Path_IsReplaying(void) {
    return (currentState == PATH_STATE_REPLAYING);
}

void Path_Delete(PathID_t id) {
    uint32 zero = 0;
    Logger_WriteBlock((uint32)id, &zero, sizeof(zero)); // 点数标0即删除
}
