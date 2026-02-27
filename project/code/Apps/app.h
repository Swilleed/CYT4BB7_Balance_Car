#ifndef __APP_H
#define __APP_H

#include "zf_common_headfile.h"
#include "pid_controller.h"

typedef enum {
    TASK_IDLE = 0,
    TASK1_RUNNING,
    TASK2_RUNNING,
    TASK3_RUNNING,
    TASK4_RECORDING,
    TASK4_PLAYBACK,
    TASK5_RUNNING
} AppTask_enum;

typedef enum {
    TASK2_STAGE1_LINE,
    TASK2_STAGE2_CIRCLE,
    TASK2_STAGE3_LINE,
    TASK2_STAGE4_CIRCLE,
    TASK2_FINISH
} Task2Stage_enum;

typedef enum {
    TASK3_STAGE1_DIAGONAL,
    TASK3_STAGE2_CIRCLE,
    TASK3_STAGE3_DIAGONAL,
    TASK3_STAGE4_CIRCLE,
    TASK3_FINISH
} Task3Stage_enum;

extern AppTask_enum current_task;
extern Task2Stage_enum task2_stage;
extern Task3Stage_enum task3_stage;
extern uint8_t black_line_count;
extern float target_yaw;
extern float forward_speed;
extern uint16_t task2_lap_count;
extern uint16_t task3_lap_count;

extern pid_controller_t yaw_pid;
extern pid_controller_t line_pid;

void App_Init(void);
void App_Loop_20ms(void);

void App_Start_Task1(void);
void App_Start_Task2(void);
void App_Start_Task3(void);
void App_Start_Task4_Record(void);
void App_Start_Task4_Playback(void);
void App_Start_Task5(void);
void App_Stop_CurrentTask(void);

#endif
