#include "app.h"
#include "path_task.h"
#include "infrared_sense.h"
#include "odometry.h"
#include "balance_task.h"
#include "Madgwick.h"
#include "remote_control.h"

AppTask_enum current_task = TASK_IDLE;
Task2Stage_enum task2_stage = TASK2_STAGE1_LINE;
Task3Stage_enum task3_stage = TASK3_STAGE1_DIAGONAL;
uint8_t black_line_count = 0;
float target_yaw = 0.0f;
float forward_speed = 15.0f;
uint16_t task2_lap_count = 0;
uint16_t task3_lap_count = 0;

pid_controller_t yaw_pid;
pid_controller_t line_pid;

static float Get_Current_Yaw(void)
{
    extern float Yaw;
    return Yaw;
}

static uint8_t Check_BlackLine_Rising(void)
{
    static uint8_t last_sensor = 0;
    uint8_t current_sensor = GetInfraredSenseFlag();
    uint8_t edge = (last_sensor == 0 && current_sensor != 0) ? 1 : 0;
    last_sensor = current_sensor;
    return edge;
}

static uint8_t Check_BlackLine_Falling(void)
{
    static uint8_t last_sensor = 0;
    uint8_t current_sensor = GetInfraredSenseFlag();
    uint8_t edge = (last_sensor != 0 && current_sensor == 0) ? 1 : 0;
    last_sensor = current_sensor;
    return edge;
}

static void Line_Following_Control(void)
{
    uint8_t sensor = GetInfraredSenseFlag();
    int error = 0;
    if(sensor & 0x10) error -= 2;
    if(sensor & 0x08) error -= 1;
    if(sensor & 0x02) error += 1;
    if(sensor & 0x01) error += 2;
    float turn_cmd = PID_Calculate(&line_pid, 0, -error);
    Balance_SetMotionCmd((int32_t)forward_speed, (int32_t)turn_cmd);
}

static void Straight_Line_Control(void)
{
    float current_yaw = Get_Current_Yaw();
    float yaw_error = target_yaw - current_yaw;
    if(yaw_error > 180) yaw_error -= 360;
    if(yaw_error < -180) yaw_error += 360;
    float turn_cmd = PID_Calculate(&yaw_pid, 0, yaw_error);
    Balance_SetMotionCmd((int32_t)forward_speed, (int32_t)turn_cmd);
}

static void Diagonal_Line_Control(float angle_offset)
{
    float current_yaw = Get_Current_Yaw();
    float target = target_yaw + angle_offset;
    if(target > 360) target -= 360;
    if(target < 0) target += 360;
    float yaw_error = target - current_yaw;
    if(yaw_error > 180) yaw_error -= 360;
    if(yaw_error < -180) yaw_error += 360;
    float turn_cmd = PID_Calculate(&yaw_pid, 0, yaw_error);
    Balance_SetMotionCmd((int32_t)forward_speed, (int32_t)turn_cmd);
}

static void Task2_Update(void)
{
    static uint8_t lap_finished = 0;
    switch(task2_stage) {
        case TASK2_STAGE1_LINE:
            if(Check_BlackLine_Rising()) {
                task2_stage = TASK2_STAGE2_CIRCLE;
                black_line_count++;
            } else {
                Straight_Line_Control();
            }
            break;
        case TASK2_STAGE2_CIRCLE:
            Line_Following_Control();
            if(Check_BlackLine_Falling()) {
                task2_stage = TASK2_STAGE3_LINE;
                target_yaw = Get_Current_Yaw();
            }
            break;
        case TASK2_STAGE3_LINE:
            if(Check_BlackLine_Rising()) {
                task2_stage = TASK2_STAGE4_CIRCLE;
                black_line_count++;
            } else {
                Straight_Line_Control();
            }
            break;
        case TASK2_STAGE4_CIRCLE:
            Line_Following_Control();
            if(Check_BlackLine_Falling()) {
                lap_finished++;
                task2_lap_count = lap_finished;
                if(lap_finished >= 1) {
                    task2_stage = TASK2_FINISH;
                    current_task = TASK_IDLE;
                    Balance_SetMotionCmd(0, 0);
                    lap_finished = 0;
                } else {
                    task2_stage = TASK2_STAGE1_LINE;
                    target_yaw = Get_Current_Yaw();
                }
            }
            break;
        default:
            break;
    }
}

static void Task3_Update(void)
{
    static uint8_t lap_finished = 0;
    switch(task3_stage) {
        case TASK3_STAGE1_DIAGONAL:
            if(Check_BlackLine_Rising()) {
                task3_stage = TASK3_STAGE2_CIRCLE;
                black_line_count++;
            } else {
                Diagonal_Line_Control(45.0f);
            }
            break;
        case TASK3_STAGE2_CIRCLE:
            Line_Following_Control();
            if(Check_BlackLine_Falling()) {
                task3_stage = TASK3_STAGE3_DIAGONAL;
                target_yaw = Get_Current_Yaw();
            }
            break;
        case TASK3_STAGE3_DIAGONAL:
            if(Check_BlackLine_Rising()) {
                task3_stage = TASK3_STAGE4_CIRCLE;
                black_line_count++;
            } else {
                Diagonal_Line_Control(-45.0f);
            }
            break;
        case TASK3_STAGE4_CIRCLE:
            Line_Following_Control();
            if(Check_BlackLine_Falling()) {
                lap_finished++;
                task3_lap_count = lap_finished;
                if(lap_finished >= 4) {
                    task3_stage = TASK3_FINISH;
                    current_task = TASK_IDLE;
                    Balance_SetMotionCmd(0, 0);
                    lap_finished = 0;
                } else {
                    task3_stage = TASK3_STAGE1_DIAGONAL;
                    target_yaw = Get_Current_Yaw();
                }
            }
            break;
        default:
            break;
    }
}

void App_Init(void)
{
    PID_Init(&yaw_pid, 1.5f, 0.0f, 0.5f, -30.0f, 30.0f);
    PID_Init(&line_pid, 0.8f, 0.0f, 0.3f, -20.0f, 20.0f);
    current_task = TASK_IDLE;
    black_line_count = 0;
    task2_lap_count = 0;
    task3_lap_count = 0;
}

void App_Loop_20ms(void)
{
    InfraredSensor_Tick();
    switch(current_task) {
        case TASK1_RUNNING:
        case TASK_IDLE:
            break;
        case TASK2_RUNNING:
            Task2_Update();
            break;
        case TASK3_RUNNING:
            Task3_Update();
            break;
        case TASK4_RECORDING:
            Path_Record_Tick();
            break;
        case TASK4_PLAYBACK:
            {
                float target_v, target_yaw_deg;
                Path_Playback_Tick(&target_v, &target_yaw_deg);
                Balance_SetMotionCmd((int32_t)target_v, (int32_t)target_yaw_deg);
            }
            break;
        case TASK5_RUNNING:
            {
                RemoteControl_Cmd_t cmd;
                if (Remote_Control_GetCmd(&cmd)) {
                    Balance_SetMotionCmd(cmd.forward, cmd.turn);
                }
            }
            break;
        default:
            break;
    }
}

void App_Start_Task1(void)
{
    current_task = TASK1_RUNNING;
}

void App_Start_Task2(void)
{
    current_task = TASK2_RUNNING;
    task2_stage = TASK2_STAGE1_LINE;
    black_line_count = 0;
    target_yaw = Get_Current_Yaw();
    task2_lap_count = 0;
}

void App_Start_Task3(void)
{
    current_task = TASK3_RUNNING;
    task3_stage = TASK3_STAGE1_DIAGONAL;
    black_line_count = 0;
    target_yaw = Get_Current_Yaw();
    task3_lap_count = 0;
}

void App_Start_Task4_Record(void)
{
    current_task = TASK4_RECORDING;
    Path_Start_Record();
}

void App_Start_Task4_Playback(void)
{
    extern uint16 path_node_count;
    if(path_node_count > 0) {
        current_task = TASK4_PLAYBACK;
        Path_Start_Playback();
    }
}

void App_Start_Task5(void)
{
    current_task = TASK5_RUNNING;
}

void App_Stop_CurrentTask(void)
{
    if(current_task == TASK4_RECORDING) {
        Path_Stop_Record();
    } else if(current_task == TASK4_PLAYBACK) {
        Path_Stop_Playback();
    }
    current_task = TASK_IDLE;
    Balance_SetMotionCmd(0, 0);
}
