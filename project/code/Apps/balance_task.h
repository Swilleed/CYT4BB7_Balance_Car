#ifndef BALANCE_TASK_H
#define BALANCE_TASK_H

#include "zf_common_headfile.h"

extern uint32_t IMUcounter;

void Balance_Control_Loop(void);
void Balance_Foc_Loop(void);
void Balance_Attitude_Update_From_Ipc(uint32 receive_data);
void Balance_SetMotionCmd(int32_t forward_cmd, int32_t turn_cmd);
void Balance_Remote_SetSpeed(int32_t forward_cmd, int32_t turn_cmd);
uint8_t Balance_Chase_Position(float target_x, float target_y, float target_yaw_rad);

#endif /* BALANCE_TASK_H */
