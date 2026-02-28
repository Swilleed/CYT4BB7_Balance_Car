#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "zf_common_headfile.h"
#include "PID.h"

typedef struct
{
    int32_t forward;
    int32_t turn;
} RemoteControl_Cmd_t;

void Remote_Control_Update(void);
uint8_t Remote_Control_GetCmd(RemoteControl_Cmd_t *cmd);
uint8_t Remote_Control_SetPIDParam(PID_TypeDef *pid);
uint8_t Remote_Control_SavePIDParam(void);
uint8_t Remote_Control_LoadPIDParam(void);

#endif /* REMOTE_CONTROL_H */
