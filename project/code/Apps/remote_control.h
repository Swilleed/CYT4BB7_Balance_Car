#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "zf_common_headfile.h"

typedef struct
{
    int32_t forward;
    int32_t turn;
} RemoteControl_Cmd_t;

void Remote_Control_Update(void);
uint8_t Remote_Control_GetCmd(RemoteControl_Cmd_t *cmd);

#endif /* REMOTE_CONTROL_H */
