#include "zf_common_headfile.h"
#include <stdlib.h>
#include "remote_control.h"

extern uint32_t BlueTooth;

uint8 data_buffer[32];
uint8 data_len;
static RemoteControl_Cmd_t remote_cmd = {0, 0};
static uint8_t remote_cmd_updated = 0;

static uint8_t Remote_Control_Parse(const uint8 *buffer, uint8 len, RemoteControl_Cmd_t *cmd)
{
    char temp[2][6];
    int i = 0, j = 0;

    if (len == 0)
    {
        return 0;
    }

    for (i = 0; (i < 5) && (10 + i < len) && buffer[10 + i] != ','; i++)
    {
        temp[0][i] = (char)buffer[10 + i];
    }
    temp[0][i] = '\0';

    for (j = 0; (j < 5) && (11 + i + j < len) && buffer[11 + i + j] != ','; j++)
    {
        temp[1][j] = (char)buffer[11 + i + j];
    }
    temp[1][j] = '\0';

    cmd->turn = atoi(temp[0]);
    cmd->forward = atoi(temp[1]);
    return 1;
}

void Remote_Control_Update(void)
{
    if (BlueTooth >= 1000)
    {
        BlueTooth = 0;
        data_len = (uint8)wireless_uart_read_buffer(data_buffer, 32);
        if (Remote_Control_Parse(data_buffer, data_len, &remote_cmd))
        {
            remote_cmd_updated = 1;
        }
    }
}

uint8_t Remote_Control_GetCmd(RemoteControl_Cmd_t *cmd)
{
    if ((0 == remote_cmd_updated) || (NULL == cmd))
    {
        return 0;
    }

    cmd->forward = remote_cmd.forward;
    cmd->turn = remote_cmd.turn;
    remote_cmd_updated = 0;
    return 1;
}