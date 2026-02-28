#include "zf_common_headfile.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "remote_control.h"
#include "PID.h"
#include "flash_logger.h"

extern uint32_t BlueTooth;

uint8 data_buffer[32];
uint8 data_len;
static RemoteControl_Cmd_t remote_cmd = {0, 0};
static uint8_t remote_cmd_updated = 0;

#define REMOTE_PARAM_FLASH_PAGE (100u)
#define REMOTE_PARAM_MAGIC (0x524D5450u)

typedef struct
{
    uint32 magic;
    float kp;
    float ki;
    float kd;
} Remote_PID_Param_t;

static Remote_PID_Param_t remote_pid_param = {
    .magic = REMOTE_PARAM_MAGIC,
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
};
static uint8_t remote_pid_param_updated = 0;
static uint8_t remote_param_flash_inited = 0;

static void Remote_Param_FlashInit(void)
{
    if (0 == remote_param_flash_inited)
    {
        Logger_Init();
        remote_param_flash_inited = 1;
    }
}

uint8_t Remote_Control_SavePIDParam(void)
{
    Remote_Param_FlashInit();
    Logger_WriteBlock(REMOTE_PARAM_FLASH_PAGE, (void *)&remote_pid_param, sizeof(remote_pid_param));
    return 1;
}

uint8_t Remote_Control_LoadPIDParam(void)
{
    Remote_PID_Param_t temp = {0};

    Remote_Param_FlashInit();
    Logger_ReadBlock(REMOTE_PARAM_FLASH_PAGE, (void *)&temp, sizeof(temp));

    if (REMOTE_PARAM_MAGIC != temp.magic)
    {
        return 0;
    }

    remote_pid_param = temp;
    remote_pid_param_updated = 1;
    return 1;
}

/**
 * 解析远程参数设置指令
 * 协议示例:
 * $PID,1.0,0.1,0.01  -> 更新PID参数缓存
 * $SAVE              -> 将PID参数缓存写入Flash
 * $LOAD              -> 从Flash加载PID参数缓存
 */
static uint8_t Remote_Set_Param_Parse(const uint8 *buffer, uint8 len)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    uint8 copy_len = len;
    char cmd[33] = {0};

    if ((NULL == buffer) || (0 == len))
    {
        return 0;
    }

    if ('$' != buffer[0])
    {
        return 0;
    }

    if (copy_len > 32)
    {
        copy_len = 32;
    }

    memcpy(cmd, buffer, copy_len);
    cmd[copy_len] = '\0';

    if (3 == sscanf(cmd, "$PID,%f,%f,%f", &kp, &ki, &kd))
    {
        remote_pid_param.magic = REMOTE_PARAM_MAGIC;
        remote_pid_param.kp = kp;
        remote_pid_param.ki = ki;
        remote_pid_param.kd = kd;
        remote_pid_param_updated = 1;
        return 1;
    }

    if (0 == strncmp(cmd, "$SAVE", 5))
    {
        return Remote_Control_SavePIDParam();
    }

    if (0 == strncmp(cmd, "$LOAD", 5))
    {
        return Remote_Control_LoadPIDParam();
    }

    return 0;
}

/**
 * 解析远程控制数据
 * @param buffer 接收到的数据缓冲区
 * @param len 数据长度
 * @param cmd 解析后的控制命令结构体指针
 * @return 解析成功返回1，失败返回0
 */
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

/**
 * 更新远程控制数据，从无线串口读取数据并解析
 */
void Remote_Control_Update(void)
{
    if (BlueTooth >= 1000)
    {
        BlueTooth = 0;
        data_len = (uint8)wireless_uart_read_buffer(data_buffer, 32);

        if (Remote_Set_Param_Parse(data_buffer, data_len))
        {
            return;
        }

        if (Remote_Control_Parse(data_buffer, data_len, &remote_cmd))
        {
            remote_cmd_updated = 1;
        }
    }
}

/**
 * 获取最新的远程控制命令
 * @param cmd 用于存储获取到的控制命令的结构体指针
 */
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

uint8_t Remote_Control_SetPIDParam(PID_TypeDef *pid)
{
    if ((pid == NULL) || (0 == remote_pid_param_updated))
    {
        return 0;
    }

    pid->Kp = remote_pid_param.kp;
    pid->Ki = remote_pid_param.ki;
    pid->Kd = remote_pid_param.kd;
    remote_pid_param_updated = 0;

    return 1;
}