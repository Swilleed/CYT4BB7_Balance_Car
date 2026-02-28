/*********************************************************************************************************************
 * CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 CYT4BB 开源库的一部分
 *
 * CYT4BB 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main_cm7_0
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 9.40.1
 * 适用平台          CYT4BB
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-4       pudding            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "balance_task.h"
#include "remote_control.h"
#include "app.h"
#include "AS5600.h"
#include "FOC.h"

uint32_t FOCcounter = 0;
uint32_t IMUcounter = 0;
uint32_t BlueTooth = 0;
soft_iic_info_struct AS561, AS562;

/**
 * IPC回调函数，接收来自M7_1的姿态数据并更新小车姿态
 * @param receive_data 从M7_1发送过来的数据，包含roll、yaw、pitch信息
 */
void my_ipc_callback(uint32 receive_data)
{
    Balance_Attitude_Update_From_Ipc(receive_data);
}

int main(void)
{
    RemoteControl_Cmd_t remote_cmd;
    uint16_t app_tick_1ms_acc = 0;

    clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
    debug_init();                  // 调试串口信息初始化
    oled_init();
    wireless_uart_init();
    system_delay_init();

    SCB_DisableDCache();
    ipc_communicate_init(IPC_PORT_1, my_ipc_callback);

    pit_init(PIT_CH0, 100); // 初始化周期定时器，周期为100us
    pit_enable(PIT_CH0);

    AS5600_Init(&AS561, 1);
    AS5600_Init(&AS562, 2);
    FOC_init();
    App_Mode_Init();

    while (true)
    {
        Remote_Control_Update(); // 更新远程控制数据，从无线串口读取数据并解析

        // 获取最新的远程控制命令
        if ((!App_Mode_IsTeachActive()) && Remote_Control_GetCmd(&remote_cmd))
        {
            Balance_SetMotionCmd(remote_cmd.forward, remote_cmd.turn);
        }

        // 每2.5ms执行一次平衡控制循环
        if (IMUcounter >= 10)
        {
            Balance_Control_Loop();
            IMUcounter = 0;

            app_tick_1ms_acc++;
            if (app_tick_1ms_acc >= 20)
            {
                app_tick_1ms_acc = 0;
                App_Mode_Tick20ms();
            }
        }

        // 每0.5ms执行一次FOC控制循环
        if (FOCcounter >= 5)
        {
            Balance_Foc_Loop();
            FOCcounter = 0;
        }
    }
}
