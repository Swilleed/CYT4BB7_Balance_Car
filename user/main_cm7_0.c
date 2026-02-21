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
#include <stdlib.h>
#include "FOC.h"
#include "AS5600.h"
#include "math.h"
#include "PID.h"
#include "KF.h"
#include "Motor.h"
#include "Madgwick.h"
#include "Position.h"
#include "Menu.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***************************
// 核心板正常供电即可 无需额外连接
// 如果使用主板测试 主板必须要用电池供电


// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，完成上电
// 2.可以看到核心板上四个 LED 呈流水灯状闪烁
// 3.将 SWITCH1 / SWITCH2 两个宏定义对应的引脚分别按照 00 01 10 11 的组合接到 1-VCC 0-GND 或者波动对应主板的拨码开关
// 3.不同的组合下，四个 LED 流水灯状闪烁的频率会发生变化
// 4.将 KEY1 / KEY2 / KEY3 / KEY4 两个宏定义对应的引脚接到 1-VCC 0-GND 或者 按对应按键
// 5.任意引脚接 GND 或者 按键按下会使得所有LED一起闪烁 松开后恢复流水灯
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************
#define LED1                    (P19_0)

#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)

#define SWITCH1                 (P21_5)
#define SWITCH2                 (P21_6)

uint16 delay_time = 0;
uint8 led_state = 0;

extern uint8_t FOCcounter;
extern uint8_t IMUcounter;
extern uint16_t BlueTooth;
extern uint16_t KeyCounter;
extern uint16_t OLEDCounter; 

soft_iic_info_struct AS561,AS562;

Position_Data POS = {
  .X = 0,
  .Y = 0,
  .Yaw = 0,
  
};


KalmanFilter1D_Speed X_GYRO = {
    .Speed_Hat = 0.0f,  
    .P = 1.0f,          
    .Q = 0.0005f,       
    .R = 0.5f         
};
KalmanFilter1D_Speed FORWARD = {
    .Speed_Hat = 0.0f,  
    .P = 1.0f,          
    .Q = 0.0005f,       
    .R = 0.5f         
};
KalmanFilter1D_Speed TURN = {
    .Speed_Hat = 0.0f,  
    .P = 1.0f,          
    .Q = 0.0005f,       
    .R = 0.5f         
};
PID_TypeDef M1 = {
	
	.error0 = 0,
	.error1 = 0,
	.error2 = 0,
	
	.Kp = 20,
	.Ki = 2,
	.Kd = 0,
	
	.OutMax = 5,
	.OutMin = -5,
	
	.Target = 0,
	.Out = 0,
	
};
PID_TypeDef M2 = {
	
	.error0 = 0,
	.error1 = 0,
	.error2 = 0,
	
	.Kp = 20,
	.Ki = 2,
	.Kd = 0,
	
	.OutMax = 5,
	.OutMin = -5,
	
	.Target = 0,
	.Out = 0,
	
};

float Roll,Yaw,Pitch,x_gyro,X_gyro = 0,Last_Roll = 0;

PID_TypeDef _Angle_Speed = {
	
	.error0 = 0,
	.error1 = 0,
	.error2 = 0,
	
	.Kp = 0.3,
	.Ki = 0,
	.Kd = 4,
	
	.OutMax = 14,
	.OutMin = -14,
	
	.Target = 0,
	.Out = 0,
	
};

float BaseAngle = 0.5;
PID_TypeDef _Angle = {
	
	.error0 = 0,
	.error1 = 0,
	.error2 = 0,
	
	.Kp = 0.9,
	.Ki = 0,
	.Kd = 6,
	
	.OutMax = 25,
	.OutMin = -25,
	
	.Target = 0,
	.Out = -0,
	
};

float current_speed,L_Speed,R_Speed;
PID_TypeDef _Speed = {
	
	.error0 = 0,
	.error1 = 0,
	.error2 = 0,
	
	.Kp = 1.2,
	.Ki = 0.0008,
	.Kd = 2.5,
	
	.OutMax = 7,
	.OutMin = -7,
	
	.Target = 0,
	.Out = 0,
	
};

PID_TypeDef _Dir = {
	
	.error0 = 0,
	.error1 = 0,
	.error2 = 0,
	
	.Kp = 0.10,
	.Ki = 0.00,
	.Kd = 5,
	
	.OutMax = 0.8,
	.OutMin = -0.8,
	
	.Target = 0,
	.Out = 0,
	
};

PID_TypeDef _Pos = {
	
	.error0 = 0,
	.error1 = 0,
	.error2 = 0,
	
	.Kp = 15.0,
	.Ki = 0.00,
	.Kd = 40.0,
	
	.OutMax = 6,
	.OutMin = -6,
	
	.Target = 0,
	.Out = 0,
	
};


uint8 data_buffer[32];
uint8 data_len;


void my_ipc_callback(uint32 receive_data)
{
    static uint8_t i = 0;
    
    if(i == 0){
      Last_Roll = Roll;
      Roll = ((float)receive_data / 100) - 180;
      KalmanFilter1D_Speed_Update(&X_GYRO,(Roll - Last_Roll) / 0.01);
      X_gyro = X_GYRO.Speed_Hat > 25 ? 25 : (X_GYRO.Speed_Hat < -25 ? -25 : X_GYRO.Speed_Hat);
      
    } else if (i == 1){
      Yaw = ((float)receive_data / 100) - 180;
    } else if (i == 2){
      Pitch = ((float)receive_data / 100) - 180;
    }
    i ++;
    if(i == 3){i = 0;}
    
}

int forward_speed = 0,turn_speed = 0;

int main(void)
{
    
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                          // 调试串口信息初始化
    oled_init();
    wireless_uart_init();
    key_init(10);
    Menu_Init();
    
    SCB_DisableDCache(); 
    ipc_communicate_init(IPC_PORT_1, my_ipc_callback);
    
    pit_init(PIT_CH0,100);
    pit_enable(PIT_CH0); 
    
    
    AS5600_Init(&AS561,1);
    AS5600_Init(&AS562,2);
    FOC_init();
    
    
    while(true)
    {
        if(KeyCounter >= 100){
          KeyCounter = 0;
          Key_Scanner();
        }
        if(OLEDCounter >= 500){
          OLEDCounter = 0;
           OLED_ShowMenu(); 
        }
      
        if(BlueTooth >= 1000){
          BlueTooth = 0;
           data_len = (uint8)wireless_uart_read_buffer(data_buffer,32);             // 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
          if(data_len != 0)                                                       // 收到了消息 读取函数会返回实际读取到的数据个数
          {
            
             char temp[2][6];  
              int i = 0, j = 0;
              for (i = 0; (i < 5) && (10 + i < data_len) && data_buffer[10 + i] != ','; i++) {
                  temp[0][i] = (char)data_buffer[10 + i];
              }
              temp[0][i] = '\0';  
              for (j = 0; (j < 5) && (11 + i + j < data_len) && data_buffer[11 + i + j] != ','; j++) {
                  temp[1][j] = (char)data_buffer[11 + i + j];
              }
              temp[1][j] = '\0';  
    
             turn_speed = atoi(temp[0]);
             forward_speed = atoi(temp[1]);

            memset(data_buffer, 0, 32);
            
          }
        }
        
        if(IMUcounter >= 10){
            IMUcounter = 0;
            
            current_speed = (L_Speed + R_Speed) / 2;
            Position_cal(&POS,Yaw,current_speed * 3 / 7,0.001);
            _Pos.Actual = POS.Y;
            PID_Update_Pos(&_Pos);

            if(Mission == 5){
              KalmanFilter1D_Speed_Update(&FORWARD,(forward_speed / 15.0));
              KalmanFilter1D_Speed_Update(&TURN,(turn_speed / 50.0));
            
              _Speed.Target = FORWARD.Speed_Hat;
              _Dir.Target = TURN.Speed_Hat;    
            }
            
           _Speed.Target = _Pos.Out;
           _Speed.Actual = current_speed;
           PID_Update_Pos(&_Speed);
           
           _Dir.Actual = L_Speed - R_Speed;
           _Dir.Actual = _Dir.Actual < (_Dir.Target - 1) ? _Dir.Actual : (_Dir.Actual > (_Dir.Target + 1) ? _Dir.Actual : _Dir.Target);
           PID_Update_Pos(&_Dir);
           
           _Angle.Target = BaseAngle - _Speed.Out;
           _Angle.Actual = Roll;
           PID_Update_Pos(&_Angle);
           
           _Angle_Speed.Target =  0.1 * _Angle.Out + 0.9 * _Angle_Speed.Target ;
           _Angle_Speed.Actual = X_gyro;
           PID_Update_Pos(&_Angle_Speed);
           oled_show_float(64,6,_Angle_Speed.Out,3,2);
           oled_show_float(64,7,_Pos.Out,3,2);
           
           printf("%.2f,%.2f,%.2f\r\n",POS.X,POS.Y,POS.Yaw);
           system_delay_us(2100); 
           
         
           
        }
        
        if(FOCcounter >= 5){
          FOCcounter = 0;

          Motor_SpeedSet(&M1,1,(_Angle_Speed.Out + 2 * _Dir.Out ) * 7.5 );
          Motor_SpeedSet(&M2,2,(_Angle_Speed.Out - 2 * _Dir.Out ) * 7.5 );
         
          //Motor_SpeedSet(&M2,2,forward_speed);
          
          //printf("%.2f,%.2f,%.2f\n",M1.Target,M1.Actual,M1.Out);
          //printf("%.2f,%.2f,%.2f\n",M2.Target,M2.Actual,M2.Out);
          //printf("%.2f,%.2f,%.2f\r\n",Roll,Pitch,-Yaw);
          
          system_delay_us(1400); 
        }
        
        
        
        led_state = !led_state;
        system_delay_ms(delay_time);
        gpio_set_level(LED1, led_state);
        
        // 此处编写需要循环执行的代码
    }
}




// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：LED 不闪烁
//      如果使用主板测试，主板必须要用电池供电
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
// 问题2：SWITCH1 / SWITCH2 更改组合流水灯频率无变化
//      如果使用主板测试，主板必须要用电池供电
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
//      万用表检查对应 SWITCH1 / SWITCH2 引脚电压是否正常变化，是否跟接入信号不符，引脚是否接错
// 问题2：KEY1 / KEY2 / KEY3 / KEY4 接GND或者按键按下无变化
//      如果使用主板测试，主板必须要用电池供电
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表测量对应 LED 引脚电压是否变化，如果不变化证明程序未运行，如果变化证明 LED 灯珠损坏
//      万用表检查对应 KEY1 / KEY2 / KEY3 / KEY4 引脚电压是否正常变化，是否跟接入信号不符，引脚是否接错
