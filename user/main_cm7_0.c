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
#include "Bluetooth.h"
#include "Control.h"
#include "Infrared.h"
#include "Flash.h"
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

#define ANGLE_A                  51.34  
#define ANGLE_B                  38.65

#define FORWARD                 4.5
#define TURN                    3.5
uint16 delay_time = 0;
uint8 led_state = 0;

extern uint8_t FOCcounter;
extern uint8_t IMUcounter;
extern uint16_t BlueTooth;
extern uint16_t KeyCounter;
extern uint16_t OLEDCounter; 
extern uint16_t DeviceCounter;
extern uint16_t FlashCounter;
soft_iic_info_struct AS561,AS562;

Position_Data POS = {
  .X = 0,
  .Y = 0,
  .Yaw = 0,
  
};
Speed_Data SPEED_record = {
  .Speed = 0,
  .Yaw = 0,
};

Speed_Data SPEED_out = {
  .Speed = 0,
  .Yaw = 0,
};




float Roll,Yaw,Pitch,X_gyro = 0,Last_Roll = 0;

float BaseAngle = 0;

float current_speed,L_Speed,R_Speed;

uint8 data_buffer[32];
uint8 data_len;
void send_three_float(float f1, float f2, float f3);

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
int8_t Error = 0;
uint8_t S2 = 0,S3 = 0,cycle = 0;
uint16_t temp = 0;
int main(void)
{
    
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                          // 调试串口信息初始化
    oled_init();
    wireless_uart_init();
    key_init(10);
    Menu_Init();
    LED_Init();
    Buzzer_Init();
    Infrared_Init();
    flash_init();
    
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
          if(Mission == 5){
            data_len = (uint8)wireless_uart_read_buffer(data_buffer,32);             // 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
            Bluetooth_receive(data_buffer,data_len,&turn_speed,&forward_speed);
          }
           //send_three_float(SPEED_record.Speed,SPEED_record.Yaw,1.0);} else {
            
        }
        if(DeviceCounter >= 100){
          DeviceCounter = 0;
          Device_Scanner();
        }
        if(FlashCounter >= 1000){
          FlashCounter = 0;
          SPEED_record.Speed = current_speed;
          SPEED_record.Yaw = Yaw;
          
          Flash_scanner(Save_State,SPEED_record);
          if(Mission == 4 && !Save_State){
            buffer_read_flash(&SPEED_out);
          }
        }

        if(IMUcounter >= 10){
            IMUcounter = 0;
            
            current_speed = (L_Speed + R_Speed) / 2;

            if(Mission == 5){
              KalmanFilter1D_Speed_Update(&_FORWARD,(forward_speed / 15.0));
              KalmanFilter1D_Speed_Update(&_TURN,(turn_speed / 40.0));
            
              _Speed.Target = _FORWARD.Speed_Hat;
              _Dir.Target = _TURN.Speed_Hat;    
            }
            if(Mission == 1){
              Position_cal(&POS,Yaw,current_speed * 3 / 7,0.001);
              _Pos.Actual = POS.Y;
              PID_Update_Pos(&_Pos);
              _Speed.Target = _Pos.Out;
            }
            if(Mission == 2){
              switch (S2){
                case 0:
                  _Speed.Target = FORWARD;
                  if(temp <= 100){
                    temp ++;
                  } else {
                    temp = 0;
                    S2 ++;
                  }
                  break;
                case 1:
                  _Speed.Target = FORWARD;
                  _Yaw.Actual = Yaw;
                  _Yaw.Target = 0;
                  PID_Update_Pos(&_Yaw);
                  _Dir.Target = _Yaw.Out;
                  Error = Infrared_ErrorGet(Infrared);
                  if(Error){
                    Device_Set();
                    S2 ++;
                  }
                  break;
                case 2:
                    S2 ++;
                  break;
                case 3:
                  _Speed.Target = TURN;
                  Error = Infrared_ErrorGet(Infrared);
                  _Dir.Target = Error * 1.2;
                  if(temp <= 500){
                    temp ++;
                  } else {
                    if(Yaw >= 155 || Yaw <= -155){
                      temp = 0;
                      Device_Set();
                      S2 ++;
                    }
                  }
                  
                  break;
                case 4:
                  _Speed.Target = 0;
                  _Yaw.Target = 179.5;
                  adjust_target_angle(&_Yaw.Target, _Yaw.Actual);
                  _Yaw.Actual = Yaw;
                  PID_Update_Pos(&_Yaw);
                  _Dir.Target = _Yaw.Out;
                  if(fabs(_Yaw.Target - _Yaw.Actual) <= 2){
                    S2 ++;
                  } 
                  break;
                case 5:
                  _Speed.Target = FORWARD;
                  _Yaw.Actual = Yaw;
                  _Yaw.Target = Yaw > 0 ? 179 : -179;
                  PID_Update_Pos(&_Yaw);
                  _Dir.Target = _Yaw.Out;
                  Error = Infrared_ErrorGet(Infrared);
                  if(temp <= 500){
                    temp ++;
                  } else {
                    if(Error){
                       temp = 0;
                       Device_Set();
                       S2 ++;
                    }
                  }
                  break;
                case 6:
                  _Speed.Target = TURN;
                  Error = Infrared_ErrorGet(Infrared);
                  _Dir.Target = Error * 1.2;
                  if(Yaw <= 10 && Yaw >= -10){
                    Device_Set();
                    S2 ++;
                  }
                  break;
                case 7:
                  Mission = 0;
                  S2 = 0;
                  _Dir.Target = 0;
                  _Speed.Target = 0;
                  break;
                
              }
            }
            if(Mission == 3){
             if(cycle < 4){
              switch (S3){
                case 0:
                  _Speed.Target = 0;
                  _Yaw.Target = ANGLE_B - 1;
                  _Yaw.Actual = Yaw;
                  PID_Update_Pos(&_Yaw);
                  _Dir.Target = _Yaw.Out;
                  if(fabs(_Yaw.Target - _Yaw.Actual) <= 2){
                    S3 ++;
                  } 
                  break;
                case 1:
                  _Speed.Target = FORWARD;
                  _Yaw.Actual = Yaw;
                  _Yaw.Target = ANGLE_B - 1;
                  PID_Update_Pos(&_Yaw);
                  _Dir.Target = _Yaw.Out;
                  Error = Infrared_ErrorGet(Infrared);
                  if(temp <= 500){
                    temp ++;
                  } else {
                    if(Error){
                      temp = 0;
                      Device_Set();
                      S3 ++;
                    }
                  }
                  break;
                case 2:
                  _Speed.Target = 0;
                  _Yaw.Target = -20;
                  _Yaw.Actual = Yaw;
                  PID_Update_Pos(&_Yaw);
                  _Dir.Target = _Yaw.Out;
                  if(fabs(_Yaw.Target - _Yaw.Actual) <= 2){
                    S3 ++;
                  } 
                  break;
                case 3:
                  _Speed.Target = TURN;
                  Error = Infrared_ErrorGet(Infrared);
                  _Dir.Target = Error * 1.3;
                  if(temp <= 500){
                    temp ++;
                  } else {
                    if(Yaw >= 170 || Yaw <= -170){
                      temp = 0;
                      Device_Set();
                      S3 ++;
                    }
                  }
                  
                  break;
               case 4:
                  _Speed.Target = 0;
                  _Yaw.Target = 179 - ANGLE_B ;
                  _Yaw.Actual = Yaw;
                  adjust_target_angle(&_Yaw.Target, _Yaw.Actual);
                  PID_Update_Pos(&_Yaw);
                  _Dir.Target = _Yaw.Out;
                  if(fabs(_Yaw.Target - _Yaw.Actual) <= 2){
                    S3 ++;
                  } 
                  break; 
               case 5:
                  _Speed.Target = FORWARD;
                  _Yaw.Actual = Yaw;
                  _Yaw.Target = 179 - ANGLE_B;
                  PID_Update_Pos(&_Yaw);
                  _Dir.Target = _Yaw.Out;
                  Error = Infrared_ErrorGet(Infrared);
                  if(temp <= 500){
                    temp ++;
                  } else {
                    if(Error){
                      temp = 0;
                      Device_Set();
                      S3 ++;
                    }   
                  }
                  
                  break;
                case 6:
                  _Speed.Target = 0;
                  _Yaw.Target = -165;
                  _Yaw.Actual = Yaw;
                  adjust_target_angle(&_Yaw.Target, _Yaw.Actual);
                  PID_Update_Pos(&_Yaw);
                  _Dir.Target = _Yaw.Out;
                  if(fabs(_Yaw.Target - _Yaw.Actual) <= 2){
                    S3 ++;
                  } 
                  break;
                case 7:
                  _Speed.Target = TURN;
                  Error = Infrared_ErrorGet(Infrared);
                  _Dir.Target = Error * 1.3;
                  if(temp <= 500){
                    temp ++;
                  } else {
                    if(Yaw <= 15 && Yaw >= -15){
                      Device_Set();
                      S3 ++;
                      temp = 0;
                      _Speed.Target = 0.5;
                    }
                  }
                  
                  break; 
                 case 8:
                  _Speed.Target = 0.5;
                  _Yaw.Target = 0;
                  _Yaw.Actual = Yaw;
                  adjust_target_angle(&_Yaw.Target, _Yaw.Actual);
                  PID_Update_Pos(&_Yaw);
                  _Dir.Target = _Yaw.Out;
                  if(fabs(_Yaw.Target - _Yaw.Actual) <= 2){
                    S3 = 0;
                    cycle ++;
                  } 
                  break;
              }
             } else {
               S3 = 0;
               _Dir.Target = 0;
               _Speed.Target = 0;
               Mission = 0;
             }
            }
            if(Mission == 4){
              if(SPEED_out.Yaw != 200 ){
                _Speed.Target = SPEED_out.Speed * 0.7;
                _Yaw.Actual = Yaw;
                _Yaw.Target = SPEED_out.Yaw;
                PID_Update_Pos(&_Yaw);
                _Dir.Target = _Yaw.Out;
               
              }else{
                Mission = 0;
                SPEED_out.Speed= 0;
                SPEED_out.Yaw= 0;
                _Yaw.Target = 0;
                _Speed.Target = 0;
              }
              
            }
            
           _Speed.Actual = current_speed;
           PID_Update_Pos(&_Speed);
           
           
           _Dir.Actual = L_Speed - R_Speed;
           _Dir.Actual = _Dir.Actual < (_Dir.Target - 1) ? _Dir.Actual : (_Dir.Actual > (_Dir.Target + 1) ? _Dir.Actual : _Dir.Target);
           PID_Update_Pos(&_Dir);
           if(Save_State){
             _Dir.Out = 0;
           }

           _Angle.Target = BaseAngle - _Speed.Out;
           _Angle.Actual = Roll;
           PID_Update_Pos(&_Angle);
           
           _Angle_Speed.Target =  0.1 * _Angle.Out + 0.9 * _Angle_Speed.Target ;
           _Angle_Speed.Actual = X_gyro;
           PID_Update_Pos(&_Angle_Speed);
           oled_show_float(64,5,Yaw,3,2);
           oled_show_float(64,6,_Dir.Out,3,2);
           oled_show_uint(64,7,Save_State,1);
           
           if(Save_State){} else {
            } 
           //printf("%.2f,%.2f\r\n",SPEED_record.Speed,SPEED_record.Yaw);  
           //printf("%.2f,%.2f\r\n",SPEED_out.Speed,SPEED_out.Yaw);
           system_delay_us(2100); 
           
         
           
        }
        
        if(FOCcounter >= 10){
          FOCcounter = 0;

          Motor_SpeedSet(&M1,1,(_Angle_Speed.Out + 1.6 * _Dir.Out ) * 7.5 );
          Motor_SpeedSet(&M2,2,(_Angle_Speed.Out - 1.6 * _Dir.Out ) * 7.5 );
         
          //printf("%.2f,%.2f,%.2f\r\n",Roll,Yaw,Pitch);
          system_delay_us(1400); 
        }
        
        
        
        
        led_state = !led_state;
        system_delay_ms(delay_time);
        gpio_set_level(LED1, led_state);
        
    }
}

void send_three_float(float f1, float f2, float f3) {
    memset(data_buffer, 0, 32);
    int len = snprintf((char*)data_buffer, 32, "[plot,%.2f,%.2f,%.2f]", f1, f2, f3);
    if (len <= 0 || len >= 32) {
        snprintf((char*)data_buffer, 32, "[plot:error,error,error]");
        len = strlen((char*)data_buffer);
    }
    wireless_uart_send_buffer(data_buffer, len);
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
