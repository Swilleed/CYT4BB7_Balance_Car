#include "zf_common_headfile.h"
#include "AS5600.h"
#define PI 3.14159f
#define R 0.0231f
#define SPEED_WIN 20   // 8 × 10ms = 80ms

uint8 Data[2] = {0};
uint16 Angle;
float Ac;

void AS5600_Init(soft_iic_info_struct *AS560,uint8_t N)
{
  if(N == 1){
    soft_iic_init(AS560,0x36,23,P05_0,P05_1);
  } else if(N == 2){
    soft_iic_init(AS560,0x36,23,P05_2,P05_3);
  }
  
}

float AS5600_Read(soft_iic_info_struct *AS560){
  soft_iic_read_8bit_registers(AS560,0x0E,Data,2);
  Angle = Data[0] & 0x0F;
  Angle <<= 8;
  Angle = Angle | Data[1];
  Ac = Angle * 0.08789;
  
  return Ac;
}




float angle_buf[3][SPEED_WIN] = {0};
uint8 idx[3] = {0};
float dt = 0.0005;

float Speed_Update1(float Mech_Angle)
{
    angle_buf[1][idx[1]] = Mech_Angle;
    idx[1] = (idx[1] + 1) % SPEED_WIN;

    float diff = angle_buf[1][idx[1]] - angle_buf[1][(idx[1] + 1) % SPEED_WIN];

    // 回绕修正
    if(diff > 180) diff -= 360;
    if(diff < -180) diff += 360;

    return diff * R / (SPEED_WIN * dt);
}

float Speed_Update2(float Mech_Angle)
{
    angle_buf[2][idx[2]] = Mech_Angle;
    idx[2] = (idx[2] + 1) % SPEED_WIN;

    float diff = angle_buf[2][idx[2]] - angle_buf[2][(idx[2] + 1) % SPEED_WIN];

    // 回绕修正
    if(diff > 180) diff -= 360;
    if(diff < -180) diff += 360;

    return diff * R / (SPEED_WIN * dt);
}





