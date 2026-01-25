#include "zf_common_headfile.h"
#define PI 3.14159f
#define R 0.0231f

uint8 Data[2] = {0};
uint16 Angle;
float Ac;

void AS5600_Init(soft_iic_info_struct *AS560,uint8_t N)
{
  if(N == 1){
    soft_iic_init(AS560,0x36,50,P05_0,P05_1);
  } else if(N == 2){
    soft_iic_init(AS560,0x36,50,P05_2,P05_3);
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

float AS5600_Speed_Read(float pre_Angle,float actual_Angle,float Pre_Speed,float Act_Speed,float ms){
    float delta = pre_Angle - actual_Angle,rate = 0.9;
    
    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }
    Pre_Speed = Act_Speed;
    Act_Speed = ( delta / 360.0 )  * 2 * PI * R * 1000 / ms;

    return Act_Speed * (1 - rate) + Pre_Speed * rate;
}





