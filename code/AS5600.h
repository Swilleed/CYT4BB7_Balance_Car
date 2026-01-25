#ifndef __AS5600_H
#define __AS5600_h
  void AS5600_Init(soft_iic_info_struct *AS560,uint8_t N);
  float AS5600_Read(soft_iic_info_struct *AS560);
  float AS5600_Speed_Read(float pre_Angle,float actual_Angle,float Pre_Speed,float Act_Speed,float ms);
  
#endif
