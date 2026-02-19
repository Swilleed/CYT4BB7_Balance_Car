#ifndef __AS5600_H
#define __AS5600_h
  void AS5600_Init(soft_iic_info_struct *AS560,uint8_t N);
  float AS5600_Read(soft_iic_info_struct *AS560);
  float Speed_Update1(float Mech_Angle);
  float Speed_Update2(float Mech_Angle);
#endif
