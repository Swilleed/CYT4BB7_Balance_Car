#ifndef __POSITION_H_
#define __POSITION_H_
typedef struct{
  float X;
  float Y;
  float Yaw;

}Position_Data;



void Position_cal(Position_Data *pos,float Yaw,float current_speed,float dt);
#endif