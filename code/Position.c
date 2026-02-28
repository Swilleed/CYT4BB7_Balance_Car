#include "zf_common_typedef.h"
#include <math.h>
#include "Position.h"

float last_speed = 0;

void Position_cal(Position_Data *pos,float Yaw,float current_speed,float dt)
{
  float Yaw_rad = Yaw * 3.141592f / 180;
  float mid_speed = (last_speed + current_speed) / 2;
  float k1 = last_speed * dt,k2 = mid_speed * dt,k3 = mid_speed * dt,k4 = current_speed * dt;
  float s = (k1 + 2 * k2 + 2 * k3 + k4) / 6;
  pos -> X += s * sinf(Yaw_rad);
  pos -> Y += s * cosf(Yaw_rad);
  pos -> Yaw = Yaw;
  
}

