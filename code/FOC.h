#ifndef __FOC_H
#define __FOC_H

void SimpleFOC(float ud, float uq, float elec_angle, float *u_u, float *u_v, float *u_w);
float Elec_Angle_Set(float Mech_Angle);
void FOC_init(void);
void FOC_pwm_set_duty(float duty1,float duty2,float duty3,uint8_t type);

#endif
