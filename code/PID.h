#ifndef __PID_H
#define __PID_H

typedef struct {
	float Target;
	float Actual;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float error0;
	float error1;
	float error2;
	float errorInt;
	
	float OutMax;
	float OutMin;
	
} PID_TypeDef;

void PID_Update_Add(PID_TypeDef *p,float dt);
void PID_Update_Pos(PID_TypeDef *p);

#endif

