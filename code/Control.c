#include "PID.h"
#include "KF.h"
#include "zf_common_headfile.h"

#include "Flash.h"
#include "Menu.h"
extern Menu menu[6];
extern float FORWARD,TURN;

KalmanFilter1D_Speed X_GYRO = {
    .Speed_Hat = 0.0f,  
    .P = 1.0f,          
    .Q = 0.0005f,       
    .R = 0.5f         
};
KalmanFilter1D_Speed _FORWARD = {
    .Speed_Hat = 0.0f,  
    .P = 1.0f,          
    .Q = 0.0005f,       
    .R = 0.5f         
};
KalmanFilter1D_Speed _TURN = {
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
	
	.Kp = 0.08,
	.Ki = 0,
	.Kd = 5.50,
	
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
	.Kd = 45.0,
	
	.OutMax = 6,
	.OutMin = -6,
	
	.Target = 0,
	.Out = 0,
	
};

PID_TypeDef _Yaw = {
	
	.error0 = 0,
	.error1 = 0,
	.error2 = 0,
	
	.Kp = 6.00,
	.Ki = 0.00,
	.Kd = 250,
	
	.OutMax = 40,
	.OutMin = -40,
	
	.Target = 0,
	.Out = 0,
	
};

void adjust_target_angle(float *target, float actual) {
    
    float angle_diff = *target - actual;

    if (angle_diff > 180) {
        *target -= 360;  
    } else if (angle_diff < -180) {
        *target += 360;  
    }
}

void LED_Init(void){
  gpio_init(P06_5, GPO, 0, GPO_PUSH_PULL);
}
void LED_ON(void){
  gpio_set_level(P06_5, 1);
}
void LED_OFF(void){
  gpio_set_level(P06_5, 0); 
}
void Buzzer_Init(void){
  gpio_init(P06_7, GPO, 1, GPO_PUSH_PULL);
}
void Buzzer_ON(void){
  gpio_set_level(P06_7, 0);
}
void Buzzer_OFF(void){
  gpio_set_level(P06_7, 1); 
}
uint8_t Device_count = 10;
void Device_Scanner(void)
{
  if(Device_count <= 10){
    LED_ON();
    Buzzer_ON();
    Device_count ++;
  } else {
    LED_OFF();
    Buzzer_OFF();
  }
}
void Device_Set(void)
{
  Device_count = 0;
}


void Read_Dir_and_Basic(PID_TypeDef *PID,float *n1,float * n2)
{
  uint32_t temp1[3],temp2[2];
  
  flash_read_page(0, 90, temp1, 3);
  flash_read_page(0, 91, temp2, 2);
  
  menu[3].item[0].value = temp1[0] / 100.0;
  menu[3].item[1].value = temp1[1] / 100.0;
  menu[3].item[2].value = temp1[2] / 100.0;
  menu[2].item[0].value = temp2[0] / 100.0;
  menu[2].item[1].value = temp2[1] / 100.0;
  
  PID->Kp = temp1[0] / 100.0;
  PID->Ki = temp1[1] / 100.0;
  PID->Kd = temp1[2] / 100.0;
  
  *n1 = temp2[0] / 100.0;
  *n2 = temp2[1] / 100.0;
}
