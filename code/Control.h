#ifndef __CONTROL_H_
#define __CONTROL_H_
extern KalmanFilter1D_Speed X_GYRO;
extern KalmanFilter1D_Speed _FORWARD;
extern KalmanFilter1D_Speed _TURN;
extern PID_TypeDef M1;
extern PID_TypeDef M2;
extern PID_TypeDef _Angle_Speed;
extern PID_TypeDef _Angle;
extern PID_TypeDef _Speed;
extern PID_TypeDef _Dir;
extern PID_TypeDef _Pos;
extern PID_TypeDef _Yaw;
void adjust_target_angle(float *target, float actual);
void LED_Init(void);
void LED_ON(void);
void LED_OFF(void);
void Buzzer_Init(void);
void Buzzer_ON(void);
void Buzzer_OFF(void);
void Device_Scanner(void);
void Device_Set(void);
#endif
