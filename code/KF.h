#ifndef __KF_H
#define __KF_H
typedef struct {
    float Angle_Hat;  
    float P;      
    float Q;      
    float R;      
} KalmanFilter1D;
void KalmanFilter1D_Update(KalmanFilter1D *kf_1d,float dAngle, float Angle_Actual, float dt);
typedef struct {
    float Speed_Hat;  
    float P;          
    float Q;         
    float R;          
} KalmanFilter1D_Speed;
void KalmanFilter1D_Speed_Update(KalmanFilter1D_Speed *kf_speed, float Speed_Obs);
#endif
