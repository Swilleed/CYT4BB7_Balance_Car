#ifndef __KF_H
#define __KF_H
typedef struct {
    float Angle_Hat;  
    float P;      
    float Q;      
    float R;      
} KalmanFilter1D;
void KalmanFilter1D_Update(KalmanFilter1D *kf_1d,float dAngle, float Angle_Actual, float dt);

#endif
