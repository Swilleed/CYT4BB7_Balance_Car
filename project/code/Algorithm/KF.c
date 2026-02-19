#include "zf_common_typedef.h"
#include "KF.h"

KalmanFilter1D KF_1D = {
    .Angle_Hat = 0.,  
    .P = 1.0,     
    .Q = 0.001,   
    .R = 0.05     
};

void KalmanFilter1D_Update(KalmanFilter1D *kf_1d,float dAngle, float Angle_Actual, float dt){
	float Angle_Hat_pre;
	Angle_Hat_pre = kf_1d->Angle_Hat + dt * dAngle;
	float P_pre;
	P_pre = kf_1d->Q + kf_1d->P;
	float K;
	K = P_pre / (P_pre + kf_1d->R);
	kf_1d->Angle_Hat = Angle_Hat_pre + K * (Angle_Actual - Angle_Hat_pre);
	kf_1d->P = (1 - K) * P_pre;
}



void KalmanFilter1D_Speed_Update(KalmanFilter1D_Speed *kf_speed, float Speed_Obs) {
   
    float Speed_Hat_pre = kf_speed->Speed_Hat;  
    float P_pre = kf_speed->P + kf_speed->Q;    
    
    float K = P_pre / (P_pre + kf_speed->R);   
    
    kf_speed->Speed_Hat = Speed_Hat_pre + K * (Speed_Obs - Speed_Hat_pre);
    
    kf_speed->P = (1 - K) * P_pre;
}

