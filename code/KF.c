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


