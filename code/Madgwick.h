#ifndef __MADGWICK_H
#define __MADGWICK_H

typedef struct {
    float q0, q1, q2, q3;  // 四元数：q0(实部), q1/q2/q3(虚部)，表示三维姿态
} MadgwickFilter;
void Madgwick_Update(MadgwickFilter *mf, float gyro[], float accel[], float dt) ;

float Madgwick_QuatToRoll(MadgwickFilter *mf);
float Madgwick_QuatToYaw(MadgwickFilter *mf);
float Madgwick_QuatToPitch(MadgwickFilter *mf);
#endif
