#include "zf_common_headfile.h"
#include "FOC.h"
#include "AS5600.h"
#include "math.h"
#include "PID.h"
#include "KF.h"
#include "Motor.h"
float Mech_Angle[3], Elec_Angle[3], Obs_Speed[3] = {0};
float duty1[3] = {0}, duty2[3] = {0}, duty3[3] = {0};
float uq[3];
extern soft_iic_info_struct AS561, AS562;
float L_Speed, R_Speed;
KalmanFilter1D_Speed KF_Speed1 = {
    .Speed_Hat = 0.0f,
    .P = 1.0f,
    .Q = 0.001f,
    .R = 0.1f};
KalmanFilter1D_Speed KF_Speed2 = {
    .Speed_Hat = 0.0f,
    .P = 1.0f,
    .Q = 0.001f,
    .R = 0.1f};
void Motor_SpeedSet(PID_TypeDef *M, uint8_t type, float Speed)
{
  M->Target = Speed;
  if (type == 1)
  {
    Mech_Angle[1] = AS5600_Read(&AS561);
    Obs_Speed[1] = Speed_Update1(Mech_Angle[1]);
    KalmanFilter1D_Speed_Update(&KF_Speed1, Obs_Speed[1]);
    M->Actual = KF_Speed1.Speed_Hat;
    L_Speed = M->Actual;
    PID_Update_Add(M, 0.0005);
    uq[1] = M->Out;
    Elec_Angle[1] = Elec_Angle_Set(Mech_Angle[1]);
    SimpleFOC(0, uq[1], Elec_Angle[1], &duty1[1], &duty2[1], &duty3[1]);
    FOC_pwm_set_duty(duty1[1], duty2[1], duty3[1], 1);
  }
  else if (type == 2)
  {
    Mech_Angle[2] = AS5600_Read(&AS562);
    Obs_Speed[2] = Speed_Update2(Mech_Angle[2]);
    KalmanFilter1D_Speed_Update(&KF_Speed2, Obs_Speed[2]);
    M->Actual = -KF_Speed2.Speed_Hat;
    R_Speed = M->Actual;
    PID_Update_Add(M, 0.0005);
    uq[2] = M->Out;
    Elec_Angle[2] = Elec_Angle_Set(Mech_Angle[2]);
    SimpleFOC(0, uq[2], Elec_Angle[2], &duty1[2], &duty2[2], &duty3[2]);
    FOC_pwm_set_duty(duty1[2], duty2[2], duty3[2], 2);
  }
}
