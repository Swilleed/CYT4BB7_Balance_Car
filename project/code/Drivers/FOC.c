#include "zf_common_headfile.h"
#include "math.h"
#include "FOC.h"

#ifndef PI
#define PI 3.14159f
#endif

#define VDC 12.0f

/**
 * 初始化 FOC 三相 PWM 输出通道
 */
void FOC_init(void)
{
  pwm_init(TCPWM_CH30_P10_2, 20000, 0);
  pwm_init(TCPWM_CH31_P10_3, 20000, 0);
  pwm_init(TCPWM_CH32_P10_4, 20000, 0);
  pwm_init(TCPWM_CH59_P17_2, 20000, 0);
  pwm_init(TCPWM_CH58_P17_3, 20000, 0);
  pwm_init(TCPWM_CH57_P17_4, 20000, 0);
}

/**
 * 简化 FOC 电压变换
 * @param ud d 轴电压分量（一般可置 0）
 * @param uq q 轴电压分量（决定转矩）
 * @param elec_angle 电角度（度）
 * @param u_u U 相占空比输出（0~1）
 * @param u_v V 相占空比输出（0~1）
 * @param u_w W 相占空比输出（0~1）
 */
void SimpleFOC(float ud, float uq, float elec_angle,
         float *u_u, float *u_v, float *u_w)
{
  float theta = elec_angle * PI / 180.0f;
  float cos_theta = cosf(theta);
  float sin_theta = sinf(theta);

  // 1) 逆 Park 变换：d/q -> α/β
  float u_alpha = ud * cos_theta - uq * sin_theta;
  float u_beta = ud * sin_theta + uq * cos_theta;

  // 2) 逆 Clarke 变换：α/β -> U/V/W
  *u_u = u_alpha;
  *u_v = (-0.5f * u_alpha) + (sqrtf(3.0f) / 2.0f * u_beta);
  *u_w = (-0.5f * u_alpha) - (sqrtf(3.0f) / 2.0f * u_beta);

  // 3) 归一化到 0~1 占空比
  float max_phase_voltage = VDC / 2.0f;
  float scale = 1.0f / max_phase_voltage;
  *u_u = *u_u * scale;
  *u_v = *u_v * scale;
  *u_w = *u_w * scale;

  // 4) 如果出现负值，整体平移到非负区间
  float min_val = fminf(*u_u, fminf(*u_v, *u_w));
  if (min_val < 0.0f)
  {
    *u_u -= min_val;
    *u_v -= min_val;
    *u_w -= min_val;
  }

  // 5) 最终限幅 0~1
  *u_u = (*u_u > 1.0f) ? 1.0f : (*u_u < 0.0f ? 0.0f : *u_u);
  *u_v = (*u_v > 1.0f) ? 1.0f : (*u_v < 0.0f ? 0.0f : *u_v);
  *u_w = (*u_w > 1.0f) ? 1.0f : (*u_w < 0.0f ? 0.0f : *u_w);
}

/**
 * 机械角转电角度
 * @param Mech_Angle 机械角（度）
 * @return 电角度（度，0~360）
 */
float Elec_Angle_Set(float Mech_Angle)
{
  return fmodf((Mech_Angle) * 7, 360.0f);
}

/**
 * 将 0~1 占空比写入对应三相 PWM 通道
 * @param duty1 相位 1 占空比
 * @param duty2 相位 2 占空比
 * @param duty3 相位 3 占空比
 * @param type 电机编号（1 或 2）
 */
void FOC_pwm_set_duty(float duty1, float duty2, float duty3, uint8_t type)
{
  if (type == 1)
  {
    pwm_set_duty(TCPWM_CH30_P10_2, (uint32)(10000 * duty1));
    pwm_set_duty(TCPWM_CH31_P10_3, (uint32)(10000 * duty2));
    pwm_set_duty(TCPWM_CH32_P10_4, (uint32)(10000 * duty3));
  }

  if (type == 2)
  {
    pwm_set_duty(TCPWM_CH59_P17_2, (uint32)(10000 * duty1));
    pwm_set_duty(TCPWM_CH58_P17_3, (uint32)(10000 * duty2));
    pwm_set_duty(TCPWM_CH57_P17_4, (uint32)(10000 * duty3));
  }
}




