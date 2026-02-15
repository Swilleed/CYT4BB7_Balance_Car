#include "zf_common_headfile.h"
#include "math.h"
#include "FOC.h"
#define PI 3.14159f
#define VDC 12.0f
// ==================== 核心变换函数 ====================
// 功能：输入ud、uq、电角度，输出三相PWM占空比
// 参数：
//   ud       - d轴电压指令（通常置0，仅uq控制转速）
//   uq       - q轴电压指令（速度环输出，决定PWM占空比大小）
//   elec_angle - 电角度（°，范围0~360，由AS5600计算得到）
//   u_u/u_v/u_w - 输出的三相PWM占空比（0~1.0 或 0~100，按需调整）


void FOC_init(void)
{
   pwm_init(TCPWM_CH30_P10_2, 20000, 0);
   pwm_init(TCPWM_CH31_P10_3, 20000, 0);
   pwm_init(TCPWM_CH32_P10_4, 20000, 0);
   pwm_init(TCPWM_CH59_P17_2, 20000, 0);
   pwm_init(TCPWM_CH58_P17_3, 20000, 0);
   pwm_init(TCPWM_CH57_P17_4, 20000, 0);
  
}

void SimpleFOC(float ud, float uq, float elec_angle, 
                    float *u_u, float *u_v, float *u_w)
{
    // 1. 电角度转换为弧度
    float theta = elec_angle * PI / 180.0f;
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    // 2. 帕克逆变换（IPark）：d/q → alpha/beta
    float u_alpha = ud * cos_theta - uq * sin_theta;
    float u_beta  = ud * sin_theta + uq * cos_theta;

    // 3. 克拉克逆变换（IClark）：alpha/beta → U/V/W
    *u_u = u_alpha;
    *u_v = (-0.5f * u_alpha) + (sqrtf(3.0f) / 2.0f * u_beta);
    *u_w = (-0.5f * u_alpha) - (sqrtf(3.0f) / 2.0f * u_beta);

    // 4. 基于母线电压的有效归一化（核心修复）
    // 步骤1：计算uq对应的最大相电压（FOC约束：相电压≤VDC/2）
    float max_phase_voltage = VDC / 2.0f;
    // 步骤2：将d/q电压归一化为占空比（0~1），直接映射到母线电压
    float scale = 1.0f / max_phase_voltage; // 电压→占空比转换系数
    
    // 步骤3：缩放三相电压到0~1占空比（关键：无额外偏移，保留有效矢量）
    *u_u = *u_u * scale;
    *u_v = *u_v * scale;
    *u_w = *u_w * scale;

    // 步骤4：偏置调整（保证占空比非负，这是硬件必需的）
    // 计算最小占空比，整体偏移使最小值为0
    float min_val = fminf(*u_u, fminf(*u_v, *u_w));
    if (min_val < 0.0f) {
        *u_u -= min_val;
        *u_v -= min_val;
        *u_w -= min_val;
    }

    // 最终限幅（严格限制在0~1，适配硬件）
    *u_u = (*u_u > 1.0f) ? 1.0f : (*u_u < 0.0f ? 0.0f : *u_u);
    *u_v = (*u_v > 1.0f) ? 1.0f : (*u_v < 0.0f ? 0.0f : *u_v);
    *u_w = (*u_w > 1.0f) ? 1.0f : (*u_w < 0.0f ? 0.0f : *u_w);

}

// ==================== 调用示例（适配逐飞库PWM输出） ====================
// 假设你已初始化好三相PWM（参考之前的PWM配置）
float Elec_Angle_Set(float Mech_Angle){
    
    return fmodf((Mech_Angle)* 7,360.0); 
}
void FOC_pwm_set_duty(float duty1,float duty2,float duty3,uint8_t type)
{
  if(type == 1){
    pwm_set_duty(TCPWM_CH30_P10_2,(uint32)(10000 * duty1));
    pwm_set_duty(TCPWM_CH31_P10_3,(uint32)(10000 * duty2));
    pwm_set_duty(TCPWM_CH32_P10_4,(uint32)(10000 * duty3));
  }
  if(type == 2){
    pwm_set_duty(TCPWM_CH59_P17_2,(uint32)(10000 * duty1));
    pwm_set_duty(TCPWM_CH58_P17_3,(uint32)(10000 * duty2));
    pwm_set_duty(TCPWM_CH57_P17_4,(uint32)(10000 * duty3));
  }
}
  
  
 

