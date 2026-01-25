#include "math.h"

#define PI 3.1415926535f

// ==================== 核心变换函数 ====================
// 功能：输入ud、uq、电角度，输出三相PWM占空比
// 参数：
//   ud       - d轴电压指令（通常置0，仅uq控制转速）
//   uq       - q轴电压指令（速度环输出，决定PWM占空比大小）
//   elec_angle - 电角度（°，范围0~360，由AS5600计算得到）
//   u_u/u_v/u_w - 输出的三相PWM占空比（0~1.0 或 0~100，按需调整）
void SimpleFOC(float ud, float uq, float elec_angle, 
                    float *u_u, float *u_v, float *u_w)
{
    // 1. 电角度转换为弧度（C语言三角函数需弧度）
    float theta = elec_angle * PI / 180.0f;
    // 计算角度的正/余弦值（核心）
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);

    // 2. 帕克逆变换（IPark）：d/q → alpha/beta
    float u_alpha = ud * cos_theta - uq * sin_theta;
    float u_beta  = ud * sin_theta + uq * cos_theta;

    // 3. 克拉克逆变换（IClark）：alpha/beta → U/V/W
    // 基础公式（无中点偏移，适配三相无刷电机）
    *u_u = u_alpha;
    *u_v = (-0.5f * u_alpha) + (sqrtf(3.0f) / 2.0f * u_beta);
    *u_w = (-0.5f * u_alpha) - (sqrtf(3.0f) / 2.0f * u_beta);

    // 4. 归一化处理（可选，将占空比限制在0~1.0，避免超量程）
    // 步骤1：找到三相中的最大值和最小值
    float max_val = fmaxf(*u_u, fmaxf(*u_v, *u_w));
    float min_val = fminf(*u_u, fminf(*u_v, *u_w));
    // 步骤2：偏移量（使最小值为0，保证占空比非负）
    float offset = (max_val + min_val) / 2.0f;
    // 步骤3：归一化到0~1.0
    *u_u = (*u_u - offset) / (max_val - min_val);
    *u_v = (*u_v - offset) / (max_val - min_val);
    *u_w = (*u_w - offset) / (max_val - min_val);

    // 可选：转换为0~100%占空比（适配PWM函数的百分比输入）
    *u_u *= 100.0f;
    *u_v *= 100.0f;
    *u_w *= 100.0f;

    // 最终限幅（防止PWM占空比超出硬件范围）
    *u_u = (*u_u > 100.0f) ? 100.0f : (*u_u < 0.0f ? 0.0f : *u_u);
    *u_v = (*u_v > 100.0f) ? 100.0f : (*u_v < 0.0f ? 0.0f : *u_v);
    *u_w = (*u_w > 100.0f) ? 100.0f : (*u_w < 0.0f ? 0.0f : *u_w);
}

// ==================== 调用示例（适配逐飞库PWM输出） ====================
// 假设你已初始化好三相PWM（参考之前的PWM配置）
float Elec_Angle_Set(float Mech_Angle){
    
    return fmodf(Mech_Angle * 7,360.0); 
}
  
  
 

