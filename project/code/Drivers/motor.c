#include "zf_common_headfile.h"
#include "motor.h"
#include "pid_controller.h"

Motor_TypeDef motorLeft;
Motor_TypeDef motorRight;

/**
 * 初始化电机结构体及其PID参数
 */
void Motor_Init(float kp, float ki, float kd)
{
    PID_Init(&motorLeft.pid, kp, ki, kd);
    PID_Init(&motorRight.pid, kp, ki, kd);

    encoder_quad_init((encoder_index_enum)MOTOR_LEFT_ENCODER_INDEX,
                      (encoder_channel1_enum)MOTOR_LEFT_ENCODER_PIN_A,
                      (encoder_channel2_enum)MOTOR_LEFT_ENCODER_PIN_B);
    encoder_quad_init((encoder_index_enum)MOTOR_RIGHT_ENCODER_INDEX,
                      (encoder_channel1_enum)MOTOR_RIGHT_ENCODER_PIN_A,
                      (encoder_channel2_enum)MOTOR_RIGHT_ENCODER_PIN_B);

    motorLeft.speed = 0.0f;
    motorLeft.targetSpeed = 0.0f;
    motorLeft.encoderCount = 0;
    motorLeft.totalCount = 0;
    motorLeft.pwmChannel = MOTOR_LEFT_PWM_CHANNEL;
    motorLeft.encoderPinA = (uint8_t)MOTOR_LEFT_ENCODER_PIN_A;
    motorLeft.encoderPinB = (uint8_t)MOTOR_LEFT_ENCODER_PIN_B;
    motorLeft.encoderTimer = (uint8_t)MOTOR_LEFT_ENCODER_TIMER;
    motorLeft.directionPin_P = MOTOR_LEFT_DIR_PIN_P;
    motorLeft.directionPin_N = MOTOR_LEFT_DIR_PIN_N;
    motorRight.speed = 0.0f;
    motorRight.targetSpeed = 0.0f;
    motorRight.encoderCount = 0;
    motorRight.totalCount = 0;
    motorRight.pwmChannel = MOTOR_RIGHT_PWM_CHANNEL;
    motorRight.encoderPinA = (uint8_t)MOTOR_RIGHT_ENCODER_PIN_A;
    motorRight.encoderPinB = (uint8_t)MOTOR_RIGHT_ENCODER_PIN_B;
    motorRight.encoderTimer = (uint8_t)MOTOR_RIGHT_ENCODER_TIMER;
    motorRight.directionPin_P = MOTOR_RIGHT_DIR_PIN_P;
    motorRight.directionPin_N = MOTOR_RIGHT_DIR_PIN_N;
}

/**
 * 设置电机的目标速度
 * @param motor 指向电机结构体的指针
 * @param targetSpeed 目标速度值
 */
void Motor_SetTargetSpeed(Motor_TypeDef *motor, float targetSpeed)
{
    motor->targetSpeed = targetSpeed;
}

/**
 * 设置电机的PWM占空比
 * @param motor 指向电机结构体的指针
 * @param duty 占空比值（-10000~10000对应-100%~100%）
 */
void Motor_SetDuty(Motor_TypeDef *motor, float duty)
{
    if (duty < 0)
    {
        duty = -duty;
        gpio_set_level((gpio_pin_enum)(motor->directionPin_P), 0);
        gpio_set_level((gpio_pin_enum)(motor->directionPin_N), 1);
    }
    pwm_set_duty(motor->pwmChannel, (uint32_t)(duty));
    if (duty > 0)
    {
        // duty = duty;
        gpio_set_level((gpio_pin_enum)(motor->directionPin_P), 1);
        gpio_set_level((gpio_pin_enum)(motor->directionPin_N), 0);
    }

    pwm_set_duty(motor->pwmChannel, (uint32_t)(duty));
}

/**
 * 设置电机的PID参数
 * @param motor 指向电机结构体的指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void Motor_SetPIDParameters(Motor_TypeDef *motor, float kp, float ki, float kd)
{
    motor->pid.Kp = kp;
    motor->pid.Ki = ki;
    motor->pid.Kd = kd;
}

/**
 * 采样电机编码器的脉冲计数
 * @param motor 指向电机结构体的指针
 * @return 本次采样的脉冲增量
 */
int16_t Motor_SampleEncoder(Motor_TypeDef *motor)
{
    if (motor == NULL)
    {
        return 0;
    }

    int16_t delta = encoder_get_count((encoder_index_enum)(motor->encoderPinA));
    motor->encoderCount = delta;
    motor->totalCount += delta;
    encoder_clear_count((encoder_index_enum)(motor->encoderTimer));

    return delta;
}

/**
 * 获取电机的累计行驶距离（编码器脉冲总数）
 * @param motor 指向电机结构体的指针
 * @return 累计的编码器脉冲数
 */
int32_t Motor_GetTotalDistanceCount(Motor_TypeDef *motor)
{
    if (motor == NULL)
    {
        return 0;
    }

    return motor->totalCount;
}

/**
 * 重置电机的累计行驶距离计数
 * @param motor 指向电机结构体的指针
 */
void Motor_ResetTotalDistance(Motor_TypeDef *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->totalCount = 0;
    motor->encoderCount = 0;
    encoder_clear_count((encoder_index_enum)(motor->encoderPinA));
}

/**
 * 通过串级PID控制电机速度，使其保持平衡
 * @param motor 指向电机结构体的指针
 * @param actualSpeed 实际速度反馈值
 */
void Motor_UpdateSpeed(Motor_TypeDef *motor, float actualSpeed)
{
    if (motor == NULL)
    {
        return;
    }

    motor->speed = actualSpeed;

    float pidOutput = PID_Calculate(&motor->pid, motor->targetSpeed, motor->speed);
    Motor_SetDuty(motor, pidOutput);
}
