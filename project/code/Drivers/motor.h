#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>
#include "pid_controller.h"

// Hardware pin configuration placeholders (fill in once pins are finalized)
#define MOTOR_LEFT_PWM_PIN TCPWM_CH11_P05_2
#define MOTOR_LEFT_DIR_PIN_P P20_0
#define MOTOR_LEFT_DIR_PIN_N P20_1
#define MOTOR_LEFT_ENCODER_TIMER TC_CH58_ENCODER
#define MOTOR_LEFT_ENCODER_PIN_A TC_CH58_ENCODER_CH1_P17_3
#define MOTOR_LEFT_ENCODER_PIN_B TC_CH58_ENCODER_CH2_P17_4

#define MOTOR_RIGHT_PWM_PIN TCPWM_CH12_P05_3
#define MOTOR_RIGHT_DIR_PIN_P P20_2
#define MOTOR_RIGHT_DIR_PIN_N P20_3
#define MOTOR_RIGHT_ENCODER_TIMER TC_CH27_ENCODER
#define MOTOR_RIGHT_ENCODER_PIN_A TC_CH27_ENCODER_CH1_P19_2
#define MOTOR_RIGHT_ENCODER_PIN_B TC_CH27_ENCODER_CH2_P19_3

#ifndef MOTOR_PWM_FREQ
#define MOTOR_PWM_FREQ 20000U
#endif

#ifndef MOTOR_PWM_INIT_DUTY
#define MOTOR_PWM_INIT_DUTY 0U
#endif

#ifndef MOTOR_LEFT_ENCODER_INDEX
#define MOTOR_LEFT_ENCODER_INDEX (0U)
#endif

#ifndef MOTOR_RIGHT_ENCODER_INDEX
#define MOTOR_RIGHT_ENCODER_INDEX (1U)
#endif

#ifndef MOTOR_LEFT_PWM_CHANNEL
#define MOTOR_LEFT_PWM_CHANNEL MOTOR_LEFT_PWM_PIN
#endif

#ifndef MOTOR_RIGHT_PWM_CHANNEL
#define MOTOR_RIGHT_PWM_CHANNEL MOTOR_RIGHT_PWM_PIN
#endif

typedef struct
{
    float speed;
    float targetSpeed;
    PID_TypeDef pid;
    int32_t encoderCount;
    int32_t totalCount;
    uint8_t encoderPinA;
    uint8_t encoderPinB;
    uint8_t encoderTimer;
    uint8_t pwmChannel;
    uint8_t directionPin_P;
    uint8_t directionPin_N;
} Motor_TypeDef;

extern Motor_TypeDef motorLeft;
extern Motor_TypeDef motorRight;

void Motor_Init(float kp, float ki, float kd);
void Motor_SetTargetSpeed(Motor_TypeDef *motor, float targetSpeed);
void Motor_UpdateSpeed(Motor_TypeDef *motor, float actualSpeed);
void Motor_SetDuty(Motor_TypeDef *motor, float duty);
void Motor_SetPIDParameters(Motor_TypeDef *motor, float kp, float ki, float kd);
int16_t Motor_SampleEncoder(Motor_TypeDef *motor);
int32_t Motor_GetTotalDistanceCount(Motor_TypeDef *motor);
void Motor_ResetTotalDistance(Motor_TypeDef *motor);

#endif // MOTOR_H
