#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>
#include "pid_controller.h"

// Hardware pin configuration placeholders (fill in once pins are finalized)
#define MOTOR_LEFT_PWM_PORT        /* GPIO port for left motor PWM */
#define MOTOR_LEFT_PWM_PIN         /* GPIO pin for left motor PWM */
#define MOTOR_LEFT_DIR_PORT        /* GPIO port for left motor direction */
#define MOTOR_LEFT_DIR_PIN         /* GPIO pin for left motor direction */
#define MOTOR_LEFT_ENCODER_TIMER   /* Timer/peripheral for left motor encoder */
#define MOTOR_LEFT_ENCODER_CHANNEL /* Channel index for left motor encoder */

#define MOTOR_RIGHT_PWM_PORT        /* GPIO port for right motor PWM */
#define MOTOR_RIGHT_PWM_PIN         /* GPIO pin for right motor PWM */
#define MOTOR_RIGHT_DIR_PORT        /* GPIO port for right motor direction */
#define MOTOR_RIGHT_DIR_PIN         /* GPIO port for right motor direction */
#define MOTOR_RIGHT_ENCODER_TIMER   /* Timer/peripheral for right motor encoder */
#define MOTOR_RIGHT_ENCODER_CHANNEL /* Channel index for right motor encoder */

#ifndef MOTOR_LEFT_ENCODER_INDEX
#define MOTOR_LEFT_ENCODER_INDEX (0U)
#endif

#ifndef MOTOR_RIGHT_ENCODER_INDEX
#define MOTOR_RIGHT_ENCODER_INDEX (1U)
#endif

#ifndef MOTOR_LEFT_PWM_CHANNEL
#define MOTOR_LEFT_PWM_CHANNEL (0U)
#endif

#ifndef MOTOR_RIGHT_PWM_CHANNEL
#define MOTOR_RIGHT_PWM_CHANNEL (1U)
#endif

typedef struct
{
    float speed;
    float targetSpeed;
    PID_TypeDef pid;
    int32_t encoderCount;
    int32_t totalCount;
    uint8_t encoderChannel;
    int8_t direction;
    uint8_t pwmChannel;
} Motor_TypeDef;

extern Motor_TypeDef motorLeft;
extern Motor_TypeDef motorRight;

void Motor_Init(void);
void Motor_SetTargetSpeed(Motor_TypeDef *motor, float targetSpeed);
void Motor_UpdateSpeed(Motor_TypeDef *motor, float actualSpeed);
void Motor_SetDuty(Motor_TypeDef *motor, float duty);
void Motor_SetPIDParameters(Motor_TypeDef *motor, float kp, float ki, float kd);
int16_t Motor_SampleEncoder(Motor_TypeDef *motor);
int32_t Motor_GetTotalDistanceCount(Motor_TypeDef *motor);
void Motor_ResetTotalDistance(Motor_TypeDef *motor);

#endif // MOTOR_H
