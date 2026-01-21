#ifndef MOTOR_H
#define MOTOR_H
// #include "pid_controller.h"

typedef struct
{
    float speed;       // Current speed
    float targetSpeed; // Target speed
    float kp;          // Proportional gain
    float ki;          // Integral gain
    float kd;          // Derivative gain
    float integral;    // Integral term
    float lastError;   // Last error for derivative calculation
} MotorControl_t;

#endif // MOTOR_H
