#ifndef __IMU_H__
#define __IMU_H__

#include "zf_common_headfile.h"

// ICM45686 I2C地址
#define ICM45686_I2C_ADDR     0x68

// 寄存器地址
#define ICM45686_REG_ID       0x00  // 设备ID
#define ICM45686_REG_PWR_MGMT 0x06  // 电源管理
#define ICM45686_REG_ACCEL_CONFIG 0x14
#define ICM45686_REG_GYRO_CONFIG  0x18
#define ICM45686_REG_ACCEL_XOUT_H 0x0D
#define ICM45686_REG_GYRO_XOUT_H  0x13

// 设备ID
#define ICM45686_ID_VALUE     0xE0

// 量程
#define ACCEL_RANGE_4G        0x08   // ±4g
#define GYRO_RANGE_500DPS     0x07   // ±500度/秒

// IMU数据结构
typedef struct {
    float ax, ay, az;     // 加速度 m/s²
    float gx, gy, gz;     // 角速度 rad/s
    uint32_t timestamp;   // 时间戳 ms
} IMU_Data;

// 函数声明
uint8_t IMU_Init(gpio_pin_enum scl_pin, gpio_pin_enum sda_pin);
uint8_t IMU_Check(void);
uint8_t IMU_ReadData(IMU_Data *data);

#endif
