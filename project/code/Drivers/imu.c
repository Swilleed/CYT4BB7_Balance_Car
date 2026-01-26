#include "imu.h"

// 私有变量
static soft_iic_info_struct iic;
static float accel_scale = 0.0f;
static float gyro_scale = 0.0f;

// ==================== 初始化 ====================
uint8_t IMU_Init(gpio_pin_enum scl_pin, gpio_pin_enum sda_pin)
{
    uint8_t id;
    
    // 1. 初始化软件I2C - 根据头文件函数原型：
    // void soft_iic_init(soft_iic_info_struct *soft_iic_obj, 
    //                    uint8 addr, 
    //                    uint32 delay, 
    //                    gpio_pin_enum scl_pin, 
    //                    gpio_pin_enum sda_pin);
    soft_iic_init(&iic, ICM45686_I2C_ADDR, 2, scl_pin, sda_pin);
    systick_delay_ms(10);
    
    // 2. 检查设备ID - 使用正确的函数名
    id = soft_iic_read_8bit_register(&iic, ICM45686_REG_ID);
    if(id != ICM45686_ID_VALUE)
    {
        // 不是ICM45686
        return 0;
    }
    
    // 3. 复位设备
    soft_iic_write_8bit_register(&iic, ICM45686_REG_PWR_MGMT, 0x80);  // 复位
    systick_delay_ms(100);
    
    // 4. 唤醒设备，选择时钟源
    soft_iic_write_8bit_register(&iic, ICM45686_REG_PWR_MGMT, 0x01);  // 唤醒
    systick_delay_ms(10);
    
    // 5. 配置加速度计 ±4g
    soft_iic_write_8bit_register(&iic, ICM45686_REG_ACCEL_CONFIG, ACCEL_RANGE_4G);
    // 计算比例因子：4g = 4*9.8 = 39.2m/s²，对应32768
    accel_scale = 39.2f / 32768.0f;
    
    // 6. 配置陀螺仪 ±500dps
    soft_iic_write_8bit_register(&iic, ICM45686_REG_GYRO_CONFIG, GYRO_RANGE_500DPS);
    // 500dps = 500 * π/180 = 8.7266 rad/s
    gyro_scale = 8.7266f / 32768.0f;
    
    systick_delay_ms(50);
    return 1;
}

// ==================== 检查设备 ====================
uint8_t IMU_ID(void)
{
    uint8_t id = soft_iic_read_8bit_register(&iic, ICM45686_REG_ID);
    return id;
}

// ==================== 读取数据（一次性读取12字节） ====================
uint8_t IMU_ReadData(IMU_Data *data)
{
    uint8_t buffer[12];
    
    data->timestamp = systick_get_ms();
    
    // 使用soft_iic_read_8bit_registers一次性读取12字节
    soft_iic_read_8bit_registers(&iic, ICM45686_REG_ACCEL_XOUT_H, buffer, 12);
    
    // 解析加速度计数据（前6字节）
    int16_t ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t az = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // 解析陀螺仪数据（后6字节）
    int16_t gx = (int16_t)((buffer[6] << 8) | buffer[7]);
    int16_t gy = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t gz = (int16_t)((buffer[10] << 8) | buffer[11]);
    
    // 转换为物理量
    data->ax = ax * accel_scale;
    data->ay = ay * accel_scale;
    data->az = az * accel_scale;
    
    data->gx = gx * gyro_scale;
    data->gy = gy * gyro_scale;
    data->gz = gz * gyro_scale;
    
    return 1;
}
