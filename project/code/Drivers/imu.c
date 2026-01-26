#include "imu.h"

// 私有变量
static soft_spi_info_struct spi;
static float accel_scale = 0.0f;
static float gyro_scale = 0.0f;
static gpio_pin_enum cs_pin;

// ==================== 私有函数 ====================
static uint8_t SPI_ReadRegister(uint8_t reg)
{
    uint8_t data;
    
    gpio_low(cs_pin);  // 片选低
    
    // 发送寄存器地址（设置读位）
    soft_spi_read_write_byte(&spi, reg | ICM45686_SPI_READ);
    // 读取数据
    data = soft_spi_read_write_byte(&spi, 0xFF);
    
    gpio_high(cs_pin);  // 片选高
    
    return data;
}

static void SPI_WriteRegister(uint8_t reg, uint8_t data)
{
    gpio_low(cs_pin);  // 片选低
    
    // 发送寄存器地址（清除读位）
    soft_spi_read_write_byte(&spi, reg & ~ICM45686_SPI_READ);
    // 发送数据
    soft_spi_read_write_byte(&spi, data);
    
    gpio_high(cs_pin);  // 片选高
}

static void SPI_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    uint8_t i;
    
    gpio_low(cs_pin);  // 片选低
    
    // 发送寄存器地址（设置读位）
    soft_spi_read_write_byte(&spi, reg | ICM45686_SPI_READ);
    
    // 连续读取数据
    for(i = 0; i < length; i++)
    {
        buffer[i] = soft_spi_read_write_byte(&spi, 0xFF);
    }
    
    gpio_high(cs_pin);  // 片选高
}

// ==================== 初始化 ====================
uint8_t IMU_Init(spi_pin_enum sck_pin, spi_pin_enum mosi_pin, spi_pin_enum miso_pin, gpio_pin_enum cs_pin_num)
{
    uint8_t id;
    
    // 保存CS引脚
    cs_pin = cs_pin_num;
    
    // 初始化CS引脚为输出高电平
    gpio_init(cs_pin, GPO, GPIO_HIGH, GPI_NO_PULL);
    
    // 初始化软件SPI
    // 根据逐飞库函数：void soft_spi_init(soft_spi_info_struct *soft_spi_obj,
    //                                    spi_pin_enum sck_pin,
    //                                    spi_pin_enum mosi_pin,
    //                                    spi_pin_enum miso_pin,
    //                                    uint32 delay)
    soft_spi_init(&spi, sck_pin, mosi_pin, miso_pin, 1);
    systick_delay_ms(10);
    
    // 检查设备ID
    id = SPI_ReadRegister(ICM45686_REG_ID);
    if(id != ICM45686_ID_VALUE)
    {
        // 不是ICM45686
        return 0;
    }
    
    // 复位设备
    SPI_WriteRegister(ICM45686_REG_PWR_MGMT, 0x80);  // 复位
    systick_delay_ms(100);
    
    // 唤醒设备，选择时钟源
    SPI_WriteRegister(ICM45686_REG_PWR_MGMT, 0x01);  // 自动选择时钟源
    systick_delay_ms(10);
    
    // 配置加速度计 ±4g
    SPI_WriteRegister(ICM45686_REG_ACCEL_CONFIG, ACCEL_RANGE_4G);
    // 计算比例因子：4g = 4*9.8 = 39.2m/s²，对应32768
    accel_scale = 39.2f / 32768.0f;
    
    // 配置陀螺仪 ±500dps
    SPI_WriteRegister(ICM45686_REG_GYRO_CONFIG, GYRO_RANGE_500DPS);
    // 500dps = 500 * π/180 = 8.7266 rad/s
    gyro_scale = 8.7266f / 32768.0f;
    
    systick_delay_ms(50);
    return 1;
}

// ==================== 检查设备 ====================
uint8_t IMU_ID(void)
{
    uint8_t id = SPI_ReadRegister(ICM45686_REG_ID);
    return id;
}

// ==================== 读取数据 ====================
uint8_t IMU_ReadData(IMU_Data *data)
{
    uint8_t buffer[12];
    
    data->timestamp = systick_get_ms();
    
    // 一次性读取12字节（加速度计6字节+陀螺仪6字节）
    SPI_ReadRegisters(ICM45686_REG_ACCEL_XOUT_H, buffer, 12);
    
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
