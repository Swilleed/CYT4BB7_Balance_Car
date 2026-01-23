#include "zf_common_headfile.h"
#include "infrared_sense.h"
#include "zf_driver_gpio.h"

uint8_t InfraredSenseFlag = 0;

void InfraredSense_Init(void)
{
    // 如果传感器默认输出高电平，检测到物体变低电平，这里用下拉可能不稳定，推荐 GPI_PULL_UP
    // 这里暂时保持原配置，如硬件需要请修改为 GPI_PULL_UP
    gpio_init(INFRARED_SENSOR_1, GPI, 0, GPI_PULL_DOWN);
    gpio_init(INFRARED_SENSOR_2, GPI, 0, GPI_PULL_DOWN);
    gpio_init(INFRARED_SENSOR_3, GPI, 0, GPI_PULL_DOWN);
    gpio_init(INFRARED_SENSOR_4, GPI, 0, GPI_PULL_DOWN);
    gpio_init(INFRARED_SENSOR_5, GPI, 0, GPI_PULL_DOWN);
}

static uint8_t InfraredSense_Read(void)
{
    uint8_t status = 0;

    // SENSOR_1 -> bit 4
    if (gpio_get_level(INFRARED_SENSOR_1) == 0)
    {
        status |= (1 << 4);
    }

    // SENSOR_2 -> bit 3
    if (gpio_get_level(INFRARED_SENSOR_2) == 0)
    {
        status |= (1 << 3);
    }

    // SENSOR_3 -> bit 2
    if (gpio_get_level(INFRARED_SENSOR_3) == 0)
    {
        status |= (1 << 2);
    }

    // SENSOR_4 -> bit 1
    if (gpio_get_level(INFRARED_SENSOR_4) == 0)
    {
        status |= (1 << 1);
    }

    // SENSOR_5 -> bit 0
    if (gpio_get_level(INFRARED_SENSOR_5) == 0)
    {
        status |= (1 << 0);
    }

    return status;
}

static uint8_t Digital_filter(void)
{
    uint8_t raw = InfraredSense_Read();
    static uint8_t history[5] = {0};
    static uint8_t index = 0;

    history[index] = raw;
    index++;
    if (index >= 5)
    {
        index = 0;
    }

    uint8_t filtered = 0x1F;
    for (uint8_t i = 0; i < 5; i++)
    {
        filtered &= history[i];
    }
    return filtered;
}

void InfraredSensor_Tick(void)
{
    InfraredSenseFlag = Digital_filter();
}

uint8_t GetInfraredSenseFlag(void)
{
    // 返回滤波后的全局变量
    return InfraredSenseFlag;
}