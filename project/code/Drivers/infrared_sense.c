#include "zf_common_headfile.h"
#include "infrared_sense.h"
#include "zf_driver_gpio.h"

uint8_t InfraredSenseFlag = 0;

void InfraredSense_Init(void)
{
    gpio_init(INFRARED_SENSOR_1, GPI, 0, GPI_PULL_DOWN);
    gpio_init(INFRARED_SENSOR_2, GPI, 0, GPI_PULL_DOWN);
    gpio_init(INFRARED_SENSOR_3, GPI, 0, GPI_PULL_DOWN);
    gpio_init(INFRARED_SENSOR_4, GPI, 0, GPI_PULL_DOWN);
    gpio_init(INFRARED_SENSOR_5, GPI, 0, GPI_PULL_DOWN);
}

static uint8_t InfraredSense_Read(void)
{
    uint8_t status = 0;

    // 左接b0
    if (gpio_get_level(INFRARED_SENSOR_1) == 0)
    {
        status |= INFRARED_SENSOR_1;
    }
    else
    {
        status &= ~INFRARED_SENSOR_1;
    }
    // 中左接b1
    if (gpio_get_level(INFRARED_SENSOR_2) == 0)
    {
        status |= INFRARED_SENSOR_2;
    }
    else
    {
        status &= ~INFRARED_SENSOR_2;
    }
    // 中右接b11
    if (gpio_get_level(INFRARED_SENSOR_3) == 0)
    {
        status |= INFRARED_SENSOR_3;
    }
    else
    {
        status &= ~INFRARED_SENSOR_3;
    }
    // 右接b10
    if (gpio_get_level(INFRARED_SENSOR_4) == 0)
    {
        status |= INFRARED_SENSOR_4;
    }
    else
    {
        status &= ~INFRARED_SENSOR_4;
    }
    if (gpio_get_level(INFRARED_SENSOR_5) == 0)
    {
        status |= INFRARED_SENSOR_5;
    }
    else
    {
        status &= ~INFRARED_SENSOR_5;
    }

    return status;
}

static uint8_t Digital_filter(void)
{
    uint8_t raw = InfraredSense_Read();
    static uint8_t history[5] = {0};
    static uint8_t index = 0;
    history[index] = raw;
    index = (index + 1) % 5;

    uint8_t filtered = 0x0F;
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
    uint8_t flag = 0;
    flag |= (gpio_get_level(INFRARED_SENSOR_1) << 4);
    flag |= (gpio_get_level(INFRARED_SENSOR_2) << 3);
    flag |= (gpio_get_level(INFRARED_SENSOR_3) << 2);
    flag |= (gpio_get_level(INFRARED_SENSOR_4) << 1);
    flag |= (gpio_get_level(INFRARED_SENSOR_5) << 0);
    return flag;
}