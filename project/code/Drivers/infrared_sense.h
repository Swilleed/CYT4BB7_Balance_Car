#ifndef INFRARED_SENSE_H
#define INFRARED_SENSE_H

#include <stdint.h>

#define INFRARED_SENSOR_1 P00_0
#define INFRARED_SENSOR_2 P00_1
#define INFRARED_SENSOR_3 P02_0
#define INFRARED_SENSOR_4 P02_1
#define INFRARED_SENSOR_5 P02_2

void InfraredSense_Init(void);
void InfraredSensor_Tick(void);
uint8_t GetInfraredSenseFlag(void);

#endif // INFRARED_SENSE_H
