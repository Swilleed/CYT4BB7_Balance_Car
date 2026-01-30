#ifndef INFRARED_SENSE_H
#define INFRARED_SENSE_H

#include <stdint.h>

#define INFRARED_SENSOR_1 P03_0
#define INFRARED_SENSOR_2 P03_1
#define INFRARED_SENSOR_3 P03_2
#define INFRARED_SENSOR_4 P03_3
#define INFRARED_SENSOR_5 P03_4

void InfraredSense_Init(void);
void InfraredSensor_Tick(void);
uint8_t GetInfraredSenseFlag(void);
void Debug_InfraredSense(void);

#endif // INFRARED_SENSE_H
