#ifndef __INFRARED_H_
#define __INFRARED_H_
#include "zf_common_headfile.h"
extern uint8_t Infrared[5];

void Infrared_Init(void);
int8_t Infrared_ErrorGet(uint8_t *Infrared);


#endif
