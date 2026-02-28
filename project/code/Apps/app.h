#ifndef APP_H
#define APP_H

#include "zf_common_headfile.h"

typedef enum
{
    APP_MODE_REMOTE = 0,
    APP_MODE_TEACH_IDLE,
    APP_MODE_TEACH_RECORDING,
    APP_MODE_TEACH_PLAYBACK,
} AppModeState_enum;

void App_Mode_Init(void);
void App_Mode_Tick20ms(void);
uint8_t App_Mode_IsTeachActive(void);
AppModeState_enum App_Mode_GetState(void);

#endif
