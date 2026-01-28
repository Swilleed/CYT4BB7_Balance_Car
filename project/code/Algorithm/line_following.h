#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include "zf_common_headfile.h"
#include "infrared_sense.h"
#include "pid_controller.h"

typedef enum {
    TRACK_OK = 0,
    TRACK_LOST
} TrackState_t;

typedef struct {
    PID_TypeDef pid;
    int16_t deviation;
    TrackState_t state;
    uint32_t lost_tick;
} LineFollower_t;

// 接口
void line_init(void);
void line_update(void);
int16_t line_get_pid_out(void);
TrackState_t line_get_state(void);
void line_debug(void);

#endif // LINE_FOLLOWING_H
