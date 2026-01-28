#include "zf_common_headfile.h"
#include "infrared_sense.h"
#include "line_following.h"

static LineFollower_t follower = {0};

// 偏差映射表（10倍放大）
static const int8_t dev_map[32] = {
     0, +10, +5, +8, -5, 0, 0, 0, -10, -5, -8, -3, -8, -4, -6, -2,
    +10, +5, +8, +3, +5, +2, +3, +1, +8, +4, +6, +2, +6, +3, +4, 0
};

// 一阶低通滤波器
static inline int16_t smooth(int16_t new_val, int16_t old_val) {
    return (new_val * 7 + old_val * 3) / 10;  // α=0.7
}

void line_init(void) {
    PID_Init(&follower.pid, 15.0f, 0.1f, 8.0f);
    follower.pid.OutputMax = 60.0f;
    follower.pid.OutputMin = -60.0f;
    InfraredSense_Init();
}

void line_update(void) {
    InfraredSensor_Tick();
    uint8_t sens = GetInfraredSenseFlag();
    
    // 计算偏差
    int16_t raw_dev = (sens < 32) ? dev_map[sens] : 0;
    if (sens == 0x1F) raw_dev = (follower.deviation > 0) ? 5 : -5;
    
    // 平滑滤波
    follower.deviation = smooth(raw_dev, follower.deviation);
    
    // 状态判断
    follower.state = (sens == 0) ? TRACK_LOST : TRACK_OK;
    
    // PID计算
    PID_Calculate(&follower.pid, 0, follower.deviation / 10.0f);
}

int16_t line_get_pid_out(void) {
    return (int16_t)follower.pid.Output;
}

TrackState_t line_get_state(void) {
    return follower.state;
}

// 直接计算差速
void line_get_motor_diff(int16_t base, int16_t *left, int16_t *right) {
    int16_t pid = line_get_pid_out();
    int16_t l = base - pid, r = base + pid;
    
    // 边界保护
    *left  = (l < 0) ? 0 : (l > 99 ? 99 : l);
    *right = (r < 0) ? 0 : (r > 99 ? 99 : r);
}

// 调试输出
void line_debug(void) {
    uint8_t sens = GetInfraredSenseFlag();
    int16_t left, right;
    line_get_motor_diff(60, &left, &right);
    
    // 一行显示所有关键信息
    oled_show_string(1, 1, "S:");
    oled_show_binary(1, 4, sens, 5);
    
    oled_show_string(1, 10, "D:");
    oled_show_int(1, 13, follower.deviation, 3);
    
    oled_show_string(2, 1, "PID:");
    oled_show_int(2, 6, (int16_t)follower.pid.Output, 3);
    
    oled_show_string(2, 10, "M:");
    oled_show_int(2, 12, left, 2);
    oled_show_string(2, 15, ":");
    oled_show_int(2, 16, right, 2);
}
