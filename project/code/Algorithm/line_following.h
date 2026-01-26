#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#endif // LINE_FOLLOWING_H

#define ON_LINE_TRACKING 0
#define OFF_LINE_TRACKING 1

// 检测到线为0，未检测到为1
// 这里的状态值是多个可能值的集合，不能用“||”生成单一常量
static inline uint8_t is_middle(uint8_t v) { return (v == 0x1B) || (v == 0x11); }
static inline uint8_t is_middle_left(uint8_t v) { return (v == 0x19) || (v == 0x1D); }
static inline uint8_t is_left(uint8_t v) { return (v == 0x18) || (v == 0x1C); }
static inline uint8_t is_far_left(uint8_t v) { return (v == 0x10) || (v == 0x14); }
static inline uint8_t is_middle_right(uint8_t v) { return (v == 0x13) || (v == 0x17); }
static inline uint8_t is_right(uint8_t v) { return (v == 0x12) || (v == 0x16); }
static inline uint8_t is_far_right(uint8_t v) { return (v == 0x02) || (v == 0x06); }