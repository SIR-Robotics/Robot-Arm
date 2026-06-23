#pragma once

#include <stdint.h>

inline int pwmCountToLogicalAngle(uint16_t count, int lo, int hi, bool invert) {
    long us = (long)count * 20000L / 4096L;
    long angle = lo + (us - 500L) * (hi - lo) / 2000L;
    if (angle < lo) angle = lo;
    if (angle > hi) angle = hi;
    return invert ? lo + hi - (int)angle : (int)angle;
}

inline uint16_t stepPwmToward(uint16_t current, uint16_t target) {
    if (current < target) return current + 1;
    if (current > target) return current - 1;
    return current;
}
