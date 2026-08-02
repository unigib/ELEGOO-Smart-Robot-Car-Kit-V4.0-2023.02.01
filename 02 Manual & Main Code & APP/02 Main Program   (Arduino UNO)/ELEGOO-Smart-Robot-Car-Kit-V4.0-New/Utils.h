/*
 * @Author: ELEGOO
 * @Date: 2025-01-15 16:04:05
 * @LastEditTime: 2026-01-14 16:04:05
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - Utility Functions Header / 智能机器人小车 V4.0 - 实用函数头文件
 */

#ifndef _UTILS_H_
#define _UTILS_H_

#include <Arduino.h>

// Non-blocking Timer Class / 非阻塞定时器类
class NonBlockingTimer {
public:
    // Construct a new Non Blocking Timer object / 构造一个新的非阻塞定时器对象
    NonBlockingTimer();

    // Start the timer with a specific duration / 启动具有特定持续时间的定时器
    void start(unsigned long durationMs);

    // Check if the timer has expired / 检查定时器是否已过期
    bool isExpired();

    // Check if the timer is currently active / 检查定时器当前是否处于活动状态
    bool isRunning();

    // Stop the timer / 停止定时器
    void stop();

private:
    unsigned long _startTime;
    unsigned long _duration;
    bool _active;
};

// Utility Functions Class / 实用函数类
class Utils {
public:
    // ========== Timing Utilities / 定时实用程序 ==========
    // Delay with watchdog reset / 带看门狗复位的延迟
    static void delayWithWatchdog(uint16_t delayMs);
    
    // ========== System Utilities / 系统实用程序 ==========
    // Reset watchdog timer / 重置看门狗定时器
    static void resetWatchdog();
    
    // Constrain value / 限制值
    template<typename T>
    static T constrainValue(T value, T minValue, T maxValue) {
        if (value < minValue) return minValue;
        if (value > maxValue) return maxValue;
        return value;
    }
    
    // Map value / 映射值
    template<typename T>
    static T mapValue(T value, T fromMin, T fromMax, T toMin, T toMax) {
        return (value - fromMin) * (toMax - toMin) / (fromMax - fromMin) + toMin;
    }
};

#endif // _UTILS_H_