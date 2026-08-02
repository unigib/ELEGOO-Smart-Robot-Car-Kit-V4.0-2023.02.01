/*
 * @Author: ELEGOO
 * @Date: 2025-01-15 16:04:05
 * @LastEditTime: 2026-01-14 16:04:05
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - Utility Functions Implementation / 智能机器人小车 V4.0 - 实用函数实现
 */

#include "Utils.h"
#include <avr/wdt.h>

// ========== NonBlockingTimer Implementation / 非阻塞定时器实现 ==========

NonBlockingTimer::NonBlockingTimer() : _startTime(0), _duration(0), _active(false) {}

void NonBlockingTimer::start(unsigned long durationMs) {
    _startTime = millis();
    _duration = durationMs;
    _active = true;
}

bool NonBlockingTimer::isExpired() {
    if (!_active) return false;
    
    if (millis() - _startTime >= _duration) {
        _active = false; // Timer finished / 定时器完成
        return true;
    }
    return false;
}

bool NonBlockingTimer::isRunning() {
    return _active;
}

void NonBlockingTimer::stop() {
    _active = false;
}

// ========== Timing Utilities / 定时实用程序 ==========
// Delay with watchdog reset / 带看门狗复位的延迟
void Utils::delayWithWatchdog(uint16_t delayMs) {
    resetWatchdog(); // Reset watchdog / 重置看门狗
    
    for (uint16_t i = 0; i < delayMs; i++) {
        delay(1); // 1ms delay / 1ms 延迟
        
        // Reset every 100ms / 每 100ms 重置
        if (i % 100 == 0) {
            resetWatchdog();
        }
    }
}

// ========== System Utilities / 系统实用程序 ==========
// Reset watchdog timer / 重置看门狗定时器
void Utils::resetWatchdog() {
    wdt_reset();
}
