/*
 * @Author: ELEGOO
 * @Date: 2025-01-15 16:04:05
 * @LastEditTime: 2026-01-14 16:04:05
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - RGB LED Control Header / 智能机器人小车 V4.0 - RGB LED 控制头文件
 */

#ifndef _RGB_CONTROL_H_
#define _RGB_CONTROL_H_

#include <Arduino.h>
#include <FastLED.h>
#include "DeviceDriverSet.h"
#include "ApplicationFunctionSet.h" 

/**
 * RGB LED Control Class / RGB LED 控制类
 */
class RgbControl {
public:
    // ========== Constructor / 构造函数 ==========
    // Constructor / 构造函数
    RgbControl(DeviceDriverSet_RGBLED &rgbLed);
    
    // ========== Public Methods / 公共方法 ==========
    // Update LED display / 更新 LED 显示
    void update(bool voltageStatus, State carState);
    
    // Execute lighting command / 执行照明命令
    void executeLightingCommand(uint8_t lightingSequence, 
                               uint8_t redValue,
                               uint8_t greenValue, 
                               uint8_t blueValue);

private:
    // ========== Hardware Reference / 硬件引用 ==========
    DeviceDriverSet_RGBLED &rgbLedDevice;
    
    // ========== Animation Constants / 动画常量 ==========
    static const uint8_t MAX_BRIGHTNESS = 100;        ///< Max brightness / 最大亮度
    static const uint8_t MIN_BRIGHTNESS = 0;          ///< Min brightness / 最小亮度
    static const unsigned long BREATH_INTERVAL = 10;  ///< Breath interval (ms) / 呼吸间隔 (ms)
    static const unsigned long BLINK_INTERVAL = 500;  ///< Blink interval (ms) / 闪烁间隔 (ms)
    static const unsigned long LOW_POWER_INTERVAL = 3000; ///< Low power interval (ms) / 低电量间隔 (ms)
    
    // ========== State Management / 状态管理 ==========
    // LED animation flags / LED 动画标志
    struct LedFlags {
        uint8_t breathingUp : 1;    ///< Breath direction / 呼吸方向
        uint8_t blinkState : 1;     ///< Blink state / 闪烁状态
        uint8_t reserved : 6;       ///< Reserved / 保留
    };
    
    // LED state / LED 状态
    struct LedState {
        uint8_t brightness;         ///< Brightness / 亮度
        LedFlags flags;             ///< Flags / 标志
        uint32_t lastUpdateTime;    ///< Last update / 上次更新
    } ledState;
    
    // ========== Color Definitions / 颜色定义 ==========
    // State color definitions / 状态颜色定义
    static const uint32_t PROGMEM STATE_COLORS[];
    
    // ========== Private Methods / 私有方法 ==========
    // Handle low power warning / 处理低电量警告
    void handleLowPowerWarning();
    
    // Handle standby mode / 处理待机模式
    void handleStandbyMode(bool isLowPower);
    
    // Execute breathing effect / 执行呼吸效果
    void executeBreathingEffect();
    
    // Execute blinking effect / 执行闪烁效果
    void executeBlinkingEffect(CRGB color);
    
    // Set LED color and brightness / 设置 LED 颜色和亮度
    void setLedColor(CRGB color, uint8_t brightness = MAX_BRIGHTNESS);
};

#endif // _RGB_CONTROL_H_
