/*
 * @Author: ELEGOO
 * @Date: 2025-01-15 16:04:05
 * @LastEditTime: 2026-01-14 16:04:05
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - RGB LED Control Implementation / 智能机器人小车 V4.0 - RGB LED 控制实现
 */

#include "RgbControl.h"

// ========== Constructor / 构造函数 ==========
// Constructor / 构造函数
RgbControl::RgbControl(DeviceDriverSet_RGBLED &rgbLed) 
    : rgbLedDevice(rgbLed) {
    // Initialize state / 初始化状态
    ledState.brightness = MIN_BRIGHTNESS;
    ledState.flags.breathingUp = true;
    ledState.flags.blinkState = false;
    ledState.lastUpdateTime = 0;
}

// ========== Public Methods / 公共方法 ==========
// Execute lighting command / 执行照明命令
void RgbControl::executeLightingCommand(uint8_t lightingSequence, 
                                       uint8_t redValue,
                                       uint8_t greenValue, 
                                       uint8_t blueValue) {
    switch (lightingSequence) {
        case 0: // Default / 默认
            rgbLedDevice.setColor(1, redValue, greenValue, blueValue);
            break;
        case 1: // Left / 左
            rgbLedDevice.setColor(3, redValue, greenValue, blueValue);
            break;
        case 2: // Forward / 前
            rgbLedDevice.setColor(2, redValue, greenValue, blueValue);
            break;
        case 3: // Right / 右
            rgbLedDevice.setColor(1, redValue, greenValue, blueValue);
            break;
        case 4: // Back / 后
            rgbLedDevice.setColor(0, redValue, greenValue, blueValue);
            break;
        case 5: // Middle / 中
            rgbLedDevice.setColor(4, redValue, greenValue, blueValue);
            break;
        default:
            break;
    }
}

// Update RGB LEDs / 更新 RGB LED
void RgbControl::update(bool voltageStatus, State carState) {
    // Low battery handling / 低电量处理
    if (voltageStatus) {
        if (millis() - ledState.lastUpdateTime > LOW_POWER_INTERVAL) {
            handleLowPowerWarning();
            return;
        }
    }
    
    // Handle state lighting / 处理状态照明
    switch (carState) {
        case State::Standby:
            handleStandbyMode(voltageStatus);
            break;
        case State::TraceBased:
            rgbLedDevice.SetRBGLED(2, CRGB::Green); // Green / 绿色
            break;
        case State::Follow:
            rgbLedDevice.SetRBGLED(2, CRGB::Blue); // Blue / 蓝色
            break;  
        case State::ObstacleAvoidance:
            rgbLedDevice.SetRBGLED(2, CRGB::Yellow); // Yellow / 黄色
            break;
        case State::Rocker:
            rgbLedDevice.SetRBGLED(2, CRGB::Violet); // Violet / 紫罗兰色
            break;
        default:
            setLedColor(CRGB::Black);
            break;
    }
}

// ========== Private Methods / 私有方法 ==========
// Handle low power warning / 处理低电量警告
void RgbControl::handleLowPowerWarning() {
    executeBlinkingEffect(CRGB::Red);
    ledState.lastUpdateTime = millis();
}

// Handle standby mode / 处理待机模式
void RgbControl::handleStandbyMode(bool isLowPower) {
    if (isLowPower) {
        executeBlinkingEffect(CRGB::Red);
    } else {
        executeBreathingEffect();
        setLedColor(CRGB::Violet, ledState.brightness);
    }
}

// Execute breathing effect / 执行呼吸效果
void RgbControl::executeBreathingEffect() {
    if (millis() - ledState.lastUpdateTime > BREATH_INTERVAL) {
        ledState.lastUpdateTime = millis();
        
        if (ledState.flags.breathingUp) {
            ledState.brightness++;
            if (ledState.brightness >= MAX_BRIGHTNESS) {
                ledState.flags.breathingUp = false;
            }
        } else {
            ledState.brightness--;
            if (ledState.brightness <= MIN_BRIGHTNESS) {
                ledState.flags.breathingUp = true;
            }
        }
    }
}

// Execute blinking effect / 执行闪烁效果
void RgbControl::executeBlinkingEffect(CRGB color) {
    if (millis() - ledState.lastUpdateTime > BLINK_INTERVAL) {
        ledState.lastUpdateTime = millis();
        ledState.flags.blinkState = !ledState.flags.blinkState;
    }
    
    setLedColor(ledState.flags.blinkState ? color : CRGB::Black);
}

// Set LED color and brightness / 设置 LED 颜色和亮度
void RgbControl::setLedColor(CRGB color, uint8_t brightness) {
    rgbLedDevice.setColor(0, color.r, color.g, color.b);
    FastLED.setBrightness(brightness);
    FastLED.show();
}