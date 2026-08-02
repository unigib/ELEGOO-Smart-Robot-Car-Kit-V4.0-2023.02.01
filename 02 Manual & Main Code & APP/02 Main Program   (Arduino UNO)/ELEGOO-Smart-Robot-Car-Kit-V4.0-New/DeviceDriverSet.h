/*
 * @Author: ELEGOO
 * @Date: 2025-01-14 11:59:09
 * @LastEditTime: 2026-01-14 16:07:48
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 Device Driver Set / 智能机器人小车 V4.0 设备驱动集
 */
#pragma once

#include <FastLED.h>
#include <Servo.h>
#include "Utils.h"

/**
 * @brief MPU6050 Sensor Class / MPU6050 传感器类
 */
class DeviceDriverSet_MPU6050 {
public:
    /**
     * @brief Initialize MPU6050 sensor / 初始化 MPU6050 传感器
     * @return true if initialization failed, false if successful / 初始化失败返回 true，成功返回 false
     */
    bool init();

    /**
     * @brief Calibrate MPU6050 gyro / 校准 MPU6050 陀螺仪
     */
    void calibration();

    /**
     * @brief Get current Euler angles (Yaw) / 获取当前欧拉角 (Yaw)
     * @param Yaw Pointer to store the calculated Yaw angle / 用于存储计算出的偏航角的指针
     */
    void getEulerAngles(float *Yaw);

public:
    int16_t gz;
    unsigned long now, lastTime = 0;
    float dt;      // Derivative time / 微分时间
    float agz = 0; // Angle variable / 角度变量
    long gzo = 0;  // Gyro offset / 陀螺仪偏移
};

/**
 * @brief RGB LED Control Class / RGB LED 控制类
 */
class DeviceDriverSet_RGBLED {
public:
    /**
     * @brief Initialize the RGB LED module / 初始化 RGB LED 模块
     * @param brightness Initial brightness level (0-255) / 初始亮度级别 (0-255)
     */
    void init(uint8_t brightness);

    /**
     * @brief Set RGB LED color using FastLED CRGB object / 使用 FastLED CRGB 对象设置 RGB LED 颜色
     * @param Traversal_Number LED index to control / 要控制的 LED 索引
     * @param colour Color to set / 设置的颜色
     */
    void SetRBGLED(uint8_t Traversal_Number, CRGB colour);

    /**
     * @brief Set RGB LED color using individual RGB values / 使用单独的 RGB 值设置 RGB LED 颜色
     * @param ledIndex LED index to control / 要控制的 LED 索引
     * @param r Red component (0-255) / 红色分量 (0-255)
     * @param g Green component (0-255) / 绿色分量 (0-255)
     * @param b Blue component (0-255) / 蓝色分量 (0-255)
     */
    void setColor(uint8_t ledIndex, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Clear all LED colors (turn off) / 清除所有 LED 颜色（关闭）
     */
    void clear();

private:
    static constexpr uint8_t PIN_RGBLED = 4;  ///< RGB LED control pin / RGB LED 控制引脚
    static constexpr uint8_t NUM_LEDS = 1;    ///< Number of RGB LEDs / RGB LED 数量
    CRGB leds_[NUM_LEDS];                     ///< LED array / LED 数组
};

/**
 * @brief Button Detection Class / 按键检测类
 */
class DeviceDriverSet_Key {
public:
    /**
     * @brief Initialize the button detection module / 初始化按键检测模块
     */
    void init();

    /**
     * @brief Get the current button state / 获取当前按键状态
     * @param get_keyValue Pointer to store the button value / 用于存储按键值的指针
     */
    void getKeyValue(uint8_t *get_keyValue);
    
public:
    static constexpr uint8_t PIN_KEY = 2;         ///< Button input pin / 按键输入引脚
    static constexpr uint8_t KEY_VALUE_MAX = 3;   ///< Maximum button value / 按键状态最大值
    static uint8_t keyValue_;                     ///< Current button value / 当前按键值
};

/**
 * @brief ITR20001 Line Tracking Sensor Class / ITR20001 循迹传感器类
 */
class DeviceDriverSet_ITR20001 {
public:
    /**
     * @brief Initialize the line tracking sensors / 初始化循迹传感器
     */
    void init();

    /**
     * @brief Get left sensor analog value / 获取左侧传感器模拟值
     * @return Analog reading (0-1023) / 模拟读数 (0-1023)
     */
    int getAnalogueL() const;

    /**
     * @brief Get middle sensor analog value / 获取中间传感器模拟值
     * @return Analog reading (0-1023) / 模拟读数 (0-1023)
     */
    int getAnalogueM() const;

    /**
     * @brief Get right sensor analog value / 获取右侧传感器模拟值
     * @return Analog reading (0-1023) / 模拟读数 (0-1023)
     */
    int getAnalogueR() const;

private:
    // Pin configuration / 引脚配置
    static constexpr uint8_t PIN_ITR_L = A2;  ///< Left sensor pin / 左侧传感器引脚
    static constexpr uint8_t PIN_ITR_M = A1;  ///< Middle sensor pin / 中间传感器引脚
    static constexpr uint8_t PIN_ITR_R = A0;  ///< Right sensor pin / 右侧传感器引脚
};

/**
 * @brief Voltage Detection Class / 电压检测类
 */
class DeviceDriverSet_Voltage {
public:
    /**
     * @brief Initialize the voltage detection module / 初始化电压检测模块
     */
    void init();

    /**
     * @brief Get current battery voltage / 获取当前电池电压
     * @return Voltage in volts / 电压（伏特）
     */
    float getVoltage() const;

private:
    static constexpr uint8_t PIN_VOLTAGE = A3;  ///< Voltage detection pin / 电压检测引脚
};


/**
 * @brief Motor Control Class / 电机控制类
 */
class DeviceDriverSet_Motor {
public:
    /**
     * @brief Initialize the motor control module / 初始化电机控制模块
     */
    void init();

    /**
     * @brief Control both motors / 控制两个电机
     * @param directionRight Motor Right direction / 右电机方向
     * @param speedRight Motor Right speed (0-255) / 右电机速度 (0-255)
     * @param directionLeft Motor Left direction / 左电机方向
     * @param speedLeft Motor Left speed (0-255) / 左电机速度 (0-255)
     */
    void control(bool directionRight, uint8_t speedRight,
                bool directionLeft, uint8_t speedLeft);

private:
    // Pin definitions / 引脚定义
    static constexpr uint8_t PIN_PWM_RIGHT = 5;    ///< Motor Right PWM / 右电机 PWM
    static constexpr uint8_t PIN_PWM_LEFT = 6;    ///< Motor Left PWM / 左电机 PWM
    static constexpr uint8_t PIN_LEFT_IN = 8;   ///< Motor Left direction / 左电机方向
    static constexpr uint8_t PIN_RIGHT_IN = 7;   ///< Motor Right direction / 右电机方向
    static constexpr uint8_t PIN_STBY = 3;    ///< Standby pin / 待机引脚

};

/**
 * @brief Ultrasonic Sensor Class / 超声波传感器类
 */
class DeviceDriverSet_Ultrasonic {
public:
    /**
     * @brief Initialize the ultrasonic sensor / 初始化超声波传感器
     */
    void init();

    /**
     * @brief Measure distance to obstacle / 测量障碍物距离
     * @param distance Pointer to store distance (cm) / 用于存储距离（厘米）的指针
     */
    void getDistance(uint16_t* distance) const;

private:
    static constexpr uint8_t TRIG_PIN = 13;  ///< Trigger pin / 触发引脚
    static constexpr uint8_t ECHO_PIN = 12;  ///< Echo pin / 回声引脚
};

/**
 * @brief Servo Control Class / 舵机控制类
 */
class DeviceDriverSet_Servo {
public:
    /**
     * @brief Initialize servo motors / 初始化舵机
     * @param positionAngle Initial angle (0-180) / 初始角度 (0-180)
     */
    void init(unsigned int positionAngle);

    /**
     * @brief Control servo position with delay / 控制舵机位置（带延迟）
     * @param positionAngle Target angle (0-180) / 目标角度 (0-180)
     * @param delayTime Delay in ms / 延迟（毫秒）
     */
    void control(unsigned int positionAngle,uint16_t delayTime);

    /**
     * @brief Control multiple servos / 控制多个舵机
     * @param servo Servo selection (0=Z axis, 1=Y axis) / 舵机选择 (0=Z 轴, 1=Y 轴)
     * @param positionAngle Target angle (0-180) / 目标角度 (0-180)
     */
    void controlMultiple(uint8_t servo, unsigned int positionAngle);

    /**
     * @brief Detach all servos to save power / 分离所有舵机以节省电力
     */
    void detachServos();

    /**
     * @brief Set Z-axis servo angle (Non-blocking) / 设置 Z 轴舵机角度（非阻塞）
     * @param positionAngle Target angle (0-180) / 目标角度 (0-180)
     */
    void setZAngle(unsigned int positionAngle);

    /**
     * @brief Get current servo angle / 获取当前舵机角度
     * @return Current angle / 当前角度
     */
    int getCurrentAngle();

    /**
     * @brief Update servo state machine / 更新舵机状态机
     */
    void update();

private:
    static constexpr uint8_t PIN_SERVO_Z = 10;  ///< Z-axis servo pin / Z 轴舵机引脚
    static constexpr uint8_t PIN_SERVO_Y = 11;  ///< Y-axis servo pin / Y 轴舵机引脚
    Servo servoZ_;                              ///< Z-axis servo object / Z 轴舵机对象
    Servo servoY_;                              ///< Y-axis servo object / Y 轴舵机对象

    // State machine for non-blocking control / 非阻塞控制状态机
    enum ServoState {
        IDLE,
        START_Z,
        WAITING_Z,
        START_Y,
        WAITING_Y,
        FINISH
    };
    
    ServoState currentState_ = IDLE;
    NonBlockingTimer timer_;
    uint8_t targetServo_ = 0;
    unsigned int targetAngle_ = 0;
};

/**
 * @brief IR Remote Control Class / 红外遥控类
 */
class DeviceDriverSet_IRrecv {

private:
#define RECV_PIN 9  ///< IR receiver pin / 红外接收引脚

public:
    /**
     * @brief IR Remote Button Codes / 红外遥控按键代码
     */
    enum IRKey {
        NONE = 0,
        UP = 1,
        DOWN = 2,
        LEFT = 3,
        RIGHT = 4,
        OK = 5,
        NUM_1 = 6,
        NUM_2 = 7,
        NUM_3 = 8,
        NUM_4 = 9,
        NUM_5 = 10,
        NUM_6 = 11,
        NUM_7 = 12,
        NUM_8 = 13,
        NUM_9 = 14
    };

    /**
     * @brief Initialize IR receiver / 初始化红外接收器
     */
    void init();

    /**
     * @brief Get IR remote control value / 获取红外遥控值
     * @param irValue Pointer to store received IR code / 用于存储接收到的红外代码的指针
     * @return true if valid IR code received / 如果接收到有效的红外代码返回 true
     */
    bool getIRValue(uint8_t* irValue);

private:
    // IR code definitions for remote control (Updated for IRremote v4 LSB) / 遥控器的红外代码定义（已针对 IRremote v4 LSB 更新）
    static constexpr unsigned long A_RECV_UPPER = 0xB946FF00;  ///< Up button code / 上键代码
    static constexpr unsigned long A_RECV_LOWER = 0xEA15FF00;  ///< Down button code / 下键代码
    static constexpr unsigned long A_RECV_LEFT = 0xBB44FF00;   ///< Left button code / 左键代码
    static constexpr unsigned long A_RECV_RIGHT = 0xBC43FF00;  ///< Right button code / 右键代码
    static constexpr unsigned long A_RECV_OK = 0xBF40FF00;     ///< OK button code / OK 键代码
    static constexpr unsigned long A_RECV_1 = 0xE916FF00;      ///< Button 1 code / 按键 1 代码
    static constexpr unsigned long A_RECV_2 = 0xE619FF00;      ///< Button 2 code / 按键 2 代码
    static constexpr unsigned long A_RECV_3 = 0xF20DFF00;      ///< Button 3 code / 按键 3 代码
    static constexpr unsigned long A_RECV_4 = 0xF30CFF00;      ///< Button 4 code / 按键 4 代码
    static constexpr unsigned long A_RECV_5 = 0xE718FF00;      ///< Button 5 code / 按键 5 代码
    static constexpr unsigned long A_RECV_6 = 0xA15EFF00;      ///< Button 6 code / 按键 6 代码
    static constexpr unsigned long A_RECV_7 = 0xF708FF00;      ///< Button 7 code / 按键 7 代码
    static constexpr unsigned long A_RECV_8 = 0xE31CFF00;      ///< Button 8 code / 按键 8 代码
    static constexpr unsigned long A_RECV_9 = 0xA55AFF00;      ///< Button 9 code / 按键 9 代码
};