/*
 * @Author: ELEGOO
 * @Date: 2025-01-15 16:04:05
 * @LastEditTime: 2026-01-08 18:04:05
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - Application Function Set Header / 智能机器人小车 V4.0 - 应用功能集合头文件
 */
#ifndef _APPLICATION_FUNCTION_SET_H_
#define _APPLICATION_FUNCTION_SET_H_

#include <Arduino.h>
#include "MotionControl.h"
#include "ArduinoJson-v6.11.1.h"

/**
 * Robot car operating states enumeration / 机器人小车运行状态枚举
 */
enum class State : uint8_t {
    Standby,           ///< Standby mode / 待机模式
    TraceBased,        ///< Line tracking mode / 循迹模式
    Follow,            ///< Object following mode / 跟随模式
    ObstacleAvoidance, ///< Obstacle avoidance mode / 避障模式
    Rocker,            ///< Remote control mode / 遥控模式
    DoNothing          ///< Inactive state / 非活动状态
};

/**
 * Main application function set class / 主应用功能集合类
 */
class ApplicationFunctionSet {
public:
    // Initialize all application functions and hardware / 初始化所有应用功能和硬件
    void init();
    
    // Check and execute current state / 检查并执行当前状态
    void checkState();
    
    // Update sensor data periodically / 定期更新传感器数据
    void updateSensorData();

    // Execute rocker/remote control mode / 执行摇杆/遥控模式
    void executeRockerMode();
    
    // Execute line tracking mode / 执行循迹模式
    void executeTrackingMode();
    
    // Execute obstacle avoidance mode / 执行避障模式
    void executeObstacleMode();
    
    // Execute following mode / 执行跟随模式
    void executeFollowMode();
    
    // Execute standby mode / 执行待机模式
    void executeStandbyMode();

    // ========== Input Processing Functions / 输入处理功能 ==========
    
    // Process key/button commands / 处理按键/按钮命令
    void processKeyCommand();
    
    // Process IR remote control commands / 处理红外遥控命令
    void processIRCommand();
    
    // Analyze serial port data and execute commands / 分析串口数据并执行命令
    void analyzeSerialData(const JsonDocument& doc);

    // ========== Servo Control / 舵机控制 ==========
    
    // Control servo motor movement / 控制舵机运动
    void controlServo(uint8_t servoCommand);

    // ========== Command Processing Functions / 命令处理功能 ==========
    
    // Process motor control command / 处理电机控制命令
    void processMotorControl(const JsonDocument &doc);
    
    // Process car control with time limit / 处理带时间限制的小车控制
    void processCarControlTimeLimit(uint8_t direction, uint8_t speed, unsigned long duration);
    
    // Process individual motor speed control / 处理单独的电机速度控制
    void processMotorSpeedControl(uint8_t leftSpeed, uint8_t rightSpeed);
    
    // Clear all functions and set new state / 清除所有功能并设置新状态
    void clearAllFunctions(State newState);
    
    
    // Get ultrasonic sensor status / 获取超声波传感器状态
    void getUltrasonicStatus(uint8_t sensorId);
    
    // Get line tracking sensor status / 获取循迹传感器状态
    void getTrackingSensorStatus(uint8_t sensorId);
    
    // Process basic car control command / 处理基本小车控制命令
    void processCarControl(uint8_t direction, uint8_t speed);
    
    // Handle lighting control commands / 处理灯光控制命令
    void handleLightingCommand(const JsonDocument &doc);
    
    // Handle car control commands / 处理小车控制命令
    void handleCarControlCommand(const JsonDocument &doc);

    // ========== Timer Functions / 定时器功能 ==========
    
    // Start a timer with callback / 启动带回调的定时器
    void startTimer(unsigned long durationMs, ApplicationFunctionSet* instance, 
                   void (ApplicationFunctionSet::*callback)());
    
    // Update timer and execute callback if expired / 更新定时器并在过期时执行回调
    void updateTimer();

    // ========== Utility Functions / 实用功能 ==========
    
    // Check if value is within range / 检查值是否在范围内
    bool isWithinRange(long value, long start, long end);

    // Send command response / 发送命令响应
    void sendCommandResponse(bool includeSerialNumber);
    
    // Send response data / 发送响应数据
    void sendResponseData(const char* data);

    // ========== Callback Functions / 回调功能 ==========
    
    // Car control callback function / 小车控制回调函数
    void carControlCallback();
    
    // Lighting control callback function / 灯光控制回调函数
    void lightingCallback();

    // ========== Public Configuration / 公共配置 ==========
    
    /// Remote control speed setting / 遥控速度设置
    uint8_t rockerCarSpeed = 250;

private:
    // ========== Private Members / 私有成员 ==========
    
    /// Command serial number for response tracking / 用于响应跟踪的命令序列号
    char* commandSerialNumber;

    /// Timer management / 定时器管理
    ApplicationFunctionSet* timerInstance; 
    unsigned long timerStart;
    unsigned long timerDuration; 
    void (ApplicationFunctionSet::*timerCallback)(); 
    bool isTimerActive; 
    
    /// Current robot state / 当前机器人状态
    State currentState;
    State lastState;

    /// Sensor raw values / 传感器原始值
    volatile int trackingDataLeft;        
    volatile int trackingDataMiddle;        
    volatile int trackingDataRight;

    /// Robot status flags / 机器人状态标志
    bool isOnGround = true;
    bool voltageDetectionStatus = false;
    bool isIrRobotControl = false;
    
    /// Movement direction / 运动方向
    CarDirection carDirection;
    
    /// Individual motor state preservation / 单独电机状态保存
    struct MotorState {
        bool directionRight;     ///< Motor right direction / 右电机方向
        uint8_t speedRight;      ///< Motor right speed / 右电机速度
        bool directionLeft;     ///< Motor left direction / 左电机方向
        uint8_t speedLeft;      ///< Motor left speed / 左电机速度
        bool isInitialized;  ///< Whether motor state has been initialized / 电机状态是否已初始化
    } motorState;

public:
    // ========== Public Constants / 公共常量 ==========
    
    /// Battery voltage threshold (volts) / 电池电压阈值 (伏特)
    static constexpr float VOLTAGE_THRESHOLD = 7.00;
    
    /// Obstacle detection threshold (cm) / 障碍物检测阈值 (厘米)
    static constexpr uint8_t OBSTACLE_DETECTION_THRESHOLD = 20;
    
    /// Line tracking detection threshold / 循迹检测阈值
    static constexpr uint16_t TRACKING_DETECTION_THRESHOLD = 950;
};

/// Global application function set instance
extern ApplicationFunctionSet applicationFunctionSet;

#endif // _APPLICATION_FUNCTION_SET_H_
