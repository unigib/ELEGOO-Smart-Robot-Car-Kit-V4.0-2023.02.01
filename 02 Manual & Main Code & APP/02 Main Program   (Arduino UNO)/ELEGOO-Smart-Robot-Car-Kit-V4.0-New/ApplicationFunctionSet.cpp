/*
 * @Author: ELEGOO
 * @Date: 2025-01-15 16:04:05
 * @LastEditTime: 2026-06-08 16:04:05
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - Application Function Set Implementation / 智能机器人小车 V4.0 - 应用功能集合实现
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet.h"
#include "DeviceDriverSet.h"
#include "RgbControl.h"
#include "MotionControl.h"

// ========== Hardware Device Objects / 硬件设备对象 ==========
DeviceDriverSet_MPU6050 AppMPU6050;
DeviceDriverSet_RGBLED AppRGB_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_Ultrasonic AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;

// ========== Control Objects / 控制对象 ==========
RgbControl rgbControl(AppRGB_LED);
MotionControl motionControl(AppMotor, AppMPU6050, AppULTRASONIC, AppServo, AppITR20001);

// ========== Global Instance / 全局实例 ==========
ApplicationFunctionSet applicationFunctionSet;

// ========== Initialization / 初始化 ==========

void ApplicationFunctionSet::init() {
    // Initialize hardware components / 初始化硬件组件
    AppVoltage.init();
    AppMotor.init();
    AppServo.init(90);
    AppKey.init();
    AppRGB_LED.init(20);
    AppIRrecv.init();
    AppULTRASONIC.init();
    AppITR20001.init();

    // Initialize MPU6050 sensor / 初始化 MPU6050 传感器
    if (AppMPU6050.init()) {
        Serial.println("MPU6050 initialization failed!");
    } else {
        AppMPU6050.calibration();
    }
    
    // Initialize motor state / 初始化电机状态
    motorState.directionRight = true;      // Forward / 前进
    motorState.speedRight = 0;             // Stop / 停止
    motorState.directionLeft = true;      // Forward / 前进
    motorState.speedLeft = 0;             // Stop / 停止
    motorState.isInitialized = true;   // Initialized / 已初始化

    // Set initial state / 设置初始状态
    carDirection = stop_it;
    currentState = State::Standby;
    isIrRobotControl = false;
}

// ========== Core State Management / 核心状态管理 ==========

void ApplicationFunctionSet::checkState() {
    // State transition handling / 状态转换处理
    if (currentState != lastState) {
        if (currentState == State::Standby) {
            AppServo.control(90, 200); // Reset servo to 90 degrees when entering Standby / 进入待机模式时将舵机重置为 90 度
        }
        lastState = currentState;
    }

    static unsigned long lastRgbUpdate = 0;
    if (millis() - lastRgbUpdate > 100) { // Update LEDs every 100ms / 每 100ms 更新一次 LED
        // Update RGB LED based on state and voltage / 根据状态和电压更新 RGB LED
        rgbControl.update(voltageDetectionStatus, currentState);
        lastRgbUpdate = millis();
    }
    
    // Update servo state machine / 更新舵机状态机
    AppServo.update();

    // Execute current state function / 执行当前状态功能
    switch (currentState) {
        case State::Standby:
            executeStandbyMode();
            break;
        case State::TraceBased:
            executeTrackingMode();
            break;
        case State::Follow:
            executeFollowMode();
            break;
        case State::ObstacleAvoidance:
            executeObstacleMode();
            break;
        case State::Rocker:
            executeRockerMode();
            break;
        default:
            break;
    }
}

void ApplicationFunctionSet::updateSensorData() {
    static uint32_t lastUpdate = 0;
    static uint8_t lowVoltageCounter = 0;
    
    const uint32_t UPDATE_INTERVAL_MS = 10;
    const uint8_t LOW_VOLTAGE_THRESHOLD = 50;

    uint32_t currentTime = millis();
    if (currentTime - lastUpdate < UPDATE_INTERVAL_MS) {
        return;
    }
    lastUpdate = currentTime;

    // Update battery voltage status / 更新电池电压状态
    noInterrupts();
    uint16_t voltage = AppVoltage.getVoltage();
    interrupts();

    if (voltage < VOLTAGE_THRESHOLD) {
        if (lowVoltageCounter < 255) {
            lowVoltageCounter++;
        }
        if (lowVoltageCounter >= LOW_VOLTAGE_THRESHOLD) {
            voltageDetectionStatus = true;
        }
    } else {
        lowVoltageCounter = 0;
        voltageDetectionStatus = false;
    }

    // Update line tracking sensor data / 更新循迹传感器数据
    trackingDataRight = AppITR20001.getAnalogueR();
    trackingDataMiddle = AppITR20001.getAnalogueM();
    trackingDataLeft = AppITR20001.getAnalogueL();

    // Check if robot is on ground / 检查机器人是否在地面上
    bool isAboveGround = (trackingDataRight > TRACKING_DETECTION_THRESHOLD) &&
                        (trackingDataMiddle > TRACKING_DETECTION_THRESHOLD) &&
                        (trackingDataLeft > TRACKING_DETECTION_THRESHOLD);

    // Reset target yaw when returning to ground / 返回地面时重置目标偏航角
    if (!isOnGround && !isAboveGround) {
        motionControl.UpdateTargetYAW();
    }

    isOnGround = !isAboveGround;
}

// ========== Mode Execution Functions / 模式执行功能 ==========

void ApplicationFunctionSet::executeRockerMode() {
    motionControl.SmartRobotCarMotionControl(carDirection, rockerCarSpeed);
}

void ApplicationFunctionSet::executeTrackingMode() {
    if (isOnGround) {
        motionControl.HandleTrackingControl();
    } 
}

void ApplicationFunctionSet::executeObstacleMode() {
    if (isOnGround) {
        motionControl.HandleObstacleAvoidance();
    } 
}

void ApplicationFunctionSet::executeFollowMode() {
    if (isOnGround) {
        motionControl.HandleFollowMode();
    } 
}

void ApplicationFunctionSet::executeStandbyMode() {
    motionControl.SmartRobotCarMotionControl(stop_it, 0);
}

// ========== Servo Control / 舵机控制 ==========

void ApplicationFunctionSet::controlServo(uint8_t servoCommand) {
    if (servoCommand < 1 || servoCommand > 5) {
        return;
    }

    static uint8_t zAxisAngle = 9;  // Z axis angle / Z 轴角度
    static uint8_t yAxisAngle = 9;  // Y axis angle / Y 轴角度
    
    const uint8_t Y_AXIS_MIN = 3;
    const uint8_t Y_AXIS_MAX = 11;
    const uint8_t Z_AXIS_MIN = 1;
    const uint8_t Z_AXIS_MAX = 17;
    const uint8_t CENTER_POSITION = 9;

    switch (servoCommand) {
        case 1: // Y axis down / Y 轴向下
            yAxisAngle = constrain(yAxisAngle - 1, Y_AXIS_MIN, Y_AXIS_MAX);
            AppServo.controlMultiple(2, yAxisAngle);
            break;
        case 2: // Y axis up / Y 轴向上
            yAxisAngle = constrain(yAxisAngle + 1, Y_AXIS_MIN, Y_AXIS_MAX);
            AppServo.controlMultiple(2, yAxisAngle);
            break;
        case 3: // Z axis right / Z 轴向右
            zAxisAngle = constrain(zAxisAngle + 1, Z_AXIS_MIN, Z_AXIS_MAX);
            AppServo.controlMultiple(1, zAxisAngle);
            break;
        case 4: // Z axis left / Z 轴向左
            zAxisAngle = constrain(zAxisAngle - 1, Z_AXIS_MIN, Z_AXIS_MAX);
            AppServo.controlMultiple(1, zAxisAngle);
            break;
        case 5: // Reset to center / 重置到中心
            yAxisAngle = CENTER_POSITION;
            zAxisAngle = CENTER_POSITION;
            AppServo.controlMultiple(2, CENTER_POSITION);
            AppServo.controlMultiple(1, CENTER_POSITION);
            break;
    }
}

// ========== Input Processing / 输入处理 ==========

void ApplicationFunctionSet::processKeyCommand() {
    static uint8_t previousKeyValue = 0;

    uint8_t currentKeyValue;
    AppKey.getKeyValue(&currentKeyValue);

    if (previousKeyValue != currentKeyValue) {
        previousKeyValue = currentKeyValue;
        
        switch (currentKeyValue) {
            case 0:
                currentState = State::Standby;
                break;
            case 1:
                currentState = State::TraceBased;
                break;
            case 2:
                currentState = State::ObstacleAvoidance;
                break;
            case 3:
                currentState = State::Follow;
                break;
            default:
                break;
        }
    }
}

void ApplicationFunctionSet::processIRCommand() {
    uint8_t irButton = 0;
    static unsigned long lastIRTime = 0;
    
    const unsigned long IR_TIMEOUT = 150; 

    // Receive and Process Command / 接收并处理命令
    if (AppIRrecv.getIRValue(&irButton)) {
        using IR = DeviceDriverSet_IRrecv; 
        
        switch (irButton) {
            // Movement Control / 移动控制
            case IR::UP:
            case IR::DOWN:
            case IR::LEFT:
            case IR::RIGHT:
                currentState = State::Rocker; 
                lastIRTime = millis();
                isIrRobotControl = true;
                
                if      (irButton == IR::UP)    carDirection = Forward;
                else if (irButton == IR::DOWN)  carDirection = Backward;
                else if (irButton == IR::LEFT)  carDirection = Left;
                else if (irButton == IR::RIGHT) carDirection = Right;
                break;

            // Mode Selection / 模式选择
            case IR::OK:    currentState = State::Standby;           break;
            case IR::NUM_1: currentState = State::TraceBased;        break;
            case IR::NUM_2: currentState = State::ObstacleAvoidance; break;
            case IR::NUM_3: currentState = State::Follow;            break;

            // Parameter Adjustment / 参数调整
            // Tracking Threshold / 循迹阈值
            case IR::NUM_4: 
                motionControl.TrackingDetection_THRESHOLD = min(motionControl.TrackingDetection_THRESHOLD + 10, 600); 
                break;
            case IR::NUM_5: 
                motionControl.TrackingDetection_THRESHOLD = 250; 
                break;
            case IR::NUM_6: 
                motionControl.TrackingDetection_THRESHOLD = max(motionControl.TrackingDetection_THRESHOLD - 10, 30); 
                break;

            // Car Speed / 小车速度
            case IR::NUM_7: 
                rockerCarSpeed = min(rockerCarSpeed + 5, 255); 
                break;
            case IR::NUM_8: 
                rockerCarSpeed = 250; 
                break;
            case IR::NUM_9: 
                rockerCarSpeed = max(rockerCarSpeed - 5, 50); 
                break;
                
            default:
                break;
        }
    }

    // Stop if timeout in Rocker mode / 摇杆模式超时停止
    if (currentState == State::Rocker && isIrRobotControl) {
        if (millis() - lastIRTime > IR_TIMEOUT) {
            currentState = State::Standby; 
            isIrRobotControl = false;
        }
    }
}

// ========== Command Processing / 命令处理 ==========

void ApplicationFunctionSet::analyzeSerialData(const JsonDocument &doc) {
    uint8_t commandMode = doc["N"];
    commandSerialNumber = doc["H"];

    switch (commandMode) {
        case 1: // Motor control mode / 电机控制模式
            processMotorControl(doc);
            break;
        case 2: // Car control mode / 小车控制模式
        case 3: // Car control with timer / 带定时器的小车控制
            handleCarControlCommand(doc);
            break;
        case 4: // Individual motor speed control / 独立电机速度控制
            {
                uint8_t leftSpeed = doc["D1"].as<uint8_t>();
                uint8_t rightSpeed = doc["D2"].as<uint8_t>();
                currentState = State::DoNothing;
                processMotorSpeedControl(leftSpeed, rightSpeed);
                sendCommandResponse(true);
            }
            break;
        case 5: // Servo control / 舵机控制
            {
                currentState = State::DoNothing;
                uint8_t angle = doc["D2"].as<uint8_t>() / 10;
                AppServo.control(angle, 0);
                sendCommandResponse(true);
            }
            break;
        case 7: // Lighting control / 灯光控制
        case 8: // Lighting control with timer / 带定时器的灯光控制
            handleLightingCommand(doc);
            break;
        case 21: // Ultrasonic sensor status / 超声波传感器状态
            {
                currentState = State::DoNothing;
                getUltrasonicStatus(doc["D1"].as<uint8_t>());
            }
            break;
        case 22: // Line tracking sensor status / 循迹传感器状态
            {
                currentState = State::DoNothing;
                getTrackingSensorStatus(doc["D1"].as<uint8_t>());
            }
            break;
        case 23: // Ground detection status / 地面检测状态
            {
                currentState = State::DoNothing;
                sendResponseData(isOnGround ? "false" : "true");
            }
            break;
        case 100: // Clear all functions - standby / 清除所有功能 - 待机
            {
                clearAllFunctions(State::Standby);
                sendCommandResponse(false);
            }
            break;
        case 101: // Mode switch command / 模式切换命令
            {
                uint8_t mode = doc["D1"];
                currentState = (mode == 1) ? State::TraceBased :
                              (mode == 2) ? State::ObstacleAvoidance :
                              (mode == 3) ? State::Follow : State::Standby;
                sendCommandResponse(false);
            }
            break;
        case 102: // Rocker control mode / 摇杆控制模式
            {
                uint8_t rockerDirection = doc["D1"];
                rockerCarSpeed = doc["D2"].as<uint8_t>();
                carDirection = static_cast<CarDirection>(rockerDirection);
                currentState = (carDirection == stop_it) ? State::Standby : State::Rocker;
                isIrRobotControl = false;
            }
            break;
        case 105: // LED brightness adjustment / LED 亮度调整
            {
                uint8_t adjustment = doc["D1"];
                int16_t brightness = FastLED.getBrightness();
                brightness += (adjustment == 1) ? 5 : (adjustment == 2) ? -5 : 0;
                FastLED.setBrightness(constrain(brightness, 0, 255));
            }
            break;
        case 106: // Servo control / 舵机控制
            {
                uint8_t servoCommand = doc["D1"].as<uint8_t>();
                controlServo(servoCommand);
                sendCommandResponse(false);
            }
            break;
        case 110: // Clear all functions - do nothing / 清除所有功能 - 不做任何操作
            {
                clearAllFunctions(State::DoNothing);
                sendCommandResponse(true);
            }
            break;
        default:
            break;
    }
}

// ========== Motor and Movement Control / 电机和移动控制 ==========

void ApplicationFunctionSet::processMotorControl(const JsonDocument &doc) {
    uint8_t motorSelection = doc["D1"].as<uint8_t>();
    uint8_t motorSpeed = doc["D2"].as<uint8_t>();
    uint8_t motorDirection = doc["D3"].as<uint8_t>();
    
    currentState = State::DoNothing;

    // Stop all motors if direction is 0 / 如果方向为 0，停止所有电机
    if (motorDirection == 0) {
        motorState.speedRight = 0;
        motorState.speedLeft = 0;
        AppMotor.control(motorState.directionRight, motorState.speedRight, 
                        motorState.directionLeft, motorState.speedLeft);
        sendCommandResponse(true);
        return;
    }

    // Determine new direction (true = forward, false = backward) / 确定新方向（true = 前进，false = 后退）
    bool newDirection = (motorDirection & 0x01) != 0;

    // Update motor state based on selection / 根据选择更新电机状态
    switch (motorSelection) {
        case 0: // Both motors / 两个电机
            motorState.directionRight = newDirection;
            motorState.speedRight = motorSpeed;
            motorState.directionLeft = newDirection;
            motorState.speedLeft = motorSpeed;
            break;
            
        case 1: // Left motor only / 仅左电机
            motorState.directionLeft = newDirection;
            motorState.speedLeft = motorSpeed;
            break;
            
        case 2: // Right motor only / 仅右电机
            motorState.directionRight = newDirection;
            motorState.speedRight = motorSpeed;
            break;
            
        default:
            sendCommandResponse(true);
            return;
    }

    // Apply the current motor state to hardware / 将当前电机状态应用到硬件
    AppMotor.control(motorState.directionRight, motorState.speedRight, 
                    motorState.directionLeft, motorState.speedLeft);
    sendCommandResponse(true);
}

void ApplicationFunctionSet::processMotorSpeedControl(uint8_t leftSpeed, uint8_t rightSpeed) {
    // Update motor state / 更新电机状态
    motorState.speedRight = rightSpeed;
    motorState.speedLeft = leftSpeed;
    
    if (leftSpeed == 0 && rightSpeed == 0) {
        // Both motors stopped - use existing directions / 两个电机停止 - 使用现有方向
        AppMotor.control(motorState.directionRight, motorState.speedRight, 
                        motorState.directionLeft, motorState.speedLeft);
    } else {
        // Motors running - assume forward direction for both / 电机运行 - 假设两个都为前进方向
        motorState.directionRight = true;
        motorState.directionLeft = true;
        AppMotor.control(motorState.directionRight, motorState.speedRight, 
                        motorState.directionLeft, motorState.speedLeft);
    }
}

void ApplicationFunctionSet::processCarControl(uint8_t direction, uint8_t speed) {
    switch (direction) {
        case 1: motionControl.SmartRobotCarMotionControl(Left, speed); break;
        case 2: motionControl.SmartRobotCarMotionControl(Right, speed); break;
        case 3: motionControl.SmartRobotCarMotionControl(Forward, speed); break;
        case 4: motionControl.SmartRobotCarMotionControl(Backward, speed); break;
        default: break;
    }
}

// ========== Command Handlers / 命令处理器 ==========

void ApplicationFunctionSet::handleCarControlCommand(const JsonDocument &doc) {
    currentState = State::DoNothing;
    uint8_t direction = doc["D1"].as<uint8_t>();
    uint8_t speed = doc["D2"].as<uint8_t>();
    
    processCarControl(direction, speed);
    
    if (doc["N"].as<uint8_t>() == 2) {
        // Timed command / 定时命令
        unsigned long duration = doc["T"].as<unsigned long>();
        startTimer(duration, this, &ApplicationFunctionSet::carControlCallback);
    } else {
        sendCommandResponse(true);
    }
}

void ApplicationFunctionSet::handleLightingCommand(const JsonDocument &doc) {
    currentState = State::DoNothing;
    uint8_t sequence = doc["D1"].as<uint8_t>();
    uint8_t red = doc["D2"].as<uint8_t>();
    uint8_t green = doc["D3"].as<uint8_t>();
    uint8_t blue = doc["D4"].as<uint8_t>();

    rgbControl.executeLightingCommand(sequence, red, green, blue);
    
    if (doc["N"].as<uint8_t>() == 7) {
        // Timed command / 定时命令
        unsigned long duration = doc["T"].as<unsigned long>();
        startTimer(duration, this, &ApplicationFunctionSet::lightingCallback);
    } else {
        sendCommandResponse(true);
    }
}

// ========== Sensor Status Functions / 传感器状态功能 ==========

void ApplicationFunctionSet::getUltrasonicStatus(uint8_t sensorId) {
    uint16_t distance;
    AppULTRASONIC.getDistance(&distance);
    bool obstacleDetected = isWithinRange(distance, 0, OBSTACLE_DETECTION_THRESHOLD);
    
    if (sensorId == 1) {
        // Return obstacle detection status / 返回障碍物检测状态
        sendResponseData(obstacleDetected ? "true" : "false");
    } else if (sensorId == 2) {
        // Return distance data / 返回距离数据
        char distanceStr[10];
        sprintf(distanceStr, "%d", distance);
        sendResponseData(distanceStr);
    }
}

void ApplicationFunctionSet::getTrackingSensorStatus(uint8_t sensorId) {
    char valueStr[5];
    volatile int sensorValue;
    
    switch (sensorId) {
        case 0: sensorValue = trackingDataLeft; break;
        case 1: sensorValue = trackingDataMiddle; break;
        case 2: sensorValue = trackingDataRight; break;
        default: sensorValue = 0; break;
    }
    
    sprintf(valueStr, "%d", sensorValue);
    sendResponseData(valueStr);
}

// ========== Utility Functions / 实用功能 ==========

void ApplicationFunctionSet::clearAllFunctions(State newState) {
    // Reset motor state / 重置电机状态
    motorState.directionRight = true;
    motorState.speedRight = 0;
    motorState.directionLeft = true;
    motorState.speedLeft = 0;
    
    // Stop motors and clear LEDs / 停止电机并清除 LED
    motionControl.SmartRobotCarMotionControl(stop_it, 0);
    AppRGB_LED.clear();
    carDirection = stop_it;
    currentState = newState;
    isIrRobotControl = false;
}

bool ApplicationFunctionSet::isWithinRange(long value, long start, long end) {
    return (value >= start && value <= end);
}

// ========== Communication Functions / 通信功能 ==========

void ApplicationFunctionSet::sendCommandResponse(bool includeSerialNumber) {
    if (includeSerialNumber && commandSerialNumber) {
        Serial.write("{");
        Serial.write(commandSerialNumber);
        Serial.write("_ok}");
    } 
}

void ApplicationFunctionSet::sendResponseData(const char* data) {
    Serial.write("{");
    Serial.write(commandSerialNumber);
    Serial.write("_");
    Serial.write(data);
    Serial.write("}");
}

// ========== Timer Functions / 定时器功能 ==========

void ApplicationFunctionSet::startTimer(unsigned long durationMs, ApplicationFunctionSet* instance, 
                                       void (ApplicationFunctionSet::*callback)()) {
    timerStart = millis();
    timerDuration = durationMs;
    timerInstance = instance;
    timerCallback = callback;
    isTimerActive = true;
}

void ApplicationFunctionSet::updateTimer() {
    if (isTimerActive && (millis() - timerStart >= timerDuration)) {
        isTimerActive = false;
        if (timerInstance && timerCallback) {
            (timerInstance->*timerCallback)();
        }
    }
}

// ========== Callback Functions / 回调功能 ==========

void ApplicationFunctionSet::carControlCallback() {
    motionControl.SmartRobotCarMotionControl(stop_it, 0);
    sendCommandResponse(true);
}

void ApplicationFunctionSet::lightingCallback() {
    AppRGB_LED.clear();
    sendCommandResponse(true);
}
