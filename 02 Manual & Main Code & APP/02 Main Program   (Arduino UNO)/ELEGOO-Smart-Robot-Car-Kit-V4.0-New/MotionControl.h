#ifndef _MOTION_CONTROL_H_
#define _MOTION_CONTROL_H_

#include <Arduino.h>
#include "DeviceDriverSet.h"

// Robot car motion control enumeration / 机器人运动控制枚举
enum CarDirection:uint8_t  {
    Forward = 1,       // Forward / 前进
    Backward = 2,      // Backward / 后退
    Left = 3,          // Left / 左转
    Right = 4,         // Right / 右转
    LeftForward = 5,   // Left Forward / 左前
    LeftBackward = 6,  // Left Backward / 左后   
    RightForward = 7,  // Right Forward / 右前
    RightBackward = 8, // Right Backward / 右后
    stop_it = 9        // Stop / 停止
};

/**
 * @brief Motion Control Class / 运动控制类
 */
class MotionControl {
public:
    /**
     * @brief Constructor / 构造函数
     * @param motor Motor driver instance / 电机驱动实例
     * @param mpu MPU6050 sensor instance / MPU6050 传感器实例
     * @param ultrasonic Ultrasonic sensor instance / 超声波传感器实例
     * @param servo Servo driver instance / 舵机驱动实例
     * @param itr20001 Line tracking sensor instance / 循迹传感器实例
     */
    MotionControl(DeviceDriverSet_Motor &motor, 
                 DeviceDriverSet_MPU6050 &mpu,
                 DeviceDriverSet_Ultrasonic &ultrasonic,
                 DeviceDriverSet_Servo &servo,
                 DeviceDriverSet_ITR20001 &itr20001);

    // Public control interfaces / 公共控制接口
    void SmartRobotCarMotionControl(CarDirection direction,uint8_t speed);
    void SmartRobotCarLinearMotionControl(CarDirection direction, CarDirection predir,uint8_t speed);
    
    // Functional mode handling interfaces / 功能模式处理接口
    void HandleTrackingControl();                    // Line tracking mode / 巡线模式
    void HandleObstacleAvoidance();                  // Obstacle avoidance mode / 避障模式
    void HandleFollowMode();                         // Follow mode / 跟随模式
    void UpdateTargetYAW();

    uint16_t TrackingDetection_THRESHOLD = 400;      // Black/White line threshold / 黑白线阈值

private:
    // Hardware device references / 硬件设备引用
    DeviceDriverSet_Motor &AppMotor;
    DeviceDriverSet_MPU6050 &AppMPU6050;
    DeviceDriverSet_Ultrasonic &AppULTRASONIC;
    DeviceDriverSet_Servo &AppServo;
    DeviceDriverSet_ITR20001 &AppITR20001;


    //=============== Straight Line Motion PID / 直线运动 PID ===============//
    // PID Tuning Guide / PID调试指南:
    // P (Proportional): Determines response speed. Too large causes oscillation; too small causes slow response.
    // P (比例): 决定响应速度。P过大车身会剧烈抖动(震荡)，P过小修正无力(响应慢)。
    // D (Derivative): Dampens oscillation. Too large makes it sensitive to noise; too small causes overshoot.
    // D (微分): 抑制震荡(阻尼)。D过大对噪声敏感(抖动)，D过小会有超调(刹不住)。
    // I (Integral): Eliminates steady-state error. Too large causes overshoot and oscillation.
    // I (积分): 消除静差。I过大容易产生超调和震荡，一般直线走偏时微调。
    const float LinerMoveKp = 15; // Proportional coefficient / 比例系数
    const float LinerMoveKd = 0.5;  // Derivative coefficient / 微分系数
    const float LinerMoveKi = 0.2; // Integral coefficient / 积分系数
    float LinerMove_previous_error;  
    float LinerMove_integral;
    float TargetYaw = 0;

    //=============== Obstacle Avoidance Mode / 避障模式相关 ===============//
    // Avoidance state machine enumeration / 避障状态机枚举
    enum AvoidanceState {
        SCANNING = 0,           // Scanning state / 扫描状态
        MOVING_FORWARD,         // Moving forward state / 前进状态  
        TURNING_LEFT,           // Turning left state / 左转状态
        TURNING_RIGHT,          // Turning right state / 右转状态
        BACKING_UP              // Backing up state / 后退状态
    };

    // Avoidance configuration parameters / 避障配置参数
    struct AvoidanceConfig {
        static constexpr uint8_t DANGER_DISTANCE = 20;     // Danger distance / 危险距离 (20cm)
        static constexpr uint8_t WARNING_DISTANCE = 25;    // Warning distance / 警告距离 (25cm)
        static constexpr uint16_t TURN_DURATION = 400;     // Turn duration (ms) / 转弯持续时间(ms)
        static constexpr uint16_t BACKUP_DURATION = 500;   // Backup duration (ms) / 后退持续时间(ms)
        static constexpr uint16_t BACKUP_TURN_DURATION = 500; // Backup turn duration (ms) / 后退后转弯持续时间(ms)
    };

    // Avoidance private methods / 避障私有方法
    void HandleObstacleAvoidanceStateMachine();

    //=============== Follow Mode / 跟随模式相关 ===============//
    static constexpr uint8_t FOLLOW_IDEAL = 25; // Ideal distance / 理想距离
    static constexpr uint8_t FOLLOW_MIN = 10;   // Min distance / 最小距离
    static constexpr uint8_t FOLLOW_LOST = 60;  // Lost distance / 丢失距离
    
    enum FollowState {
        FOLLOW_IDLE = 0,
        FOLLOW_TRACKING, 
        FOLLOW_SCANNING,
        FOLLOW_TURNING
    };

    struct {
        FollowState state = FOLLOW_IDLE;
        uint16_t lastDistance = 0;
        unsigned long lastScanTime = 0;
        bool obstacleDetected = false;
        
        // PID Parameters / PID参数
        float Kp = 5.0;   // Proportional / 比例
        float Ki = 0.1;   // Integral / 积分
        float Kd = 2.0;   // Derivative / 微分
        float integral = 0;
        float prevError = 0;
        long lastTime = 0;
        
        // Scan state / 扫描状态
        uint8_t scanIndex = 0;
        int targetAngle = 90;
    } followController;

    static constexpr uint8_t MAX_SPEED = 180;     // Max PWM / 最大PWM
    static constexpr uint8_t MIN_SPEED = 70;      // Min PWM / 最小PWM

    //=============== Line Tracking Mode / 巡线模式相关 ===============//
    
    static constexpr uint8_t TRACKING_BASE_SPEED = 80;   // Base speed / 基础速度

    float TRACKING_lastError = 0;
    
    const float KP = 40;    // Proportional / 比例
    const float KD = 0.8;   // Derivative / 微分
    const float KI = 0;     // Integral / 积分
    struct TrackingState {
        
        bool isLost;          // Line lost / 丢线
        int8_t searchDir;     // Search direction / 搜索方向
        uint32_t lostTime;    // Lost time / 丢线时间
        uint8_t lostCounter;  // Lost counter / 丢线计数器
    } trackingState;
    
    // Tracking private methods / 巡线私有方法
    void handleLineLost();                         // Handle line lost / 处理丢线
    float calculateErrorByTable(int left, int mid, int right); // Table lookup / 查表法
    void processPIDControl(float error);
};

#endif