/*
 * @Author: ELEGOO
 * @Date: 2025-01-15 16:04:05
 * @LastEditTime: 2026-01-14 16:04:05
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 Motion Control Implementation / 智能机器人小车 V4.0 运动控制实现
 * This file implements the motion control logic for the smart robot car, including:
 * 本文件实现了智能机器人小车的运动控制逻辑，包括：
 * - Basic movement control with PID for straight line motion / 直线运动的 PID 基础运动控制
 * - Line tracking mode with weighted position calculation / 具有加权位置计算的巡线模式
 * - Obstacle avoidance with ultrasonic sensor / 超声波传感器避障
 * - Following mode with PID speed control / PID 速度控制的跟随模式
 */
#include "MotionControl.h"
#include <avr/wdt.h>
#include "Utils.h"

MotionControl::MotionControl(DeviceDriverSet_Motor &motor,
                             DeviceDriverSet_MPU6050 &mpu,
                             DeviceDriverSet_Ultrasonic &ultrasonic,
                             DeviceDriverSet_Servo &servo,
                             DeviceDriverSet_ITR20001 &itr20001)
    : AppMotor(motor),
      AppMPU6050(mpu),
      AppULTRASONIC(ultrasonic),
      AppServo(servo),
      AppITR20001(itr20001) 
{
    // Initialize tracking state / 初始化巡线状态
    trackingState.isLost = false;
    trackingState.searchDir = 1;
    trackingState.lostTime = 0;
    trackingState.lostCounter = 0;
}


//=============== Basic Motion Control / 基础运动控制 ===============//
/**
 * @brief Controls the robot car's motion in different directions / 控制机器人小车在不同方向上的运动
 * @param direction The target direction of motion / 目标运动方向
 * @param speed The target speed (PWM value 0-255) / 目标速度 (PWM 值 0-255)
 */
void MotionControl::SmartRobotCarMotionControl(CarDirection direction,uint8_t speed)
{
    static CarDirection PreDir = stop_it;

    // Map direction to corresponding motor control logic / 将方向映射到相应的电机控制逻辑
    switch (direction)
    {
    case Forward:
    case Backward:
    {
        SmartRobotCarLinearMotionControl(direction,PreDir, speed);
        break;
    }
    case Left:
        AppMotor.control(true, speed, false, speed);  // Left turn: left motor forward, right motor backward / 左转：左电机前进，右电机后退
        break;
    case Right:
    {
        AppMotor.control(false, speed, true, speed);  // Right turn: left motor backward, right motor forward / 右转：左电机后退，右电机前进
        break;
    }
    case LeftForward:
        AppMotor.control(true, speed, true, speed -60);  // Left-forward: left motor full speed, right motor half speed / 左前：左电机全速，右电机半速
        break;
    case LeftBackward:
        AppMotor.control(false, speed , false, speed-60);  // Left-backward: left motor half speed backward, right motor full speed / 左后：左电机半速后退，右电机全速
        break;
    case RightForward:
        AppMotor.control(true, speed -60, true, speed);  // Right-forward: left motor half speed, right motor full speed / 右前：左电机半速，右电机全速
        break;
    case RightBackward:
        AppMotor.control(false, speed -60, false, speed);  // Right-backward: left motor half speed, right motor full speed backward / 右后：左电机半速，右电机全速后退
        break;
    case stop_it:
        AppMotor.control(false, 0, false, 0);  // Stop: both motors off / 停止：两个电机都关闭
        break;
    default:
        break;
    }
    PreDir = direction;
}


/**
 * @brief Updates the target yaw angle for straight line motion / 更新直线运动的目标偏航角
 * Updates the target yaw from MPU6050 and resets the integral term / 从 MPU6050 更新目标偏航角并重置积分项
 */
void MotionControl::UpdateTargetYAW(){
    AppMPU6050.getEulerAngles(&TargetYaw);
    LinerMove_integral = 0;
}

/**
 * @brief Controls straight line motion using PID control / 使用 PID 控制控制直线运动
 * @param direction Current direction (Forward/Backward) / 当前方向（前进/后退）
 * @param predir Previous direction for detecting direction changes / 用于检测方向变化的上一个方向
 * @param speed Target speed (PWM value 0-255) / 目标速度 (PWM 值 0-255)
 * Uses MPU6050 yaw angle for maintaining straight line motion / 使用 MPU6050 偏航角保持直线运动
 */
void MotionControl::SmartRobotCarLinearMotionControl(CarDirection direction,CarDirection predir,uint8_t speed)
{
    static unsigned long last_control_time = 0;
    float yaw;
    unsigned long now = millis();
    unsigned long time_diff = now - last_control_time;
        
    // Update control loop every 10ms (100Hz) / 每 10ms (100Hz) 更新一次控制循环
    if (time_diff >= 10) {
        AppMPU6050.getEulerAngles(&yaw);
        yaw = fmod(yaw + 360.0, 360.0); // Normalize yaw to 0-360 range / 将偏航角标准化为 0-360 范围
    
        if (direction != predir)
        {
            TargetYaw = yaw;  // Set new target yaw when direction changes / 当方向改变时设置新的目标偏航角
            LinerMove_integral = 0;  // Reset integral term / 重置积分项
            LinerMove_previous_error = 0;  // Reset derivative history / 重置微分历史
        }

        // PID Control Calculation / PID 控制计算
        float LinerMove_error = TargetYaw - yaw;  
        // Normalize error to -180 to +180 range / 将误差标准化为 -180 到 +180 范围
        if (LinerMove_error > 180.0) {
            LinerMove_error -= 360.0;
        } else if (LinerMove_error < -180.0) {
            LinerMove_error += 360.0;
        }

        float delta_time = time_diff / 1000.0; // Dynamic delta time in seconds / 秒为单位的动态时间差
        
        // Only calculate integral if Ki > 0 to save computation / 仅当 Ki > 0 时计算积分以节省计算量
        if (LinerMoveKi > 0) {
            // Update integral term with time factor / 使用时间因子更新积分项
            LinerMove_integral += LinerMove_error * delta_time;
            // Limit integral windup (Optimized: Reduced range) / 限制积分饱和（优化：减小范围）
            LinerMove_integral = constrain(LinerMove_integral, -200.0, 200.0);
        }

        // Calculate derivative term / 计算微分项
        float derivative = (LinerMove_error - LinerMove_previous_error) / delta_time;

        // Calculate PID output / 计算 PID 输出
        float LinerMovePID_output = LinerMoveKp * LinerMove_error + LinerMoveKi * LinerMove_integral + LinerMoveKd * derivative;

        LinerMove_previous_error = LinerMove_error;
        
        // Adjust PID output direction for backward movement / 调整后退运动的 PID 输出方向
        // In Backward mode, the correction logic needs to be inverted because:
        // Forward: Left < Right -> Turns Left (CCW)
        // Backward: Left < Right -> Turns Right (CW)
        // So we flip the sign of the PID output to maintain the correct correction direction.
        if (direction == Backward) {
            LinerMovePID_output = -LinerMovePID_output;
        }

        // Adjust motor speeds based on PID output / 根据 PID 输出调整电机速度
        // Use intermediate float to avoid overflow / 使用中间浮点数以避免溢出

        int speedL = (int)(speed + LinerMovePID_output); 
        int speedR = (int)(speed - LinerMovePID_output);

        // Constrain PWM values to valid range / 将 PWM 值限制在有效范围内
        speedL = constrain(speedL, 60, 255);
        speedR = constrain(speedR, 60, 255);

        // Apply motor control / 应用电机控制
        // Fixed: Correctly map speedR to Right Motor and speedL to Left Motor / 修正：正确将 speedR 映射到右电机，speedL 映射到左电机
        AppMotor.control(
            direction == Forward,
            speedR,
            direction == Forward,
            speedL
        );

        // Update timestamp / 更新时间戳
        last_control_time = now;
    }
}

//=============== Line Tracking Mode / 巡线模式 ===============//
/**
 * @brief Main function for line tracking control / 巡线控制主函数
 * Reads line sensor values and implements PID control for line following / 读取巡线传感器值并实现巡线 PID 控制
 */
void MotionControl::HandleTrackingControl()
{
    static unsigned long last_control_time = 0;
    unsigned long now = millis();
    
    // Limit control frequency to 10ms (100Hz) to ensure PID calculation stability and prevent derivative term explosion due to small dt / 限制控制频率为 10ms (100Hz)，保证 PID 计算稳定性，避免 dt 过小导致微分项爆炸
    if (now - last_control_time >= 10) {
        last_control_time = now;
        
        // Calculate position error using Table Lookup / 使用查表法计算位置误差
        float position = calculateErrorByTable(AppITR20001.getAnalogueL(), AppITR20001.getAnalogueM(), AppITR20001.getAnalogueR());
      
        if(position != -100) {
            trackingState.isLost = false;
            processPIDControl(position);  // Apply PID control when line is detected / 当检测到线时应用 PID 控制
        } else {
            handleLineLost();  // Execute line recovery procedure / 执行巡线恢复程序
        }
    }
}

/**
 * @brief Calculates error using Table Lookup method / 使用查表法计算误差
 * @param left Left sensor value / 左传感器值
 * @param mid Middle sensor value / 中传感器值
 * @param right Right sensor value / 右传感器值
 * @return Error value (-2 to 2) or -100 if line is lost / 误差值 (-2 到 2) 或 -100 如果丢失线
 */
float MotionControl::calculateErrorByTable(int left, int mid, int right) {
    // Threshold judgment / 阈值判定
    bool l = left > TrackingDetection_THRESHOLD;
    bool m = mid > TrackingDetection_THRESHOLD;
    bool r = right > TrackingDetection_THRESHOLD;

    // Table lookup logic / 查表逻辑
    if (!l && m && !r) return 0;        // 0 1 0: Center / 正中
    if (!l && m && r) return -1;        // 0 1 1: Right deviation / 偏右
    if (!l && !m && r) return -2;       // 0 0 1: Extreme right / 极右
    if (l && m && !r) return 1;         // 1 1 0: Left deviation / 偏左
    if (l && !m && !r) return 2;        // 1 0 0: Extreme left / 极左
    
    if (l && m && r) return 0;          // 1 1 1: Intersection / 十字路口
    
    if (!l && !m && !r) return -100;    // 0 0 0: Line lost / 丢线
    
    return 0;
}

/**
 * @brief Implements PID control for line tracking / 实现巡线 PID 控制
 * @param error Position error from line center / 偏离中心线的误差
 * Adjusts motor speeds based on PID calculation / 根据 PID 计算调整电机速度
 */
void MotionControl::processPIDControl(float error) {
    unsigned long currentTime = millis();
    static unsigned long lastTime = 0;
    
    // Handle first run or large time gaps / 处理首次运行或大时间间隔
    if (lastTime == 0 || currentTime < lastTime || (currentTime - lastTime) > 100) {
        lastTime = currentTime;
        TRACKING_lastError = error;
        return;
    }
    
    float deltaTime = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    
    // Calculate derivative term / 计算微分项
    float raw_derivative = (error - TRACKING_lastError) / deltaTime;
    static float smooth_derivative = 0;
    smooth_derivative = 0.6 * raw_derivative + 0.4 * smooth_derivative;
    
    TRACKING_lastError = error;
    
    // Calculate PID output (PD Control) / 计算 PID 输出 (PD 控制)
    float output = KP * error + KD * smooth_derivative;

    // Dynamic speed adjustment / 动态速度调整
    float dynamicBaseSpeed = TRACKING_BASE_SPEED;
    if (abs(error) >= 1.5) {
        dynamicBaseSpeed = TRACKING_BASE_SPEED * 0.7;
    } else if (abs(error) >= 1.0) {
        dynamicBaseSpeed = TRACKING_BASE_SPEED * 0.85;
    }

    // Convert PID output to motor speeds / 将 PID 输出转换为电机速度
    int leftSpeed = dynamicBaseSpeed - output;
    int rightSpeed = dynamicBaseSpeed + output;

    // Constrain speeds within valid range / 将速度限制在有效范围内
    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
    
    // Deadzone Compensation / 死区补偿
    if (abs(leftSpeed) < MIN_SPEED && abs(leftSpeed) > 5) {
        leftSpeed = (leftSpeed > 0) ? MIN_SPEED : -MIN_SPEED;
    }
    if (abs(rightSpeed) < MIN_SPEED && abs(rightSpeed) > 5) {
        rightSpeed = (rightSpeed > 0) ? MIN_SPEED : -MIN_SPEED;
    }

    // AppMotor.control(direction_right, speed_right, direction_left, speed_left)
    AppMotor.control(rightSpeed > 0, abs(rightSpeed), leftSpeed > 0, abs(leftSpeed));
}

/**
 * @brief Handles line lost situation / 处理丢线情况
 * Implements search pattern when line is lost / 当丢失线时实现搜索模式
 * Uses time-based stages for different search behaviors / 使用基于时间的阶段进行不同的搜索行为
 */
void MotionControl::handleLineLost()
{
    if (!trackingState.isLost) {
        trackingState.isLost = true;
        trackingState.lostTime = millis();
        // If error > 0 (Line on right), search right (searchDir = 1) / 如果 error > 0 (线在右边)，向右搜索 (searchDir = 1)
        // If error < 0 (Line on left), search left (searchDir = -1) / 如果 error < 0 (线在左边)，向左搜索 (searchDir = -1)
        trackingState.searchDir = (TRACKING_lastError > 0) ? 1 : -1;
    }
    uint32_t lostDuration = millis() - trackingState.lostTime;
    
    // Search speed: Ensure it is greater than startup speed / 搜索速度：确保大于启动速度
    uint8_t searchSpeed = MIN_SPEED + 10; 
    
    if (lostDuration < 1500) {    // Stage 1: Rotate in place / 第一阶段：原位旋转搜索
        // searchDir = 1 (Turn Right): Right motor back (false), Left motor forward (true) / searchDir = 1 (右转): 右轮后退(false), 左轮前进(true)
        // searchDir = -1 (Turn Left): Right motor forward (true), Left motor back (false) / searchDir = -1 (左转): 右轮前进(true), 左轮后退(false)
        AppMotor.control(trackingState.searchDir == -1, searchSpeed, trackingState.searchDir == 1, searchSpeed);
    } else if (lostDuration < 3000) {  // Stage 2: Wide range search / 第二阶段：加大范围搜索
        searchSpeed += 20; 
        AppMotor.control(trackingState.searchDir == -1, searchSpeed, trackingState.searchDir == 1, searchSpeed);
    } else {
        // Not found yet? Reverse search direction / 还没找到？反向搜索
        trackingState.searchDir *= -1;
        trackingState.lostTime = millis();
    }
}

//=============== Obstacle Avoidance Mode / 避障模式 ===============//
/**
 * @brief Main function for obstacle avoidance (State Machine Version) / 避障模式主函数（状态机版本）
 * Uses state machine approach for better responsiveness and control / 使用状态机方法以获得更好的响应性和控制
 * Replaces blocking delays with non-blocking state transitions / 用非阻塞状态转换替换阻塞延迟
 */
void MotionControl::HandleObstacleAvoidance()
{
    HandleObstacleAvoidanceStateMachine();
}

/**
 * @brief Obstacle Avoidance State Machine / 避障状态机
 */
void MotionControl::HandleObstacleAvoidanceStateMachine()
{
    // State variables (Static locals to match Follow Mode style)
    // 状态变量（静态局部变量，以匹配跟随模式的风格）
    static AvoidanceState state = SCANNING;
    static unsigned long stateStartTime = 0;
    static uint16_t currentTurnDuration = 0;

    static uint8_t scanStep = 0;
    static unsigned long scanTimer = 0;
    static uint16_t leftDist = 0;
    static uint16_t rightDist = 0;

    unsigned long currentTime = millis();
    
    // Static variables for periodic distance checking
    // 用于定期距离检查的静态变量
    static unsigned long lastCheckTime = 0;
    static uint16_t currentDistance = 200; 
    
    // Only read distance if in MOVING_FORWARD and interval has passed
    // 仅在前进状态且时间间隔已过时读取距离
    if (state == MOVING_FORWARD) {
        if (currentTime - lastCheckTime > 50) { // Check every 50ms / 每50ms检测一次
            lastCheckTime = currentTime;
            uint16_t tempDist = 0;
            AppULTRASONIC.getDistance(&tempDist);
            if (tempDist > 0) { // Filter invalid readings / 过滤无效读数
                currentDistance = tempDist;
            } else {
                currentDistance = 200; // Treat 0 as safe/far / 将 0 视为安全/远距离
            }
        }
    } else {
        // Reset distance when not moving forward to prevent false trigger upon return
        // 当不前进时重置距离，以防止返回时误触发
        currentDistance = 200; 
    }

    switch (state) {
        case MOVING_FORWARD:
            if (currentDistance < AvoidanceConfig::WARNING_DISTANCE) {
                // Obstacle detected, stop and start scanning / 检测到障碍物，停止并开始扫描
                SmartRobotCarMotionControl(stop_it, 0); // Stop / 停止
                state = SCANNING;
                scanStep = 0;
            } else {
                // No obstacle, move forward / 无障碍物，前进
                SmartRobotCarMotionControl(Forward, 130);
            }
            break;

        case SCANNING:
            // Non-blocking scan sequence / 非阻塞扫描序列
            switch (scanStep) {
                case 0: // Scan Right (30 degrees) / 扫描右侧 (30度)
                    AppServo.setZAngle(30);
                    scanTimer = currentTime;
                    scanStep = 1;
                    break;
                    
                case 1: // Wait 300ms, Read Right, Scan Left (150 degrees) / 等待300ms，读取右侧，扫描左侧 (150度)
                    if (currentTime - scanTimer > 300) {
                        AppULTRASONIC.getDistance(&rightDist);
                        
                        AppServo.setZAngle(150);
                        scanTimer = currentTime;
                        scanStep = 2;
                    }
                    break;
                    
                case 2: // Wait 300ms, Read Left, Return to Center (90 degrees) / 等待300ms，读取左侧，回到中间 (90度)
                    if (currentTime - scanTimer > 300) {
                        AppULTRASONIC.getDistance(&leftDist);
                        
                        AppServo.setZAngle(90);
                        scanTimer = currentTime;
                        scanStep = 3;
                    }
                    break;
                    
                case 3: // Wait 200ms, Analyze and Move / 等待200ms，分析并移动
                    if (currentTime - scanTimer > 200) {
                        // Treat 0 as "Very Far" / 将0视为“非常远”
                        if (rightDist == 0) rightDist = 200;
                        if (leftDist == 0) leftDist = 200;

                        if (rightDist > leftDist && rightDist > AvoidanceConfig::DANGER_DISTANCE) {
                            // Right is clearer / 右边更空旷
                            state = TURNING_RIGHT;
                            currentTurnDuration = AvoidanceConfig::TURN_DURATION;
                        } else if (leftDist >= rightDist && leftDist > AvoidanceConfig::DANGER_DISTANCE) {
                            // Left is clearer (or equal) / 左边更空旷（或相等）
                            state = TURNING_LEFT;
                            currentTurnDuration = AvoidanceConfig::TURN_DURATION;
                        } else {
                            // Both sides blocked or too close -> Back up and turn around / 两边都被堵死或太近 -> 后退并掉头
                            state = BACKING_UP;
                        }
                        stateStartTime = currentTime;
                    }
                    break;
            }
            break;

        case TURNING_LEFT:
            SmartRobotCarMotionControl(Left, 130);
            if (currentTime - stateStartTime > currentTurnDuration) {
                state = MOVING_FORWARD;
            }
            break;

        case TURNING_RIGHT:
            SmartRobotCarMotionControl(Right, 130);
            if (currentTime - stateStartTime > currentTurnDuration) {
                state = MOVING_FORWARD;
            }
            break;

        case BACKING_UP:
            SmartRobotCarMotionControl(Backward, 130);
            if (currentTime - stateStartTime > AvoidanceConfig::BACKUP_DURATION) {
                // After backing up, turn right / 后退后，右转
                state = TURNING_RIGHT;
                stateStartTime = currentTime;
                currentTurnDuration = AvoidanceConfig::BACKUP_TURN_DURATION;
            }
            break;
            
        default:
            break;
    }
}


//=============== Following Mode / 跟随模式 ===============//
/*
 * @brief Follow Mode / 跟随模式
 * Logic: 1. Check Center -> Follow; 2. If Empty -> Scan & Find Best Angle -> Turn / 逻辑：1. 检查中心 -> 跟随；2. 如果为空 -> 扫描并找到最佳角度 -> 转向
 */
void MotionControl::HandleFollowMode()
{
    static uint16_t ULTRASONIC_Get = 0; 
    static unsigned long check_timer = 0; 
    static unsigned long scan_timer = 0; 
    static uint8_t scan_step = 0; // 0: Center, 1: Left, 2: Center, 3: Right / 0: 中, 1: 左, 2: 中, 3: 右
    static boolean is_tracking = false; 
    static const uint8_t scan_angles[] = {90, 150, 90, 30}; // Center, Left, Center, Right / 中, 左, 中, 右

    if (is_tracking) 
    { 
        // Tracking Mode: Only check front / 追踪模式：仅检查前方
        if (millis() - check_timer > 100) 
        { 
            check_timer = millis(); 
            AppULTRASONIC.getDistance(&ULTRASONIC_Get); 
            
            // Check if target is present (range 1-20cm) / 检查目标是否存在（范围 1-20cm）
            if (ULTRASONIC_Get >= 1 && ULTRASONIC_Get <= 20) 
            { 
                // Target confirmed: Keep moving forward / 目标确认：保持前进
                SmartRobotCarMotionControl(Forward, 100); 
                // Ensure servo is looking forward / 确保舵机向前看
                AppServo.setZAngle(90); 
            } 
            else 
            { 
                // Target lost: Stop and switch to scanning / 目标丢失：停止并切换到扫描
                is_tracking = false; 
                SmartRobotCarMotionControl(stop_it, 0); 
                
                // Reset scan sequence / 重置扫描序列
                scan_step = 0; 
                scan_timer = millis(); 
                AppServo.setZAngle(scan_angles[scan_step]); 
            } 
        } 
    } 
    else 
    { 
        // Scanning Mode: Center -> Left -> Center -> Right / 扫描模式：中 -> 左 -> 中 -> 右
        if (millis() - scan_timer > 300) // Wait for servo to settle / 等待舵机稳定
        { 
            scan_timer = millis(); 
            
            // Read ultrasonic distance at current servo position / 读取当前舵机位置的超声波距离
            AppULTRASONIC.getDistance(&ULTRASONIC_Get); 
            
            if (ULTRASONIC_Get >= 1 && ULTRASONIC_Get <= 20) 
            { 
                // Target Found / 发现目标
                if (scan_step == 0 || scan_step == 2) // Found at Center / 在中心发现
                { 
                    is_tracking = true; 
                    check_timer = millis(); 
                    SmartRobotCarMotionControl(Forward, 100); 
                    return; 
                } 
                else if (scan_step == 1) // Found at Left (150) / 在左侧发现 (150)
                { 
                    SmartRobotCarMotionControl(Left, 150); 
                    delay(100); // Turn Left 100ms / 左转 100ms
                    SmartRobotCarMotionControl(stop_it, 0); 
                } 
                else if (scan_step == 3) // Found at Right (30) / 在右侧发现 (30)
                { 
                    SmartRobotCarMotionControl(Right, 150); 
                    delay(100); // Turn Right 100ms / 右转 100ms
                    SmartRobotCarMotionControl(stop_it, 0); 
                } 
            } 
            
            // Move to next scan position / 移动到下一个扫描位置
            scan_step++; 
            if (scan_step > 3) scan_step = 0; 
            AppServo.setZAngle(scan_angles[scan_step]); 
        } 
    } 
}




