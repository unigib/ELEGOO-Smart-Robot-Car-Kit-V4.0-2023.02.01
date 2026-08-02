/*
 * @Author: ELEGOO
 * @Date: 2025-01-14 11:59:09
 * @LastEditTime: 2026-01-14 16:07:48
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - Hardware Device Drivers Implementation / 智能机器人小车 V4.0 - 硬件设备驱动实现
 */
#include "DeviceDriverSet.h"
// Only enable NEC protocol to save flash and RAM / 仅启用 NEC 协议以节省 Flash 和 RAM
#define DECODE_NEC
#include <IRremote.hpp>
#include <Arduino.h>
#include "Utils.h"
#include <MPU6050.h>
#include "Wire.h"

static MPU6050 accelgyro;

/*================ MPU6050 Driver / MPU6050 驱动 =================*/

bool DeviceDriverSet_MPU6050::init(void)
{
  Wire.begin();
  uint8_t chip_id = 0x00;
  uint8_t cout = 0;
  do
  {
    chip_id = accelgyro.getDeviceID();
    delay(10);
    cout += 1;
    if (cout > 10)
    {
      return true;
    }
  } while (chip_id == 0X00 || chip_id == 0XFF); //Ensure that the slave device is online（Wait forcibly to get the ID） / 确保从设备在线（强制等待获取 ID）
  accelgyro.initialize();
  return false;
}

void DeviceDriverSet_MPU6050::calibration(void)
{
  unsigned short times = 100; //Sampling times / 采样次数
  for (int i = 0; i < times; i++)
  {
    gz = accelgyro.getRotationZ();
    gzo += gz;
  }
  gzo /= times; //Calculate gyroscope offset / 计算陀螺仪偏移
}

void DeviceDriverSet_MPU6050::getEulerAngles(float *Yaw)
{
  unsigned long now = millis();           //Record the current time(ms) / 记录当前时间(ms)
  dt = (now - lastTime) / 1000.0;         //Caculate the derivative time(s) / 计算微分时间(s)
  lastTime = now;                         //Record the last sampling time(ms) / 记录最后一次采样时间(ms)
  gz = accelgyro.getRotationZ();          //Read the raw values of the six axes / 读取六轴原始值
  float gyroz = -(gz - gzo) / 131.0 * dt; //z-axis angular velocity / Z 轴角速度
  if (fabs(gyroz) < 0.05)                 //Clear instant zero drift signal / 清除瞬时零漂信号
  {
    gyroz = 0.00;
  }
  agz += gyroz; //z-axis angular velocity integral / Z 轴角速度积分
  *Yaw = agz;
}

/*================ RGB LED Driver / RGB LED 驱动 =================*/

/**
 * @brief Initialize the RGB LED module / 初始化 RGB LED 模块
 * @param brightness Initial brightness level (0-255) / 初始亮度级别 (0-255)
 */
void DeviceDriverSet_RGBLED::init(uint8_t brightness)
{
    FastLED.addLeds<NEOPIXEL, PIN_RGBLED>(leds_, NUM_LEDS);
    FastLED.setBrightness(brightness);
}

/**
 * @brief Set multiple RGB LEDs to the same color / 将多个 RGB LED 设置为相同颜色
 * @param Traversal_Number Number of LEDs to set (from index 0) / 要设置的 LED 数量（从索引 0 开始）
 * @param colour The color to set for all specified LEDs / 为所有指定 LED 设置的颜色
 */
void DeviceDriverSet_RGBLED::SetRBGLED(uint8_t Traversal_Number, CRGB colour)
{
  if (NUM_LEDS < Traversal_Number)
  {
    Traversal_Number = NUM_LEDS;
  }
  for (int Number = 0; Number < Traversal_Number; Number++)
  {
    leds_[Number] = colour;
    FastLED.show();
  }
}

/**
 * @brief Set a specific LED to a custom RGB color / 将特定 LED 设置为自定义 RGB 颜色
 * @param ledIndex Index of the LED to set (if >= NUM_LEDS, sets all LEDs) / 要设置的 LED 索引（如果 >= NUM_LEDS，则设置所有 LED）
 * @param r Red component (0-255) / 红色分量 (0-255)
 * @param g Green component (0-255) / 绿色分量 (0-255)
 * @param b Blue component (0-255) / 蓝色分量 (0-255)
 */
void DeviceDriverSet_RGBLED::setColor(uint8_t ledIndex, uint8_t r, uint8_t g, uint8_t b)
{
    if (ledIndex >= NUM_LEDS)
    {
        FastLED.showColor(CRGB(r, g, b)); // Set all LEDs to the same color / 将所有 LED 设置为相同颜色
    }
    else
    {
        leds_[ledIndex] = CRGB(r, g, b); // Set only the specified LED / 仅设置指定的 LED
    }
    FastLED.show();
}

/**
 * @brief Clear all RGB LEDs (turn them off) / 清除所有 RGB LED（关闭它们）
 */
void DeviceDriverSet_RGBLED::clear()
{
    FastLED.clear(true);
}

/*================ Key/Button Detection / 按键检测 =================*/
// Initialize static member / 初始化静态成员

/**
 * @brief Static variable to store the current key value / 存储当前按键值的静态变量
 */
uint8_t DeviceDriverSet_Key::keyValue_ = 0;

/**
 * @brief Interrupt handler for button press detection / 按键按下检测的中断处理程序
 * Implements debouncing and increments the key value when a valid press is detected / 实现去抖动并在检测到有效按下时递增按键值
 */
static void attachPinChangeInterrupt_GetKeyValue(void)
{
    static uint32_t lastDebounceTime = 0;
    const uint32_t debounceDelay = 500; // milliseconds / 毫秒

    // Read the state of the button / 读取按键状态
    int buttonState = digitalRead(DeviceDriverSet_Key::PIN_KEY);

    // Check if the button is pressed (taking into account the pull-up mode) / 检查按键是否被按下（考虑到上拉模式）
    if (buttonState == LOW) {
        uint32_t currentTime = millis();

        // Check if the current time is at least debounceDelay ms greater than the last debounce time / 检查当前时间是否比上次去抖动时间至少大 debounceDelay 毫秒
        if ((currentTime - lastDebounceTime) > debounceDelay) {
            // Update the last debounce time / 更新上次去抖动时间
            lastDebounceTime = currentTime;

            // Increment key value and wrap around if necessary / 递增按键值并在必要时回绕
            DeviceDriverSet_Key::keyValue_++;
            if (DeviceDriverSet_Key::keyValue_ > DeviceDriverSet_Key::KEY_VALUE_MAX) {
                DeviceDriverSet_Key::keyValue_ = 0;
            }
        }
    }
}

/**
 * @brief Initialize the key/button detection module / 初始化按键检测模块
 */
void DeviceDriverSet_Key::init()
{
    pinMode(PIN_KEY, INPUT_PULLUP);
    attachInterrupt(0, attachPinChangeInterrupt_GetKeyValue, FALLING); // Using interrupt 0 / 使用中断 0
}

/**
 * @brief Get the current key/button value / 获取当前按键值
 * @param get_keyValue Pointer to store the current key value / 用于存储当前按键值的指针
 */
void DeviceDriverSet_Key::getKeyValue(uint8_t *get_keyValue)
{
  *get_keyValue = keyValue_;
}

/*================ ITR20001 Line Tracking Sensor / ITR20001 循迹传感器 =================*/

/**
 * @brief Initialize the ITR20001 line tracking sensors / 初始化 ITR20001 循迹传感器
 */
void DeviceDriverSet_ITR20001::init()
{
    pinMode(PIN_ITR_L, INPUT);
    pinMode(PIN_ITR_M, INPUT);
    pinMode(PIN_ITR_R, INPUT);
}

/**
 * @brief Get the analog value from the left line tracking sensor / 获取左侧循迹传感器的模拟值
 * @return Analog value (0-1023) where higher values indicate lighter surface / 模拟值 (0-1023)，值越高表示表面越亮
 */
int DeviceDriverSet_ITR20001::getAnalogueL() const
{
    return analogRead(PIN_ITR_L);
}

/**
 * @brief Get the analog value from the middle line tracking sensor / 获取中间循迹传感器的模拟值
 * @return Analog value (0-1023) where higher values indicate lighter surface / 模拟值 (0-1023)，值越高表示表面越亮
 */
int DeviceDriverSet_ITR20001::getAnalogueM() const
{
    return analogRead(PIN_ITR_M);
}

/**
 * @brief Get the analog value from the right line tracking sensor / 获取右侧循迹传感器的模拟值
 * @return Analog value (0-1023) where higher values indicate lighter surface / 模拟值 (0-1023)，值越高表示表面越亮
 */
int DeviceDriverSet_ITR20001::getAnalogueR() const
{
    return analogRead(PIN_ITR_R);
}


/*================ Battery Voltage Detection / 电池电压检测 =================*/

/**
 * @brief Initialize the voltage detection module / 初始化电压检测模块
 */
void DeviceDriverSet_Voltage::init()
{
    pinMode(PIN_VOLTAGE, INPUT);
}

/**
 * @brief Get the current battery voltage / 获取当前电池电压
 * @return Battery voltage in volts / 电池电压（伏特）
 */
float DeviceDriverSet_Voltage::getVoltage() const
{
    float voltage = analogRead(PIN_VOLTAGE) * 0.0375;
    voltage += voltage * 0.08; // Add 8% compensation / 增加 8% 补偿
    return voltage;
}


/*================ DC Motor Control / 直流电机控制 =================*/

/**
 * @brief Initialize the motor control module / 初始化电机控制模块
 */
void DeviceDriverSet_Motor::init()
{
    pinMode(PIN_PWM_RIGHT, OUTPUT);
    pinMode(PIN_PWM_LEFT, OUTPUT);
    pinMode(PIN_RIGHT_IN, OUTPUT);
    pinMode(PIN_LEFT_IN, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);
}


void DeviceDriverSet_Motor::control(bool directionRight, uint8_t speedRight,
                                    bool directionLeft, uint8_t speedLeft)
{
    digitalWrite(PIN_STBY, HIGH); // Enable motor driver / 启用电机驱动器
    
    // Motor Right / 右电机
    digitalWrite(PIN_RIGHT_IN, directionRight ? HIGH : LOW); 
    analogWrite(PIN_PWM_RIGHT, speedRight);
    
    // Motor Left / 左电机
    digitalWrite(PIN_LEFT_IN, directionLeft ? HIGH : LOW); 
    analogWrite(PIN_PWM_LEFT, speedLeft);

    // If both motors have zero speed, enter standby mode to save power / 如果两个电机速度都为零，则进入待机模式以节省电力
    if (speedRight == 0 && speedLeft == 0) {
        digitalWrite(PIN_STBY, LOW); 
    }
}

/*================ Ultrasonic Distance Sensor / 超声波距离传感器 =================*/
/**
 * @brief Initialize the ultrasonic distance sensor / 初始化超声波距离传感器
 */
void DeviceDriverSet_Ultrasonic::init()
{
    pinMode(ECHO_PIN, INPUT); //Ultrasonic module initialization / 超声波模块初始化
    pinMode(TRIG_PIN, OUTPUT);
}

/**
 * @brief Measure distance using the ultrasonic sensor / 使用超声波传感器测量距离
 * @param distance Pointer to store the measured distance in centimeters / 用于存储测量距离（厘米）的指针
 */
void DeviceDriverSet_Ultrasonic::getDistance(uint16_t *distance) const
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Measure the width of the echo pulse in microseconds / 测量回声脉冲的宽度（微秒）
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 15000);
        
    // Calculate distance in centimeters / 计算距离（厘米）
    *distance = duration / 58;  // Conversion formula: cm = microseconds / 58 / 转换公式：cm = 微秒 / 58

    // Handle timeout condition / 处理超时情况
    if (duration == 0) {
        *distance = 150;  // Return maximum valid distance on timeout / 超时时返回最大有效距离
    }    

    // Limit to valid range / 限制在有效范围内
    if(*distance > 150) {        // Maximum valid distance: 150cm / 最大有效距离：150cm
        *distance = 150;
    }
    else if(*distance < 2) {     // Minimum valid distance: 2cm / 最小有效距离：2cm
        *distance = 0;
    }
}


/*================ Servo Motor Control / 舵机控制 =================*/

/**
 * @brief Initialize the servo motors / 初始化舵机
 * @param positionAngle Initial position angle (0-180 degrees) / 初始位置角度（0-180 度）
 */
void DeviceDriverSet_Servo::init(unsigned int positionAngle)
{
    // Initialize servos / 初始化舵机
    servoZ_.attach(PIN_SERVO_Z, 500, 2400); // Z-axis servo with custom pulse width range / 具有自定义脉冲宽度范围的 Z 轴舵机
    servoZ_.write(positionAngle);
    Utils::delayWithWatchdog(400); // Wait for servo to reach position / 等待舵机到达位置
    
    detachServos(); // Detach to prevent jitter and save power / 分离以防止抖动并节省电力
}

/**
 * @brief Control the Z-axis servo position / 控制 Z 轴舵机位置
 * @param positionAngle Target position angle (0-180 degrees) / 目标位置角度（0-180 度）
 * @param delayTime Time to wait for the servo to reach position (milliseconds) / 等待舵机到达位置的时间（毫秒）
 */
void DeviceDriverSet_Servo::control(unsigned int positionAngle,uint16_t delayTime)
{
    servoZ_.attach(PIN_SERVO_Z);
    servoZ_.write(positionAngle);
    Utils::delayWithWatchdog(delayTime); // Use blocking delay with watchdog / 使用带看门狗的阻塞延迟
    servoZ_.detach(); // Detach to prevent jitter and save power / 分离以防止抖动并节省电力
}

/**
 * @brief Set Z-axis servo angle without delay / 无延迟设置 Z 轴舵机角度
 * @param positionAngle Target angle position (0-180) / 目标角度位置 (0-180)
 */
void DeviceDriverSet_Servo::setZAngle(unsigned int positionAngle)
{
    servoZ_.attach(PIN_SERVO_Z);
    servoZ_.write(positionAngle);
}

/**
 * @brief Get the current angle of the Z-axis servo / 获取 Z 轴舵机的当前角度
 * @return Current servo angle (0-180 degrees) / 当前舵机角度（0-180 度）
 */
int DeviceDriverSet_Servo::getCurrentAngle(){
    return servoZ_.read();
}

/**
 * @brief Control multiple servos with a simplified angle system / 使用简化的角度系统控制多个舵机
 * @param servo Servo selection (1=Z-axis, 2=Y-axis, 3=both) / 舵机选择 (1=Z 轴, 2=Y 轴, 3=两者)
 * @param positionAngle Simplified angle value (multiplied by 10 internally) / 简化角度值（内部乘以 10）
 */
void DeviceDriverSet_Servo::controlMultiple(uint8_t servo, unsigned int positionAngle)
{
    targetServo_ = servo;
    targetAngle_ = positionAngle;
    currentState_ = START_Z; // Start the sequence / 开始序列
}

/**
 * @brief Update servo state machine / 更新舵机状态机
 */
void DeviceDriverSet_Servo::update()
{
    switch (currentState_) {
        case IDLE:
            break;
            
        case START_Z:
            if (targetServo_ == 1 || targetServo_ == 3) {
                unsigned int angle = constrain(targetAngle_, 1, 17);
                servoZ_.attach(PIN_SERVO_Z);
                servoZ_.write(10 * angle);
                timer_.start(500);
                currentState_ = WAITING_Z;
            } else {
                currentState_ = START_Y; // Skip Z / 跳过 Z
            }
            break;
            
        case WAITING_Z:
            if (timer_.isExpired()) {
                currentState_ = START_Y;
            }
            break;
            
        case START_Y:
            if (targetServo_ == 2 || targetServo_ == 3) {
                unsigned int angle = constrain(targetAngle_, 3, 11);
                servoY_.attach(PIN_SERVO_Y);
                servoY_.write(10 * angle);
                timer_.start(500);
                currentState_ = WAITING_Y;
            } else {
                currentState_ = FINISH; // Skip Y / 跳过 Y
            }
            break;
            
        case WAITING_Y:
            if (timer_.isExpired()) {
                currentState_ = FINISH;
            }
            break;
            
        case FINISH:
            detachServos();
            currentState_ = IDLE;
            break;
    }
}

/**
 * @brief Detach all servos to prevent jitter and save power / 分离所有舵机以防止抖动并节省电力
 */
void DeviceDriverSet_Servo::detachServos()
{
    servoZ_.detach();
    servoY_.detach();
}


/*================ IR Remote Receiver / 红外遥控接收器 =================*/

/**
 * @brief Initialize the IR receiver / 初始化红外接收器
 * @details Enables the IR receiver to start accepting IR signals / 启用红外接收器以开始接收红外信号
 */
void DeviceDriverSet_IRrecv::init() 
{
    IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);
}

/**
 * @brief Get and decode IR remote control values / 获取并解码红外遥控值
 * @param irValue Pointer to store the decoded IR button value (1-14) / 用于存储解码后的红外按键值 (1-14) 的指针
 * @return True if a valid IR code was received and decoded, false otherwise / 如果接收并解码了有效的红外代码，则返回 True，否则返回 false
 */
bool DeviceDriverSet_IRrecv::getIRValue(uint8_t *irValue)
{
    if (IrReceiver.decode())
    {
        
        uint32_t decodedValue = IrReceiver.decodedIRData.decodedRawData;
        
        // Handle repeat codes / 处理重复代码
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
            decodedValue = 0xFFFFFFFF;
        }

        bool isRecognized = false;
        static uint8_t preValue = 0;

        // Lookup table for IR codes / 红外代码查找表
        static const struct {
            uint32_t rawCode;
            uint8_t key;
        } irMap[] = {
            {A_RECV_UPPER, UP}, {A_RECV_LOWER, DOWN}, {A_RECV_LEFT, LEFT}, {A_RECV_RIGHT, RIGHT},
            {A_RECV_OK, OK},
            {A_RECV_1, NUM_1}, {A_RECV_2, NUM_2}, {A_RECV_3, NUM_3},
            {A_RECV_4, NUM_4}, {A_RECV_5, NUM_5}, {A_RECV_6, NUM_6},
            {A_RECV_7, NUM_7}, {A_RECV_8, NUM_8}, {A_RECV_9, NUM_9}
        };

        if (decodedValue == 0xFFFFFFFF) {
            // Handle repeat code: only valid for direction keys / 处理重复代码：仅对方向键有效
            if (preValue >= UP && preValue <= RIGHT) {
                *irValue = preValue;
                isRecognized = true;
            } else {
                preValue = 0; // Invalid repeat / 无效重复
            }
        } else {
            // Search for matching code in the map / 在映射中搜索匹配的代码
            for (const auto& item : irMap) {
                if (decodedValue == item.rawCode) {
                    *irValue = item.key;
                    preValue = *irValue; // Update preValue for future repeats / 更新 preValue 以供将来的重复使用
                    isRecognized = true;
                    break;
                }
            }
            
            if (!isRecognized) {
                preValue = 0; // Reset if code not found / 如果未找到代码，则重置
            }
        }
        
        IrReceiver.resume(); // Ready to receive the next value / 准备接收下一个值
        return isRecognized;
    }
    return false;
}

