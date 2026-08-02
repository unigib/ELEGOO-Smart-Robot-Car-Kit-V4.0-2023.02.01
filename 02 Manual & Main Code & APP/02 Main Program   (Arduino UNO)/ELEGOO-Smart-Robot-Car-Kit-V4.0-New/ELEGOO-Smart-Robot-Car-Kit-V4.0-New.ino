/*
 * @Author: ELEGOO
 * @Date: 2025-01-15 16:04:05
 * @LastEditTime: 2026-01-9 16:04:05
 * @LastEditors: cd
 * @Description: Smart Robot Car V4.0 - Main Program 
 * @version 2.1.2  
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet.h"
#include "ArduinoJson-v6.11.1.h"
#include <Arduino.h>

// ========== Serial Communication Constants / 串行通信常量 ==========
#define MAX_MSG_LENGTH 128   // Maximum message length / 最大消息长度
#define START_CHAR '{'       // JSON start identifier / JSON 开始标识符
#define END_CHAR '}'         // JSON end identifier / JSON 结束标识符
#define TIMEOUT_MS 1000      // Receive timeout in milliseconds / 接收超时（毫秒）

// ========== Global Objects / 全局对象 ==========
extern ApplicationFunctionSet applicationFunctionSet;


/* Arduino initialization / Arduino 初始化 */
void setup() {
    // Initialize application components / 初始化应用程序组件
    applicationFunctionSet.init();
    
    // Enable watchdog timer for system stability / 启用看门狗定时器以保持系统稳定性
    wdt_enable(WDTO_4S);
    
    // Initialize serial communication / 初始化串行通信
    Serial.begin(9600);
    
    // Print startup message / 打印启动消息
    Serial.println(F("Smart Robot Car V4.0 - System Ready"));
}

/* Main program loop / 主程序循环 */
void loop() {
    // Reset watchdog timer to prevent system reset / 重置看门狗定时器以防止系统复位
    wdt_reset();
    
    // Update sensor data and process inputs / 更新传感器数据并处理输入
    applicationFunctionSet.updateSensorData();
    applicationFunctionSet.processKeyCommand();
    applicationFunctionSet.processIRCommand();
    
    // Update timers and execute current state / 更新定时器并执行当前状态
    applicationFunctionSet.updateTimer();
    applicationFunctionSet.checkState();
}

/* Serial event handler for processing incoming commands / 用于处理传入命令的串行事件处理程序 */
void serialEvent() {
    if (Serial.available() <= 0) {
        return;
    }
    
    // Phase 1: Find the start character / 阶段 1：查找开始字符
    if (Serial.peek() != START_CHAR) {
        Serial.read(); // Discard non-start characters / 丢弃非开始字符
        return;
    }
    
    // Phase 2: Capture the complete message / 阶段 2：捕获完整消息
    String message;
    unsigned long startTime = millis();
    
    while (millis() - startTime < TIMEOUT_MS) {
        if (Serial.available()) {
            char character = Serial.read();
            
            // Terminate immediately when end character is found / 找到结束字符时立即终止
            if (character == END_CHAR) {
                message += character;
                break;
            }
            
            // Prevent buffer overflow / 防止缓冲区溢出
            if (message.length() >= MAX_MSG_LENGTH - 1) {
                return;
            }
            
            message += character;
        }
    }

    // Phase 3: Validate and process message / 阶段 3：验证并处理消息
    if (!message.endsWith("}")) {
        return;
    }

    // Phase 4: Parse JSON and execute command / 阶段 4：解析 JSON 并执行命令
    StaticJsonDocument<100> jsonDocument;
    DeserializationError error = deserializeJson(jsonDocument, message);
    
    if (error) {
        return;
    }
    
    // Process valid command / 处理有效命令
    applicationFunctionSet.analyzeSerialData(jsonDocument);
}
