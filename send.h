#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <utility>

// 外部引用（由main.ino定义）
extern WebSocketsClient webSocket;
extern String WS_TARGET_ID;

// WebSocket消息发送通用函数
template <typename TDoc>
static inline bool _wsSend(const TDoc& doc) {
  String json; 
  serializeJson(doc, json);
  Serial.println("发送消息: " + json);
  return webSocket.sendTXT(json);
}

// 发送私聊消息
static inline bool sendPrivateToId(const String& toId, const String& content) {
  if (!toId.length()) { 
    Serial.println("[warn] 目标ID为空, 未发送"); 
    return false; 
  }
  StaticJsonDocument<384> doc;
  doc["type"]    = "servo_control";
  doc["to"]      = toId;
  doc["content"] = content;
  return _wsSend(doc);
}

// 发送舵机控制指令（私聊格式）
static inline bool sendServoControl(const std::pair<int,int>* items, size_t count) {
  StaticJsonDocument<512> arrDoc;
  JsonArray arr = arrDoc.to<JsonArray>();
  
  for (size_t i = 0; i < count; i++) {
    JsonObject o = arr.createNestedObject();
    o["c"] = items[i].first;  // 通道号
    o["p"] = items[i].second; // 脉宽值
  }
  
  String content; 
  serializeJson(arrDoc, content);
  return sendPrivateToId(WS_TARGET_ID, content);
}

// 发送舵机控制指令（server-c专用格式）
static inline bool sendServoControlAsServerControl(const std::pair<int,int>* items, size_t count) {
  StaticJsonDocument<768> arrDoc;

 
  JsonArray arr = arrDoc.to<JsonArray>();
  
  for (size_t i = 0; i < count; i++) {
    JsonObject o = arr.createNestedObject();
   
    // o["c"] = items[i].first;
    // o["p"] = items[i].second;

    o["character_name"] = "jiyuan";              
    o["web_servo"]["is_bus_servo"] = true;        
    o["web_servo"]["servo_id"]     = items[i].first;   // 舵机ID
    o["web_servo"]["position"]     = items[i].second;  // 角度
    o["web_servo"]["speed"]        = 100;
  }
  
  String content; 
  serializeJson(arrDoc, content);
  
  // 按server-c要求的格式封装
  StaticJsonDocument<576> doc;
  doc["type"]    = "servo_control";
  doc["content"] = content;
  return _wsSend(doc);
}
