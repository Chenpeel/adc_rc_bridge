#pragma once
#include <string.h>
#include <utility>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_websocket_client.h"
#include "cJSON.h"

// 外部引用（由app_main定义）
extern esp_websocket_client_handle_t ws_client;
extern char ws_target_id[];

static const char* TAG_SEND = "send";
static const TickType_t WS_SEND_TIMEOUT_TICKS = pdMS_TO_TICKS(3000);

static inline bool _ws_is_connected() {
  return ws_client && esp_websocket_client_is_connected(ws_client);
}

// WebSocket消息发送通用函数
static inline bool _ws_send(const char* json) {
  if (!json || !_ws_is_connected()) {
    return false;
  }
  ESP_LOGI(TAG_SEND, "发送消息: %s", json);
  int ret = esp_websocket_client_send_text(ws_client, json, (int)strlen(json), WS_SEND_TIMEOUT_TICKS);
  return ret >= 0;
}

// 发送私聊消息
static inline bool sendPrivateToId(const char* toId, const char* content) {
  if (!toId || !*toId) {
    ESP_LOGW(TAG_SEND, "目标ID为空, 未发送");
    return false;
  }
  cJSON* doc = cJSON_CreateObject();
  if (!doc) return false;
  cJSON_AddStringToObject(doc, "type", "servo_control");
  cJSON_AddStringToObject(doc, "to", toId);
  cJSON_AddStringToObject(doc, "content", content ? content : "");

  char* json = cJSON_PrintUnformatted(doc);
  bool ok = json ? _ws_send(json) : false;
  if (json) {
    free(json);
  }
  cJSON_Delete(doc);
  return ok;
}

// 发送舵机控制指令（私聊格式）
static inline bool sendServoControl(const std::pair<int,int>* items, size_t count) {
  cJSON* arr = cJSON_CreateArray();
  if (!arr) return false;

  for (size_t i = 0; i < count; i++) {
    cJSON* o = cJSON_CreateObject();
    if (!o) continue;
    cJSON_AddNumberToObject(o, "c", items[i].first);
    cJSON_AddNumberToObject(o, "p", items[i].second);
    cJSON_AddItemToArray(arr, o);
  }

  char* content = cJSON_PrintUnformatted(arr);
  bool ok = content ? sendPrivateToId(ws_target_id, content) : false;
  if (content) {
    free(content);
  }
  cJSON_Delete(arr);
  return ok;
}

// 发送舵机控制指令（server-c专用格式）
static inline bool sendServoControlAsServerControl(const std::pair<int,int>* items, size_t count) {
  cJSON* arr = cJSON_CreateArray();
  if (!arr) return false;

  for (size_t i = 0; i < count; i++) {
    cJSON* o = cJSON_CreateObject();
    if (!o) continue;
    cJSON_AddStringToObject(o, "character_name", "jiyuan");

    cJSON* web_servo = cJSON_CreateObject();
    if (web_servo) {
      cJSON_AddBoolToObject(web_servo, "is_bus_servo", true);
      cJSON_AddNumberToObject(web_servo, "servo_id", items[i].first);
      cJSON_AddNumberToObject(web_servo, "position", items[i].second);
      cJSON_AddNumberToObject(web_servo, "speed", 100);
      cJSON_AddItemToObject(o, "web_servo", web_servo);
    }
    cJSON_AddItemToArray(arr, o);
  }

  char* content = cJSON_PrintUnformatted(arr);
  if (!content) {
    cJSON_Delete(arr);
    return false;
  }

  cJSON* doc = cJSON_CreateObject();
  if (!doc) {
    free(content);
    cJSON_Delete(arr);
    return false;
  }
  cJSON_AddStringToObject(doc, "type", "servo_control");
  cJSON_AddStringToObject(doc, "content", content);

  char* json = cJSON_PrintUnformatted(doc);
  bool ok = json ? _ws_send(json) : false;

  if (json) {
    free(json);
  }
  free(content);
  cJSON_Delete(doc);
  cJSON_Delete(arr);
  return ok;
}
