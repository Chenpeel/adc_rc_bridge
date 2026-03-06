#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <utility>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_websocket_client.h"
#include "cJSON.h"

#include "gain.h"
#include "send.h"

static const char* TAG = "app";
static const char* TARGET_NAME = "esp32";
static const uint32_t SEND_INTERVAL_MS = 50; // 20Hz

static const char* DEFAULT_SSID = "gaoda";
static const char* DEFAULT_PASS = "gaoda123";
static const char* DEFAULT_NAME = "body";

static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT = BIT1;
static int s_retry_num = 0;

static bool g_send_as_servo_control = false;

static char g_ssid[33] = {0};
static char g_pass[65] = {0};
static char g_board_name[32] = "body";

// WebSocket globals used by send.h
esp_websocket_client_handle_t ws_client = NULL;
char ws_target_id[64] = {0};

static httpd_handle_t s_httpd = NULL;
static bool s_ap_netif_created = false;

static const char html_form[] =
  "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>ESP32 配网</title></head>"
  "<body><h2>请输入 WiFi 信息</h2>"
  "<form action=\"/save\" method=\"POST\">"
  "SSID: <input type=\"text\" name=\"ssid\"><br><br>"
  "密码: <input type=\"password\" name=\"pass\"><br><br>"
  "<input type=\"submit\" value=\"提交\">"
  "</form></body></html>";

static void safe_strcpy(char* dst, size_t dst_len, const char* src) {
  if (!dst || dst_len == 0) return;
  if (!src) {
    dst[0] = '\0';
    return;
  }
  size_t n = strlen(src);
  if (n >= dst_len) n = dst_len - 1;
  memcpy(dst, src, n);
  dst[n] = '\0';
}

static esp_err_t nvs_load_str(nvs_handle_t h, const char* key, char* out, size_t out_len, const char* def) {
  size_t required = 0;
  esp_err_t err = nvs_get_str(h, key, NULL, &required);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    safe_strcpy(out, out_len, def);
    return ESP_OK;
  }
  if (err != ESP_OK) {
    safe_strcpy(out, out_len, def);
    return err;
  }
  if (required == 0) {
    safe_strcpy(out, out_len, def);
    return ESP_OK;
  }
  if (required <= out_len) {
    return nvs_get_str(h, key, out, &required);
  }
  char* tmp = (char*)malloc(required);
  if (!tmp) {
    safe_strcpy(out, out_len, def);
    return ESP_ERR_NO_MEM;
  }
  err = nvs_get_str(h, key, tmp, &required);
  if (err == ESP_OK) {
    safe_strcpy(out, out_len, tmp);
  } else {
    safe_strcpy(out, out_len, def);
  }
  free(tmp);
  return err;
}

static esp_err_t nvs_save_str(const char* key, const char* val) {
  nvs_handle_t h;
  esp_err_t err = nvs_open("wifi", NVS_READWRITE, &h);
  if (err != ESP_OK) return err;
  err = nvs_set_str(h, key, val ? val : "");
  if (err == ESP_OK) {
    err = nvs_commit(h);
  }
  nvs_close(h);
  return err;
}

static void nvs_clear_wifi() {
  nvs_handle_t h;
  if (nvs_open("wifi", NVS_READWRITE, &h) == ESP_OK) {
    nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
  }
}

static void scan_networks() {
  ESP_LOGI(TAG, "Scanning WiFi...");
  wifi_scan_config_t scan_config = {};
  scan_config.ssid = NULL;
  scan_config.bssid = NULL;
  scan_config.channel = 0;
  scan_config.show_hidden = true;
  scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
  scan_config.scan_time.active.min = 100;
  scan_config.scan_time.active.max = 300;
  esp_err_t err = esp_wifi_scan_start(&scan_config, true);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "WiFi scan failed: %s", esp_err_to_name(err));
    return;
  }
  uint16_t ap_num = 0;
  esp_wifi_scan_get_ap_num(&ap_num);
  if (ap_num == 0) {
    ESP_LOGI(TAG, "未找到网络");
    return;
  }
  wifi_ap_record_t* ap_records = (wifi_ap_record_t*)calloc(ap_num, sizeof(wifi_ap_record_t));
  if (!ap_records) {
    return;
  }
  if (esp_wifi_scan_get_ap_records(&ap_num, ap_records) == ESP_OK) {
    for (int i = 0; i < ap_num; ++i) {
      ESP_LOGI(TAG, "%d: %s (%d)", i + 1, (char*)ap_records[i].ssid, ap_records[i].rssi);
    }
  }
  free(ap_records);
}

static void restart_task(void* arg) {
  vTaskDelay(pdMS_TO_TICKS(1500));
  esp_restart();
}

static esp_err_t root_get_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html_form, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t save_post_handler(httpd_req_t* req) {
  int total_len = req->content_len;
  if (total_len <= 0 || total_len > 512) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "参数错误");
  }
  char* buf = (char*)calloc(total_len + 1, 1);
  if (!buf) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "内存不足");
  }
  int received = 0;
  while (received < total_len) {
    int ret = httpd_req_recv(req, buf + received, total_len - received);
    if (ret <= 0) {
      free(buf);
      return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "读取失败");
    }
    received += ret;
  }

  char ssid[33] = {0};
  char pass[65] = {0};
  if (httpd_query_key_value(buf, "ssid", ssid, sizeof(ssid)) != ESP_OK ||
      httpd_query_key_value(buf, "pass", pass, sizeof(pass)) != ESP_OK) {
    free(buf);
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "参数错误");
  }
  free(buf);

  nvs_save_str("ssid", ssid);
  nvs_save_str("pass", pass);

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "配置已保存，正在尝试连接...");
  xTaskCreate(restart_task, "restart_task", 2048, NULL, 5, NULL);
  return ESP_OK;
}

static httpd_handle_t start_http_server() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) != ESP_OK) {
    return NULL;
  }

  httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
    .user_ctx = NULL
  };
  httpd_register_uri_handler(server, &root);

  httpd_uri_t save = {
    .uri = "/save",
    .method = HTTP_POST,
    .handler = save_post_handler,
    .user_ctx = NULL
  };
  httpd_register_uri_handler(server, &save);

  return server;
}

static void start_ap_mode() {
  ESP_LOGW(TAG, "WiFi 连接失败，进入AP模式");
  nvs_clear_wifi();

  if (!s_ap_netif_created) {
    esp_netif_create_default_wifi_ap();
    s_ap_netif_created = true;
  }

  wifi_config_t wifi_config = {};
  safe_strcpy((char*)wifi_config.ap.ssid, sizeof(wifi_config.ap.ssid), "ESP_32_AP");
  safe_strcpy((char*)wifi_config.ap.password, sizeof(wifi_config.ap.password), "00000000");
  wifi_config.ap.ssid_len = strlen((char*)wifi_config.ap.ssid);
  wifi_config.ap.channel = 1;
  wifi_config.ap.max_connection = 4;
  wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
  if (strlen((char*)wifi_config.ap.password) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  s_httpd = start_http_server();
  if (s_httpd) {
    ESP_LOGI(TAG, "AP 配网页面已启动");
  }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < 20) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGW(TAG, "重连WiFi中... (%d)", s_retry_num);
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    ESP_LOGI(TAG, "WiFi 连接成功, IP: " IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static void wifi_init_sta(const char* ssid, const char* pass) {
  s_wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

  wifi_config_t wifi_config = {};
  safe_strcpy((char*)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), ssid);
  safe_strcpy((char*)wifi_config.sta.password, sizeof(wifi_config.sta.password), pass);
  wifi_config.sta.pmf_cfg.capable = true;
  wifi_config.sta.pmf_cfg.required = false;
  wifi_config.sta.threshold.authmode = (strlen(pass) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  scan_networks();
  ESP_LOGI(TAG, "正在连接 WiFi (%s)...", ssid);
  esp_wifi_connect();

  EventBits_t bits = xEventGroupWaitBits(
      s_wifi_event_group,
      WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
      pdFALSE,
      pdFALSE,
      pdMS_TO_TICKS(20000));

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "WiFi 已连接");
  } else {
    ESP_LOGW(TAG, "WiFi 连接失败");
    start_ap_mode();
  }
}

static void handle_servo_control(cJSON* obj) {
  if (!cJSON_IsObject(obj)) return;
  cJSON* c = cJSON_GetObjectItemCaseSensitive(obj, "c");
  cJSON* p = cJSON_GetObjectItemCaseSensitive(obj, "p");
  if (cJSON_IsNumber(c) && cJSON_IsNumber(p)) {
    int channel = c->valueint;
    int pulse = p->valueint;
    if (pulse < 0) pulse = 0;
    if (pulse > 4095) pulse = 4095;
    ESP_LOGI(TAG, "设置通道 %d 脉宽 %d", channel, pulse);
  }
}

static void try_pick_target_id_from_list(cJSON* arr) {
  if (!cJSON_IsArray(arr)) return;
  cJSON* item = NULL;
  cJSON_ArrayForEach(item, arr) {
    cJSON* name = cJSON_GetObjectItemCaseSensitive(item, "name");
    cJSON* id = cJSON_GetObjectItemCaseSensitive(item, "id");
    const char* name_str = cJSON_IsString(name) ? name->valuestring : "";
    const char* id_str = cJSON_IsString(id) ? id->valuestring : "";
    if (!id_str || !*id_str) continue;
    if (strcmp(name_str, TARGET_NAME) == 0) {
      if (strcmp(ws_target_id, id_str) != 0) {
        safe_strcpy(ws_target_id, sizeof(ws_target_id), id_str);
        ESP_LOGI(TAG, "绑定目标 %s, id=%s", TARGET_NAME, ws_target_id);
      }
      return;
    }
  }
  ESP_LOGW(TAG, "在线列表里没有名为 %s 的设备", TARGET_NAME);
}

static void handle_message(const char* message) {
  cJSON* doc = cJSON_Parse(message);
  if (!doc) {
    ESP_LOGW(TAG, "JSON解析错误");
    return;
  }
  cJSON* type = cJSON_GetObjectItemCaseSensitive(doc, "type");
  if (!cJSON_IsString(type) || !type->valuestring) {
    cJSON_Delete(doc);
    return;
  }

  const char* type_str = type->valuestring;
  if (strcmp(type_str, "heartbeat") == 0) {
    _ws_send("{\"type\":\"heartbeat\"}");
    cJSON_Delete(doc);
    return;
  }

  if (strcmp(type_str, "connected") == 0 || strcmp(type_str, "presence") == 0) {
    cJSON* onlineClients = cJSON_GetObjectItemCaseSensitive(doc, "onlineClients");
    cJSON* onlineUsers = cJSON_GetObjectItemCaseSensitive(doc, "onlineUsers");
    if (onlineClients) try_pick_target_id_from_list(onlineClients);
    if (onlineUsers) try_pick_target_id_from_list(onlineUsers);

    if (strcmp(type_str, "connected") == 0) {
      g_send_as_servo_control = true;
      ESP_LOGI(TAG, "已切换：开始发送 type=servo_control");
    }
    cJSON_Delete(doc);
    return;
  }

  if (strcmp(type_str, "broadcast") == 0) {
    cJSON* content = cJSON_GetObjectItemCaseSensitive(doc, "content");
    const char* content_str = cJSON_IsString(content) ? content->valuestring : "";
    ESP_LOGI(TAG, "收到广播: %s", content_str);
    cJSON_Delete(doc);
    return;
  }

  if (strcmp(type_str, "private") == 0) {
    cJSON* fromName = cJSON_GetObjectItemCaseSensitive(doc, "fromName");
    cJSON* content = cJSON_GetObjectItemCaseSensitive(doc, "content");
    const char* from_str = cJSON_IsString(fromName) ? fromName->valuestring : "";
    const char* content_str = cJSON_IsString(content) ? content->valuestring : "";
    ESP_LOGI(TAG, "收到私聊 - 来自 %s: %s", from_str, content_str);

    cJSON* arr = cJSON_Parse(content_str);
    if (arr && cJSON_IsArray(arr)) {
      cJSON* item = NULL;
      cJSON_ArrayForEach(item, arr) {
        handle_servo_control(item);
      }
    }
    if (arr) cJSON_Delete(arr);
    cJSON_Delete(doc);
    return;
  }

  if (strcmp(type_str, "servo_control") == 0) {
    cJSON* content = cJSON_GetObjectItemCaseSensitive(doc, "content");
    const char* content_str = cJSON_IsString(content) ? content->valuestring : "";
    cJSON* arr = cJSON_Parse(content_str);
    if (arr && cJSON_IsArray(arr)) {
      cJSON* item = NULL;
      cJSON_ArrayForEach(item, arr) {
        handle_servo_control(item);
      }
    }
    if (arr) cJSON_Delete(arr);
    cJSON_Delete(doc);
    return;
  }

  ESP_LOGI(TAG, "未知消息类型");
  cJSON_Delete(doc);
}

static void websocket_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
  esp_websocket_event_data_t* data = (esp_websocket_event_data_t*)event_data;
  switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED: {
      ESP_LOGI(TAG, "WebSocket 连接成功");
      cJSON* reg = cJSON_CreateObject();
      if (reg) {
        cJSON_AddStringToObject(reg, "type", "register");
        cJSON_AddStringToObject(reg, "name", g_board_name);
        char* json = cJSON_PrintUnformatted(reg);
        if (json) {
          _ws_send(json);
          free(json);
        }
        cJSON_Delete(reg);
      }
      break;
    }
    case WEBSOCKET_EVENT_DISCONNECTED:
      ESP_LOGW(TAG, "WebSocket 断开连接");
      g_send_as_servo_control = false;
      break;
    case WEBSOCKET_EVENT_DATA:
      if (data && data->data_ptr && data->data_len > 0) {
        char* msg = (char*)calloc(data->data_len + 1, 1);
        if (msg) {
          memcpy(msg, data->data_ptr, data->data_len);
          msg[data->data_len] = '\0';
          ESP_LOGI(TAG, "收到消息: %s", msg);
          handle_message(msg);
          free(msg);
        }
      }
      break;
    default:
      break;
  }
}

static void start_websocket() {
  esp_websocket_client_config_t ws_cfg = {};
  ws_cfg.uri = "ws://192.168.0.100:9102/";
  ws_cfg.buffer_size = 2048;
  ws_cfg.disable_auto_reconnect = false;
  ws_client = esp_websocket_client_init(&ws_cfg);
  esp_websocket_register_events(ws_client, WEBSOCKET_EVENT_ANY, websocket_event_handler, NULL);
  esp_websocket_client_start(ws_client);
}

static void app_task(void* arg) {
  uint32_t last_send_ms = 0;
  uint32_t last_heartbeat_ms = 0;
  while (true) {
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    if (bits & WIFI_CONNECTED_BIT) {
      uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
      if (now_ms - last_send_ms >= SEND_INTERVAL_MS) {
        last_send_ms = now_ms;

        std::pair<int, int> servoParams[SEND_CHANNELS];
        for (int ch = 0; ch < SEND_CHANNELS; ch++) {
          servoParams[ch] = {ch + 21, (int)readChannel(ch)};
        }

        sendServoControl(servoParams, SEND_CHANNELS);
        if (g_send_as_servo_control) {
          sendServoControlAsServerControl(servoParams, SEND_CHANNELS);
        }

        ESP_LOGI(TAG, "等待绑定尚未获取目标ID, 跳过发送");
      }

      if (now_ms - last_heartbeat_ms > 15000) {
        _ws_send("{\"type\":\"heartbeat\"}");
        last_heartbeat_ms = now_ms;
        ESP_LOGI(TAG, "已发送WebSocket心跳包");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

extern "C" void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  nvs_handle_t h;
  if (nvs_open("wifi", NVS_READWRITE, &h) == ESP_OK) {
    nvs_load_str(h, "ssid", g_ssid, sizeof(g_ssid), DEFAULT_SSID);
    nvs_load_str(h, "pass", g_pass, sizeof(g_pass), DEFAULT_PASS);
    nvs_load_str(h, "name", g_board_name, sizeof(g_board_name), DEFAULT_NAME);
    nvs_close(h);
  }

  ESP_LOGI(TAG, "读取到的设备名: %s", g_board_name);

  initPins();
  setupADC();

  wifi_init_sta(g_ssid, g_pass);

  if (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) {
    start_websocket();
  }

  xTaskCreate(app_task, "app_task", 4096, NULL, 5, NULL);
}
