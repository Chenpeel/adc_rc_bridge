#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "send.h"
#include "gain.h"

static bool g_send_as_servo_control = false;

// 目标设备名
const char *TARGET_NAME = "esp32";
String WS_TARGET_ID; // 绑定目标设备ID

const uint32_t SEND_INTERVAL_MS = 50; // 20Hz发送频率
static uint32_t lastSend = 0;

AsyncWebServer server(80);
Preferences preferences;

String input_ssid, input_password;
String board_name = "body";
bool wifiConfigured = false;

WebSocketsClient webSocket;

// 舵机单条控制处理
void handleServoControl(JsonObject &obj)
{
  if (obj.containsKey("c") && obj.containsKey("p"))
  {
    int channel = obj["c"];
    int pulse = obj["p"];
    pulse = constrain(pulse, 0, 4095);
    Serial.printf("设置通道 %d 脉宽 %d\n", channel, pulse);
    // TODO: 后续可添加PCA9685输出代码
  }
}

// 从在线列表中匹配目标设备ID
void tryPickTargetIdFromList(JsonVariant v)
{
  if (!v || !v.is<JsonArray>())
    return;
  for (JsonObject u : v.as<JsonArray>())
  {
    const char *name = u["name"] | "";
    const char *id = u["id"] | "";
    if (!id || !*id)
      continue;
    if (strcmp(name, TARGET_NAME) == 0)
    {
      if (WS_TARGET_ID != id)
      {
        WS_TARGET_ID = id;
        Serial.printf("[info] 绑定目标 %s, id=%s\n", TARGET_NAME, WS_TARGET_ID.c_str());
      }
      return;
    }
  }
  Serial.printf("[warn] 在线列表里没有名为 %s 的设备\n", TARGET_NAME);
}

// WebSocket消息处理
void handleMessage(const char *message)
{
  StaticJsonDocument<2048> doc;
  DeserializationError error = deserializeJson(doc, message);
  if (error)
  {
    Serial.println("JSON解析错误");
    return;
  }

  const char *type = doc["type"] | "";
  if (!*type)
    return;

  if (strcmp(type, "heartbeat") == 0)
  {
    StaticJsonDocument<64> pong;
    pong["type"] = "heartbeat";
    String js;
    serializeJson(pong, js);
    webSocket.sendTXT(js);
    return;
  }

  if (strcmp(type, "connected") == 0 || strcmp(type, "presence") == 0)
  {
    if (doc.containsKey("onlineClients"))
      tryPickTargetIdFromList(doc["onlineClients"]);
    if (doc.containsKey("onlineUsers"))
      tryPickTargetIdFromList(doc["onlineUsers"]);

    // 收到连接成功消息后切换发送模式
    if (strcmp(type, "connected") == 0)
    {
      g_send_as_servo_control = true;
      Serial.println("[mode] 已切换：开始发送 type=servo_control");
    }
    return;
  }

  if (strcmp(type, "broadcast") == 0)
  {
    const char *content = doc["content"] | "";
    Serial.println(String("收到广播: ") + content);
    return;
  }

  if (strcmp(type, "private") == 0)
  {
    const char *fromName = doc["fromName"] | "";
    const char *content = doc["content"] | "";
    Serial.println(String("收到私聊 - 来自 ") + fromName + ": " + content);

    // 解析舵机控制指令
    StaticJsonDocument<256> arr;
    DeserializationError e = deserializeJson(arr, content);
    if (!e && arr.is<JsonArray>())
    {
      for (JsonObject servo : arr.as<JsonArray>())
        handleServoControl(servo);
    }
    return;
  }

  if (strcmp(type, "servo_control") == 0)
  {
    const char *contentStr = doc["content"] | "";
    StaticJsonDocument<256> arr;
    if (!deserializeJson(arr, contentStr) && arr.is<JsonArray>())
    {
      for (JsonObject servo : arr.as<JsonArray>())
        handleServoControl(servo);
    }
    return;
  }

  Serial.println("未知消息类型");
}

// WebSocket事件回调
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.println("WebSocket 断开连接");
    g_send_as_servo_control = false;
    break;
  case WStype_CONNECTED:
  {
    Serial.println("WebSocket 连接成功");
    StaticJsonDocument<128> reg;
    reg["type"] = "register";
    reg["name"] = board_name.c_str();
    String js;
    serializeJson(reg, js);
    Serial.println("发送注册: " + js);
    webSocket.sendTXT(js);
    break;
  }
  case WStype_TEXT:
    Serial.print("收到消息: ");
    Serial.println((char *)payload);
    handleMessage((char *)payload);
    break;
  case WStype_BIN:
    Serial.println("收到二进制数据");
    break;
  default:
    break;
  }
}

// 配网页面HTML
const char html_form[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><title>ESP32 配网</title></head>
<body><h2>请输入 WiFi 信息</h2>
<form action="/save" method="POST">
SSID: <input type="text" name="ssid"><br><br>
密码: <input type="password" name="pass"><br><br>
<input type="submit" value="提交">
</form></body></html>
)HTML";

// 处理WiFi配置保存
void handleSave(AsyncWebServerRequest *request)
{
  if (request->hasParam("ssid", true))
  {
    input_ssid = request->getParam("ssid", true)->value();
    input_password = request->getParam("pass", true)->value();
    preferences.begin("wifi", false);
    preferences.putString("ssid", input_ssid);
    preferences.putString("pass", input_password);
    preferences.end();
    request->send(200, "text/plain", "配置已保存，正在尝试连接...");
    delay(1500);
    ESP.restart();
  }
  else
  {
    request->send(400, "text/plain", "参数错误");
  }
}

// 扫描周围WiFi
void scanNetworks()
{
  Serial.println("Scanning WiFi...");
  int n = WiFi.scanNetworks();
  if (n <= 0)
  {
    Serial.println("未找到网络");
    return;
  }
  for (int i = 0; i < n; ++i)
  {
    Serial.printf("%d: %s (%d)\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
  }
}

// 启动AP模式用于配网
void startAPMode()
{
  preferences.begin("wifi", false);
  preferences.clear();
  preferences.end();

  WiFi.disconnect();
  WiFi.softAP("ESP_32_AP", "00000000", 1);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html", html_form); });
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
}

// 初始化设置
void setup()
{
  Serial.begin(115200);

  // 初始化硬件
  initPins();
  setupADC();

  // 读取WiFi配置
  preferences.begin("wifi", false);
  String saved_ssid = preferences.getString("ssid", "gaoda");
  String saved_pass = preferences.getString("pass", "gaoda123");
  board_name = preferences.getString("name", "body");
  preferences.end();

  Serial.println("读取到的设备名: " + board_name);
  scanNetworks();

  // 连接WiFi
  WiFi.begin(saved_ssid.c_str(), saved_pass.c_str());
  Serial.printf("正在连接 WiFi (%s)...\n", saved_ssid.c_str());
  for (int tries = 0; WiFi.status() != WL_CONNECTED && tries < 20; ++tries)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("WiFi 连接成功, IP: ");
    Serial.println(WiFi.localIP());
    // 初始化WebSocket
    webSocket.begin("192.168.0.100", 9102, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
  }
  else
  {
    Serial.printf("WiFi 连接失败 (%d)\n", WiFi.status());
    startAPMode();
  }
}

// 主循环
void loop()
{
  //  Serial.printf("D5=%4f ",readChannel(5));
  //   Serial.printf("D6=%4f ",readChannel(6));
  // Serial.printf("D7=%4f ",readChannel(7));
  if (WiFi.status() != WL_CONNECTED)
    return;
  webSocket.loop();

  uint32_t now = millis();
  if (now - lastSend >= SEND_INTERVAL_MS)
  {
    lastSend = now;

    // 1. 读取16路ADC值并打印
    int adcVals[SEND_CHANNELS] = {0};
    Serial.print("\n[ADC读取] 通道0-7: ");
    for (int ch = 0; ch < SEND_CHANNELS; ch++)
    {
      //   adcVals[ch] = readChannel(ch);
      Serial.printf("%4f", readChannel(ch));
      if (ch == 7)
        Serial.print("\n          通道8-15:");
    }
    Serial.println();

    // 2. 转换为舵机脉宽并打印
    float pulseVals[SEND_CHANNELS] = {0};
    Serial.print("[脉宽计算] 通道0-7: ");
    for (int ch = 0; ch < SEND_CHANNELS; ch++)
    {
      pulseVals[ch] = readChannel(ch);
      // Serial.printf("%4d ", pulseVals[ch]);
      if (ch == 7)
        Serial.print("\n          通道8-15:");
    }
    Serial.println();

    // 3. 绑定目标ID后发送控制指令
    //  if (WS_TARGET_ID.length()) {
    std::pair<int, int> servoParams[SEND_CHANNELS];
    for (int ch = 0; ch < SEND_CHANNELS; ch++)
    {
      servoParams[ch] = {ch + 21, readChannel(ch)};
    }

    // 发送控制指令
    sendServoControl(servoParams, SEND_CHANNELS);
    if (g_send_as_servo_control)
    {
      sendServoControlAsServerControl(servoParams, SEND_CHANNELS);
    }

    // Serial.printf("[发送成功] 16路指令已发送至ID: %s\n", WS_TARGET_ID.c_str());
    //  } else {
    Serial.println("[等待绑定] 尚未获取目标ID, 跳过发送");
    //   }
  }

  // WebSocket心跳包
  static uint32_t lastHeartbeat = 0;
  if (now - lastHeartbeat > 15000)
  {
    webSocket.sendTXT("{\"type\":\"heartbeat\"}");
    lastHeartbeat = now;
    Serial.println("[心跳] 已发送WebSocket心跳包");
  }
}
