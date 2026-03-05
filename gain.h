#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <utility>
#include <math.h>

// ====== 可调映射参数（16路）======
#define SERVO_PULSE_MIN  110     // 舵机最小脉宽
#define SERVO_PULSE_MAX  530     // 舵机最大脉宽
static int  ADC_MIN16[16]    = {550,550,550,550,550,550,550,550,550,550,550,550,550,550,550,550};  // ADC最小值
static int  ADC_MAX16[16]    = {3550,3500,3550,4095,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550};  // ADC最大值
static bool ADC_INVERT16[16] = {false};  // 是否反向映射

// 工具函数：限制数值范围
static inline int clampi(int v, int lo, int hi) { 
  return v < lo ? lo : (v > hi ? hi : v); 
}

// 声明发送函数（由send.h实现）
bool sendServoControl(const std::pair<int, int>* items, size_t count);

// ================== 引脚定义 ==================
#define MUX_S0   16
#define MUX_S1   17
#define MUX_S2   18
#define MUX_S3   19
#define MUX_EN   21      // 若EN接地，注释此行及相关代码
#define MUX_SIG  34      // ADC输入引脚（ADC1通道）

// 读数相关参数
#define ADC_BITS         12                  // 12位ADC
#define ADC_MAX_VALUE    ((1 << ADC_BITS) - 1) // 4095
#define ADC_ATTEN        ADC_11db             // 0~3.3V量程
#define SAMPLE_COUNT     4                   // 采样次数（取平均）
#define SETTLE_US        8                   // 通道切换稳定时间(us)
#define SEND_CHANNELS    16                  // 16路通道

// 打印函数（仅用串口）
inline void toPrint(const String &msg) { 
  Serial.println(msg); 
}
inline void toPrint(int v) { 
  String s = String(v); 
  Serial.println(s); 
}

// ================== 硬件初始化 ==================
inline void initPins() {
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_SIG, INPUT);
#ifdef MUX_EN
  pinMode(MUX_EN, OUTPUT);
  digitalWrite(MUX_EN, LOW);  // 低电平使能多路选择器
#endif
}

inline void setupADC() { 
  analogSetPinAttenuation(MUX_SIG, ADC_ATTEN);  // 设置ADC衰减
  analogReadResolution(ADC_BITS);               // 设置ADC分辨率
}

static bool _mux_adc_inited = false;
inline void ensureMuxADC() { 
  if (!_mux_adc_inited) { 
    initPins(); 
    setupADC(); 
    _mux_adc_inited = true; 
  } }


 float Angle(int adc){

  float t = (float)(adc ) / (float)(4095);
  t=t*360.0;
  t= t>360 ? 360 : t;
  t= t<0 ? 0: t;
  return t;



}
// ================== 采样控制 ==================
// 切换多路选择器通道
inline void muxSelect(uint8_t ch) {
  digitalWrite(MUX_S0, ch & 0x01);
  digitalWrite(MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(MUX_S2, (ch >> 2) & 0x01);
  digitalWrite(MUX_S3, (ch >> 3) & 0x01);
}

// 读取指定通道ADC值（限制在550~3550）
inline float readChannel(uint8_t ch, uint8_t samples = SAMPLE_COUNT) {
  ensureMuxADC();
  muxSelect(ch);
  delayMicroseconds(SETTLE_US);
  (void)analogRead(MUX_SIG); // 丢弃第一次不稳定读数
  uint32_t acc = 0; 
  for (uint8_t i = 0; i < samples; i++) {
    acc += analogRead(MUX_SIG);
  }
  int rawValue = (int)(acc / samples);

   //clampi(rawValue,550,3550); // 限制在目标范围

   return Angle(clampi(rawValue,0,4095));
}





// ================== ADC→脉宽映射 ==================
static inline int adcToPulse(int adc, int idx) {
  int amin = ADC_MIN16[idx];
  int amax = ADC_MAX16[idx];
  if (amax <= amin) return SERVO_PULSE_MIN;
  
  // 计算归一化比例
  float t = (float)(adc - amin) / (float)(amax - amin);
  t = clampi(t * 1000, 0, 1000) / 1000.0f; // 限制在0~1
  
  // 处理反向映射
  if (ADC_INVERT16[idx]) t = 1.f - t;
  
  // 计算脉宽并限制范围
  int pulse = (int)lroundf(SERVO_PULSE_MIN + t * (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
  return clampi(pulse, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
}

// 读取16路并发送控制指令
inline void readAll16AndSendServos() {
  std::pair<int, int> servoPairs[SEND_CHANNELS];
  for (int ch = 0; ch < SEND_CHANNELS; ch++) {
    float anglet  = readChannel(ch);
 //   int angle = Angle(adcVal);
 int angle=(float)anglet;
    servoPairs[ch] = {ch, angle};
  }
  sendServoControl(servoPairs, SEND_CHANNELS);
}
