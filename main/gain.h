#pragma once
#include <stdint.h>
#include <math.h>
#include <utility>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "send.h"

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

// ================== 引脚定义 ==================
#define MUX_S0   GPIO_NUM_16
#define MUX_S1   GPIO_NUM_17
#define MUX_S2   GPIO_NUM_18
#define MUX_S3   GPIO_NUM_19
#define MUX_EN   GPIO_NUM_21      // 若EN接地，注释此行及相关代码
#define MUX_SIG_GPIO  GPIO_NUM_34 // ADC输入引脚
#define MUX_SIG_ADC_CHANNEL ADC_CHANNEL_6

// 读数相关参数
#define ADC_BITS         12                  // 12位ADC
#define ADC_MAX_VALUE    ((1 << ADC_BITS) - 1) // 4095
#define ADC_ATTEN        ADC_ATTEN_DB_12     // 0~3.3V量程
#define SAMPLE_COUNT     4                   // 采样次数（取平均）
#define SETTLE_US        8                   // 通道切换稳定时间(us)
#define SEND_CHANNELS    16                  // 16路通道

// ================== 硬件初始化 ==================
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static bool s_adc_inited = false;

inline void initPins() {
  uint64_t mask = (1ULL << MUX_S0) | (1ULL << MUX_S1) | (1ULL << MUX_S2) | (1ULL << MUX_S3);
#ifdef MUX_EN
  mask |= (1ULL << MUX_EN);
#endif
  gpio_config_t io_conf = {
    .pin_bit_mask = mask,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);

#ifdef MUX_EN
  gpio_set_level(MUX_EN, 0);  // 低电平使能多路选择器
#endif
}

inline void setupADC() {
  adc_oneshot_unit_init_cfg_t init_cfg = {};
  init_cfg.unit_id = ADC_UNIT_1;
  init_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
  if (adc_oneshot_new_unit(&init_cfg, &s_adc_handle) != ESP_OK) {
    s_adc_handle = NULL;
    return;
  }

  adc_oneshot_chan_cfg_t chan_cfg = {
    .atten = ADC_ATTEN,
    .bitwidth = ADC_BITWIDTH_12,
  };
  adc_oneshot_config_channel(s_adc_handle, MUX_SIG_ADC_CHANNEL, &chan_cfg);
}

inline void ensureMuxADC() {
  if (!s_adc_inited) {
    initPins();
    setupADC();
    s_adc_inited = true;
  }
}

inline float Angle(int adc) {
  float t = (float)(adc) / (float)(ADC_MAX_VALUE);
  t = t * 360.0f;
  t = t > 360.0f ? 360.0f : t;
  t = t < 0.0f ? 0.0f : t;
  return t;
}

// ================== 采样控制 ==================
// 切换多路选择器通道
inline void muxSelect(uint8_t ch) {
  gpio_set_level(MUX_S0, ch & 0x01);
  gpio_set_level(MUX_S1, (ch >> 1) & 0x01);
  gpio_set_level(MUX_S2, (ch >> 2) & 0x01);
  gpio_set_level(MUX_S3, (ch >> 3) & 0x01);
}

static inline int readRawOnce() {
  int raw = 0;
  if (s_adc_handle) {
    if (adc_oneshot_read(s_adc_handle, MUX_SIG_ADC_CHANNEL, &raw) != ESP_OK) {
      raw = 0;
    }
  }
  return raw;
}

// 读取指定通道ADC值（限制在0~4095）
inline float readChannel(uint8_t ch, uint8_t samples = SAMPLE_COUNT) {
  ensureMuxADC();
  muxSelect(ch);
  esp_rom_delay_us(SETTLE_US);
  (void)readRawOnce(); // 丢弃第一次不稳定读数

  uint32_t acc = 0;
  for (uint8_t i = 0; i < samples; i++) {
    acc += (uint32_t)readRawOnce();
  }
  int rawValue = (int)(acc / samples);
  return Angle(clampi(rawValue, 0, ADC_MAX_VALUE));
}

// ================== ADC→脉宽映射 ==================
static inline int adcToPulse(int adc, int idx) {
  int amin = ADC_MIN16[idx];
  int amax = ADC_MAX16[idx];
  if (amax <= amin) return SERVO_PULSE_MIN;

  float t = (float)(adc - amin) / (float)(amax - amin);
  t = clampi((int)(t * 1000.0f), 0, 1000) / 1000.0f;
  if (ADC_INVERT16[idx]) t = 1.0f - t;

  int pulse = (int)lroundf(SERVO_PULSE_MIN + t * (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
  return clampi(pulse, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
}

// 读取16路并发送控制指令
inline void readAll16AndSendServos() {
  std::pair<int, int> servoPairs[SEND_CHANNELS];
  for (int ch = 0; ch < SEND_CHANNELS; ch++) {
    float anglet = readChannel(ch);
    int angle = (int)anglet;
    servoPairs[ch] = {ch, angle};
  }
  sendServoControl(servoPairs, SEND_CHANNELS);
}
