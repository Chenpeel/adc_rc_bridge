#pragma once
#include <stdint.h>
#include <math.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

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
// ---------- 第一片 MUX ----------
#define MUX1_S0   GPIO_NUM_16
#define MUX1_S1   GPIO_NUM_17
#define MUX1_S2   GPIO_NUM_18
#define MUX1_S3   GPIO_NUM_19
#define MUX1_SIG_ADC_CHANNEL ADC_CHANNEL_6  // GPIO34

// ---------- 第二片 MUX ----------
#define MUX2_S0   GPIO_NUM_13
#define MUX2_S1   GPIO_NUM_12
#define MUX2_S2   GPIO_NUM_14
#define MUX2_S3   GPIO_NUM_27
#define MUX2_SIG_ADC_CHANNEL ADC_CHANNEL_7  // GPIO35

// 可选使能引脚（若硬件接了共用使能，可取消注释）
// #define MUX_EN   GPIO_NUM_21

// 读数相关参数
#define ADC_BITS         12                  // 12位ADC
#define ADC_MAX_VALUE    ((1 << ADC_BITS) - 1) // 4095
#define ADC_ATTEN        ADC_ATTEN_DB_12     // 0~3.3V量程
#define SAMPLE_COUNT     6                   // 采样次数（取平均）
#define SETTLE_US        20                  // 通道切换稳定时间(us)
#define SEND_CHANNELS    32                  // 32路通道（16+16）

// ================== 硬件初始化 ==================
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static bool s_adc_inited = false;

inline void initPins() {
  uint64_t mask = (1ULL << MUX1_S0) | (1ULL << MUX1_S1) | (1ULL << MUX1_S2) | (1ULL << MUX1_S3)
                | (1ULL << MUX2_S0) | (1ULL << MUX2_S1) | (1ULL << MUX2_S2) | (1ULL << MUX2_S3);
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
  adc_oneshot_config_channel(s_adc_handle, MUX1_SIG_ADC_CHANNEL, &chan_cfg);
  adc_oneshot_config_channel(s_adc_handle, MUX2_SIG_ADC_CHANNEL, &chan_cfg);
}

inline void ensureMuxADC() {
  if (!s_adc_inited) {
    initPins();
    setupADC();
    s_adc_inited = (s_adc_handle != NULL);
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
// 切换第一片多路选择器通道
inline void mux1Select(uint8_t ch) {
  gpio_set_level(MUX1_S0, ch & 0x01);
  gpio_set_level(MUX1_S1, (ch >> 1) & 0x01);
  gpio_set_level(MUX1_S2, (ch >> 2) & 0x01);
  gpio_set_level(MUX1_S3, (ch >> 3) & 0x01);
}

// 切换第二片多路选择器通道
inline void mux2Select(uint8_t ch) {
  gpio_set_level(MUX2_S0, ch & 0x01);
  gpio_set_level(MUX2_S1, (ch >> 1) & 0x01);
  gpio_set_level(MUX2_S2, (ch >> 2) & 0x01);
  gpio_set_level(MUX2_S3, (ch >> 3) & 0x01);
}

static inline int readRawOnce(adc_channel_t adc_channel) {
  int raw = 0;
  if (s_adc_handle) {
    if (adc_oneshot_read(s_adc_handle, adc_channel, &raw) != ESP_OK) {
      raw = 0;
    }
  }
  return raw;
}

// 读取指定通道ADC值（限制在0~4095）
inline float readChannel(uint8_t ch, uint8_t samples = SAMPLE_COUNT) {
  ensureMuxADC();

  uint8_t mux_ch = ch % 16;
  adc_channel_t adc_channel = MUX1_SIG_ADC_CHANNEL;
  if (ch < 16) {
    mux1Select(mux_ch);
    adc_channel = MUX1_SIG_ADC_CHANNEL;
  } else {
    mux2Select(mux_ch);
    adc_channel = MUX2_SIG_ADC_CHANNEL;
  }

  esp_rom_delay_us(SETTLE_US);
  (void)readRawOnce(adc_channel); // 丢弃前两次不稳定读数
  (void)readRawOnce(adc_channel);

  uint32_t acc = 0;
  int vmin = ADC_MAX_VALUE;
  int vmax = 0;
  for (uint8_t i = 0; i < samples; i++) {
    int v = readRawOnce(adc_channel);
    if (v < 0) v = 0;
    if (v > ADC_MAX_VALUE) v = ADC_MAX_VALUE;
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
    acc += (uint32_t)v;
  }
  uint8_t denom = samples;
  if (samples >= 3) {
    acc -= (uint32_t)vmin; // 去掉最小值
    acc -= (uint32_t)vmax; // 去掉最大值
    denom = samples - 2;
  }
  int rawValue = (int)(acc / (denom ? denom : 1));
  return Angle(clampi(rawValue, 0, ADC_MAX_VALUE));
}

// ================== ADC→脉宽映射 ==================
static inline int adcToPulse(int adc, int idx) {
  int calib_idx = idx % 16;
  if (calib_idx < 0) calib_idx += 16;

  int amin = ADC_MIN16[calib_idx];
  int amax = ADC_MAX16[calib_idx];
  if (amax <= amin) return SERVO_PULSE_MIN;

  float t = (float)(adc - amin) / (float)(amax - amin);
  t = clampi((int)(t * 1000.0f), 0, 1000) / 1000.0f;
  if (ADC_INVERT16[calib_idx]) t = 1.0f - t;

  int pulse = (int)lroundf(SERVO_PULSE_MIN + t * (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
  return clampi(pulse, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
}
