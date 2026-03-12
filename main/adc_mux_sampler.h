#pragma once
// ================================================================
// ADC + 双 MUX 采样模块（ESP32 / ESP-IDF）
//
// 解决的问题：
// - 硬件上有两片 16:1 模拟多路选择器（MUX），通过 8 根地址线 + 2 路 ADC 输入读取最多 32 路模拟量
// - 每次切换 MUX 通道后，ADC 读数会受残留电荷/瞬态影响，需要做稳定处理
//
// 模块提供的能力：
// - 标准化 init/read API：函数返回 esp_err_t，数据通过 out 参数返回
// - read 内置：通道切换、稳定延时、丢弃前 N 次读数、多次采样、去极值平均
//
// 注意：
// - 本模块会操作 MUX 地址线 GPIO，同一时刻只能有一个调用者在读；不适合并发多任务读取。
// ================================================================

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 默认采样参数（对齐历史实现，便于平滑迁移）。
static const uint32_t ADC_MUX_SAMPLER_DEFAULT_SETTLE_DELAY_US = 20;
static const uint8_t ADC_MUX_SAMPLER_DEFAULT_DISCARD_READ_COUNT = 2;
static const uint8_t ADC_MUX_SAMPLER_DEFAULT_SAMPLE_COUNT = 6;

typedef struct adc_mux_sampler_config_t {
  // ADC oneshot unit 配置
  adc_unit_t adc_unit;
  adc_atten_t adc_atten;
  adc_bitwidth_t adc_bitwidth;

  // 第一片 MUX：地址线 S0~S3 + SIG 接到的 ADC 通道
  gpio_num_t mux1_s0;
  gpio_num_t mux1_s1;
  gpio_num_t mux1_s2;
  gpio_num_t mux1_s3;
  adc_channel_t mux1_sig_adc_channel;

  // 第二片 MUX：地址线 S0~S3 + SIG 接到的 ADC 通道
  gpio_num_t mux2_s0;
  gpio_num_t mux2_s1;
  gpio_num_t mux2_s2;
  gpio_num_t mux2_s3;
  adc_channel_t mux2_sig_adc_channel;

  // 可选：MUX 使能脚（没有就填 GPIO_NUM_NC）
  gpio_num_t mux_enable_gpio;
  int mux_enable_active_level; // 0 或 1

  // 通道切换后稳定延时（us）
  uint32_t settle_delay_us;

  // 丢弃前 N 次读数（通常 1~3 次）
  uint8_t discard_read_count;
} adc_mux_sampler_config_t;

typedef struct adc_mux_sampler_read_opts_t {
  // 有效采样次数（越大越稳，但越慢）
  uint8_t sample_count;
  // sample_count >= 3 时，是否启用去极值平均（去掉 min/max）
  bool trim_extremes;
} adc_mux_sampler_read_opts_t;

typedef struct adc_mux_sampler_t {
  // ADC oneshot 句柄（由 init 创建，deinit 删除）
  adc_oneshot_unit_handle_t adc_handle;

  // 固化配置（init 时拷贝）
  adc_mux_sampler_config_t config;

  // 初始化标记
  bool initialized;
} adc_mux_sampler_t;

// 返回一份“符合本项目硬件接线”的默认配置。
adc_mux_sampler_config_t adc_mux_sampler_default_config(void);

// 返回一份默认 read 参数（对齐历史实现：sample_count=6 + trim_extremes=true）。
adc_mux_sampler_read_opts_t adc_mux_sampler_default_read_opts(void);

// 初始化（GPIO + ADC oneshot）。
esp_err_t adc_mux_sampler_init(adc_mux_sampler_t* sampler, const adc_mux_sampler_config_t* config);

// 读取指定逻辑通道的 ADC 原始码值（0~max_code）。
esp_err_t adc_mux_sampler_read_raw_code(adc_mux_sampler_t* sampler,
                                       uint8_t logical_channel,
                                       const adc_mux_sampler_read_opts_t* opts,
                                       int* out_adc_code);

#ifdef __cplusplus
} // extern "C"
#endif
