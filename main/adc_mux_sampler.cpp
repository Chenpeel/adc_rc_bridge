// ADC + 双 MUX 采样模块实现（ESP32 / ESP-IDF）
#include "adc_mux_sampler.h"

#include "esp_rom_sys.h"

static int adc_mux_sampler_max_code_from_bitwidth(adc_bitwidth_t bitwidth) {
  // ESP-IDF 里 bitwidth 是枚举；这里把常用档位映射成最大码值。
  // 若遇到未知值，默认按 12bit 处理（与本工程默认配置一致）。
  switch (bitwidth) {
    case ADC_BITWIDTH_9:
      return (1 << 9) - 1;
    case ADC_BITWIDTH_10:
      return (1 << 10) - 1;
    case ADC_BITWIDTH_11:
      return (1 << 11) - 1;
    case ADC_BITWIDTH_12:
      return (1 << 12) - 1;
    default:
      return 4095;
  }
}

static int adc_mux_sampler_clamp_int(int value, int min_value, int max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static bool adc_mux_sampler_gpio_is_valid_output(gpio_num_t gpio) {
  // GPIO_NUM_NC 通常是 -1；此外 ESP32 的有效 GPIO 也不可能超过 63。
  // 这里做一个保守校验，避免后续位移 (1ULL << gpio) 触发未定义行为。
  return (gpio >= 0) && (gpio < 64);
}

static void adc_mux_sampler_select_mux1_channel(const adc_mux_sampler_config_t* cfg,
                                                uint8_t mux_channel) {
  gpio_set_level(cfg->mux1_s0, mux_channel & 0x01);
  gpio_set_level(cfg->mux1_s1, (mux_channel >> 1) & 0x01);
  gpio_set_level(cfg->mux1_s2, (mux_channel >> 2) & 0x01);
  gpio_set_level(cfg->mux1_s3, (mux_channel >> 3) & 0x01);
}

static void adc_mux_sampler_select_mux2_channel(const adc_mux_sampler_config_t* cfg,
                                                uint8_t mux_channel) {
  gpio_set_level(cfg->mux2_s0, mux_channel & 0x01);
  gpio_set_level(cfg->mux2_s1, (mux_channel >> 1) & 0x01);
  gpio_set_level(cfg->mux2_s2, (mux_channel >> 2) & 0x01);
  gpio_set_level(cfg->mux2_s3, (mux_channel >> 3) & 0x01);
}

extern "C" {

adc_mux_sampler_config_t adc_mux_sampler_default_config(void) {
  // 注意：该默认配置对齐本工程历史硬件接线（保证迁移后采样通道不变）。
  // - MUX1 地址线: GPIO16/17/18/19, SIG->ADC1_CH6(GPIO34)
  // - MUX2 地址线: GPIO13/12/14/27, SIG->ADC1_CH7(GPIO35)
  // - 使用 ADC1（避免 ADC2 与 WiFi 资源冲突）
  adc_mux_sampler_config_t cfg = {};
  cfg.adc_unit = ADC_UNIT_1;
  cfg.adc_atten = ADC_ATTEN_DB_12;
  cfg.adc_bitwidth = ADC_BITWIDTH_12;

  cfg.mux1_s0 = GPIO_NUM_16;
  cfg.mux1_s1 = GPIO_NUM_17;
  cfg.mux1_s2 = GPIO_NUM_18;
  cfg.mux1_s3 = GPIO_NUM_19;
  cfg.mux1_sig_adc_channel = ADC_CHANNEL_6; // GPIO34

  cfg.mux2_s0 = GPIO_NUM_13;
  cfg.mux2_s1 = GPIO_NUM_12;
  cfg.mux2_s2 = GPIO_NUM_14;
  cfg.mux2_s3 = GPIO_NUM_27;
  cfg.mux2_sig_adc_channel = ADC_CHANNEL_7; // GPIO35

  cfg.mux_enable_gpio = GPIO_NUM_NC; // 默认不使用 EN
  cfg.mux_enable_active_level = 0;

  cfg.settle_delay_us = ADC_MUX_SAMPLER_DEFAULT_SETTLE_DELAY_US;
  cfg.discard_read_count = ADC_MUX_SAMPLER_DEFAULT_DISCARD_READ_COUNT;
  return cfg;
}

adc_mux_sampler_read_opts_t adc_mux_sampler_default_read_opts(void) {
  adc_mux_sampler_read_opts_t opts = {};
  opts.sample_count = ADC_MUX_SAMPLER_DEFAULT_SAMPLE_COUNT;
  opts.trim_extremes = true;
  return opts;
}

esp_err_t adc_mux_sampler_init(adc_mux_sampler_t* sampler, const adc_mux_sampler_config_t* config) {
  if (!sampler || !config) return ESP_ERR_INVALID_ARG;
  if (sampler->initialized) return ESP_OK;

  // 参数校验：
  // - 地址线必须是有效 GPIO（不能是 GPIO_NUM_NC）
  // - enable 脚如果存在，则 active_level 只能是 0/1
  if (!adc_mux_sampler_gpio_is_valid_output(config->mux1_s0) ||
      !adc_mux_sampler_gpio_is_valid_output(config->mux1_s1) ||
      !adc_mux_sampler_gpio_is_valid_output(config->mux1_s2) ||
      !adc_mux_sampler_gpio_is_valid_output(config->mux1_s3) ||
      !adc_mux_sampler_gpio_is_valid_output(config->mux2_s0) ||
      !adc_mux_sampler_gpio_is_valid_output(config->mux2_s1) ||
      !adc_mux_sampler_gpio_is_valid_output(config->mux2_s2) ||
      !adc_mux_sampler_gpio_is_valid_output(config->mux2_s3)) {
    return ESP_ERR_INVALID_ARG;
  }
  if (config->mux_enable_gpio != GPIO_NUM_NC) {
    if (!adc_mux_sampler_gpio_is_valid_output(config->mux_enable_gpio)) {
      return ESP_ERR_INVALID_ARG;
    }
    if (config->mux_enable_active_level != 0 && config->mux_enable_active_level != 1) {
      return ESP_ERR_INVALID_ARG;
    }
  }

  // 1) 配置 MUX 地址线为输出
  uint64_t pin_mask = 0;
  pin_mask |= (1ULL << config->mux1_s0);
  pin_mask |= (1ULL << config->mux1_s1);
  pin_mask |= (1ULL << config->mux1_s2);
  pin_mask |= (1ULL << config->mux1_s3);
  pin_mask |= (1ULL << config->mux2_s0);
  pin_mask |= (1ULL << config->mux2_s1);
  pin_mask |= (1ULL << config->mux2_s2);
  pin_mask |= (1ULL << config->mux2_s3);
  if (config->mux_enable_gpio != GPIO_NUM_NC) {
    pin_mask |= (1ULL << config->mux_enable_gpio);
  }

  gpio_config_t io_conf = {
      .pin_bit_mask = pin_mask,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t err = gpio_config(&io_conf);
  if (err != ESP_OK) return err;

  if (config->mux_enable_gpio != GPIO_NUM_NC) {
    gpio_set_level(config->mux_enable_gpio, config->mux_enable_active_level ? 1 : 0);
  }

  // 2) 初始化 ADC oneshot unit
  adc_oneshot_unit_init_cfg_t unit_init_cfg = {};
  unit_init_cfg.unit_id = config->adc_unit;
  unit_init_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;

  adc_oneshot_unit_handle_t adc_handle = NULL;
  err = adc_oneshot_new_unit(&unit_init_cfg, &adc_handle);
  if (err != ESP_OK) return err;

  // 3) 配置两路 SIG 的 ADC 通道参数
  adc_oneshot_chan_cfg_t sig_channel_cfg = {
      .atten = config->adc_atten,
      .bitwidth = config->adc_bitwidth,
  };
  err = adc_oneshot_config_channel(adc_handle, config->mux1_sig_adc_channel, &sig_channel_cfg);
  if (err != ESP_OK) {
    adc_oneshot_del_unit(adc_handle);
    return err;
  }
  err = adc_oneshot_config_channel(adc_handle, config->mux2_sig_adc_channel, &sig_channel_cfg);
  if (err != ESP_OK) {
    adc_oneshot_del_unit(adc_handle);
    return err;
  }

  sampler->adc_handle = adc_handle;
  sampler->config = *config;
  sampler->initialized = true;
  return ESP_OK;
}

esp_err_t adc_mux_sampler_read_raw_code(adc_mux_sampler_t* sampler,
                                       uint8_t logical_channel,
                                       const adc_mux_sampler_read_opts_t* opts,
                                       int* out_adc_code) {
  if (!sampler || !out_adc_code) return ESP_ERR_INVALID_ARG;
  if (!sampler->initialized || !sampler->adc_handle) return ESP_ERR_INVALID_STATE;
  if (logical_channel >= 32) return ESP_ERR_INVALID_ARG;

  // opts 是“单次读取”的策略参数；config 是“硬件固定参数”（比如地址线、SIG 接哪路 ADC）。
  // 这样做的好处：
  // - 硬件接线不变时，config 只需初始化一次
  // - debug 现场可以只改 opts（例如临时把 sample_count 调大来观察是否能稳定下来）
  adc_mux_sampler_read_opts_t read_opts = opts ? *opts : adc_mux_sampler_default_read_opts();
  if (read_opts.sample_count == 0) return ESP_ERR_INVALID_ARG;

  const int max_code = adc_mux_sampler_max_code_from_bitwidth(sampler->config.adc_bitwidth);

  // logical_channel: 0~31
  // - 0~15  : 走 MUX1
  // - 16~31 : 走 MUX2
  const uint8_t mux_channel = (uint8_t)(logical_channel % 16);
  adc_channel_t sig_adc_channel = sampler->config.mux1_sig_adc_channel;

  if (logical_channel < 16) {
    // 选择第一片 MUX 的通道，并确保后续读的是 mux1_sig_adc_channel。
    adc_mux_sampler_select_mux1_channel(&sampler->config, mux_channel);
    sig_adc_channel = sampler->config.mux1_sig_adc_channel;
  } else {
    // 选择第二片 MUX 的通道，并确保后续读的是 mux2_sig_adc_channel。
    adc_mux_sampler_select_mux2_channel(&sampler->config, mux_channel);
    sig_adc_channel = sampler->config.mux2_sig_adc_channel;
  }

  // 切换通道后等待稳定（模拟链路 + ADC 采样保持电容都会引入瞬态）
  // 经验：
  // - 如果你发现“读出来像上一通道的值”，通常不是软件 bug，而是这里的 settle/discard 不够。
  if (sampler->config.settle_delay_us > 0) {
    esp_rom_delay_us(sampler->config.settle_delay_us);
  }

  // 丢弃前 N 次读数（通常能显著降低串扰/残影）。
  // 这里不把“丢弃读数的失败”当错误：因为本质只是预热/清电荷，失败也不会影响后续正式采样的错误返回。
  for (uint8_t i = 0; i < sampler->config.discard_read_count; i++) {
    int discarded = 0;
    (void)adc_oneshot_read(sampler->adc_handle, sig_adc_channel, &discarded);
  }

  // 多次采样
  uint32_t sample_sum = 0;
  int min_sample_code = max_code;
  int max_sample_code = 0;
  for (uint8_t i = 0; i < read_opts.sample_count; i++) {
    int sample_code = 0;
    esp_err_t err = adc_oneshot_read(sampler->adc_handle, sig_adc_channel, &sample_code);
    if (err != ESP_OK) return err;

    sample_code = adc_mux_sampler_clamp_int(sample_code, 0, max_code);
    if (sample_code < min_sample_code) min_sample_code = sample_code;
    if (sample_code > max_sample_code) max_sample_code = sample_code;
    sample_sum += (uint32_t)sample_code;
  }

  uint8_t effective_count = read_opts.sample_count;
  if (read_opts.trim_extremes && read_opts.sample_count >= 3) {
    // 去极值平均：
    // - 适合抑制偶发尖峰（比如某一次采样被瞬态污染）
    // - 代价是响应更慢、并且 sample_count < 3 时无意义
    sample_sum -= (uint32_t)min_sample_code;
    sample_sum -= (uint32_t)max_sample_code;
    effective_count = (uint8_t)(read_opts.sample_count - 2);
  }

  const int avg_code = (int)(sample_sum / (effective_count ? effective_count : 1));
  *out_adc_code = adc_mux_sampler_clamp_int(avg_code, 0, max_code);
  return ESP_OK;
}

} // extern "C"
