#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "gain.h"

static const char* TAG = "esp_bridge";

// ====== ADC/舵机通道定义 ======
static const uint8_t SERVO_ID_MIN = 21;
static const uint8_t SERVO_ID_MAX = 43;
static const size_t SERVO_ID_COUNT = SERVO_ID_MAX - SERVO_ID_MIN + 1; // 23路
static_assert(SERVO_ID_COUNT == 23, "SERVO_ID_COUNT should be 23");

// ====== I2C从机配置 ======
static const i2c_port_t I2C_SLAVE_PORT = I2C_NUM_0;
static const gpio_num_t I2C_SLAVE_SDA_IO = GPIO_NUM_21;
static const gpio_num_t I2C_SLAVE_SCL_IO = GPIO_NUM_22;
static const uint8_t I2C_SLAVE_ADDR = 0x28;
static const uint8_t I2C_CMD_GET_FRAME = 0xA5;
static const size_t I2C_SLAVE_RX_BUF_LEN = 128;
static const size_t I2C_SLAVE_TX_BUF_LEN = 256;

// ====== 采样与任务配置 ======
static const uint32_t ADC_CAPTURE_INTERVAL_MS = 15; // ~66Hz
static const uint32_t I2C_CMD_WAIT_MS = 100;
static const uint32_t ADC_DEBUG_LOG_INTERVAL_MS = 1000;
static const uint8_t SERVO_GROUP_A_MAX_ID = 34; // 21~34
static const float SERVO_GROUP_A_MAX_DEG = 240.0f;
static const float SERVO_GROUP_B_MAX_DEG = 270.0f;
static const float SEND_MIN_DEG = -90.0f;
static const float SEND_MAX_DEG = 90.0f;
static const int ADC_CODE_MAX = 4095;
static const int ADC_BP0 = 0;
static const int ADC_BP1 = 1023;
static const int ADC_BP2 = 2047;
static const int ADC_BP3 = 3071;
static const int ADC_BP4 = 4095;
// 每路归一化区间（原始ADC码值，0~4095）
static const int ADC_NORM_MIN[SERVO_ID_COUNT] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const int ADC_NORM_MAX[SERVO_ID_COUNT] = {
    4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
    4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
// 每路相位偏移（单位：ADC码值，正值=向更大码值方向平移，负值相反）
static const int ADC_PHASE_OFFSET[SERVO_ID_COUNT] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const uint8_t DEBUG_SERVO_IDS[] = {21, 22, 23, 32, 33, 34, 35, 41, 42, 43};
static const size_t DEBUG_SERVO_ID_COUNT = sizeof(DEBUG_SERVO_IDS) / sizeof(DEBUG_SERVO_IDS[0]);

// ====== 数据帧协议 ======
// 帧格式(共60字节, 小端):
// [0]   magic0      = 0xAA
// [1]   magic1      = 0x55
// [2]   version     = 2
// [3]   channel_cnt = 23
// [4:6] seq
// [6:10]uptime_ms
// [10]  servo_id_min=21
// [11]  reserved    =0
// [12:58] send_deg_x10[23] (int16, 单位0.1度, 范围-90.0~90.0)
// [58:60] crc16_ccitt_false (对前58字节计算)
static const uint8_t FRAME_MAGIC0 = 0xAA;
static const uint8_t FRAME_MAGIC1 = 0x55;
static const uint8_t FRAME_VERSION = 2;
static const size_t FRAME_SIZE = 60;
static_assert(FRAME_SIZE == (12 + SERVO_ID_COUNT * 2 + 2), "unexpected frame size");

struct ServoSnapshot {
  uint16_t seq = 0;
  uint32_t uptime_ms = 0;
  int16_t send_deg_x10[SERVO_ID_COUNT] = {0};
};

static ServoSnapshot s_latest_snapshot;
static SemaphoreHandle_t s_snapshot_mutex = nullptr;

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float servo_max_deg(uint8_t servo_id) {
  return (servo_id <= SERVO_GROUP_A_MAX_ID) ? SERVO_GROUP_A_MAX_DEG : SERVO_GROUP_B_MAX_DEG;
}

static inline int adc_deg_to_code(float adc_deg) {
  float deg = clampf(adc_deg, 0.0f, 360.0f);
  int code = (int)roundf(deg * (float)ADC_CODE_MAX / 360.0f);
  return clampi(code, 0, ADC_CODE_MAX);
}

static inline int wrap_code(int code) {
  int range = ADC_CODE_MAX + 1;
  int wrapped = code % range;
  if (wrapped < 0) wrapped += range;
  return wrapped;
}

static inline int normalize_adc_code(size_t idx, int raw_code) {
  int lo = clampi(ADC_NORM_MIN[idx], 0, ADC_CODE_MAX - 1);
  int hi = clampi(ADC_NORM_MAX[idx], lo + 1, ADC_CODE_MAX);
  int c = clampi(raw_code, lo, hi);
  int norm = (int)roundf((float)(c - lo) * (float)ADC_CODE_MAX / (float)(hi - lo));
  return clampi(norm, 0, ADC_CODE_MAX);
}

static inline bool map_adc_code_to_send_deg(int adc_code, float* out_send_deg) {
  if (!out_send_deg) return false;
  int code = clampi(adc_code, 0, ADC_CODE_MAX);

  // 仅保留以下有效区间并线性映射:
  // 3071 -> 4095/0 -> 1023  对应  -90 -> 0 -> 90
  // 其余(1024~3070)区间返回false，表示屏蔽更新
  if (code >= ADC_BP3) {
    float t = (float)(code - ADC_BP3) / (float)(ADC_BP4 - ADC_BP3); // 0 -> 1
    *out_send_deg = SEND_MIN_DEG + t * (0.0f - SEND_MIN_DEG);       // -90 -> 0
    return true;
  }
  if (code <= ADC_BP1) {
    float t = (float)(code - ADC_BP0) / (float)(ADC_BP1 - ADC_BP0); // 0 -> 1
    *out_send_deg = 0.0f + t * SEND_MAX_DEG;                         // 0 -> 90
    return true;
  }
  return false;
}

static inline float send_deg_to_servo_deg(uint8_t servo_id, float send_deg) {
  float max_deg = servo_max_deg(servo_id);
  float reset_deg = max_deg * 0.5f;
  float s = clampf(send_deg, SEND_MIN_DEG, SEND_MAX_DEG);
  return clampf(reset_deg + s * (reset_deg / 90.0f), 0.0f, max_deg);
}

static inline void write_u16_le(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void write_i16_le(uint8_t* p, int16_t v) {
  write_u16_le(p, (uint16_t)v);
}

static inline void write_u32_le(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

static bool copy_latest_snapshot(ServoSnapshot* out) {
  if (!out || !s_snapshot_mutex) return false;
  if (xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    return false;
  }
  *out = s_latest_snapshot;
  xSemaphoreGive(s_snapshot_mutex);
  return true;
}

static size_t build_adc_frame(uint8_t* frame, size_t cap) {
  if (!frame || cap < FRAME_SIZE) return 0;

  ServoSnapshot snap;
  if (!copy_latest_snapshot(&snap)) {
    return 0;
  }

  size_t pos = 0;
  frame[pos++] = FRAME_MAGIC0;
  frame[pos++] = FRAME_MAGIC1;
  frame[pos++] = FRAME_VERSION;
  frame[pos++] = (uint8_t)SERVO_ID_COUNT;
  write_u16_le(frame + pos, snap.seq);
  pos += 2;
  write_u32_le(frame + pos, snap.uptime_ms);
  pos += 4;
  frame[pos++] = SERVO_ID_MIN;
  frame[pos++] = 0; // reserved

  for (size_t i = 0; i < SERVO_ID_COUNT; i++) {
    write_i16_le(frame + pos, snap.send_deg_x10[i]);
    pos += 2;
  }

  uint16_t crc = crc16_ccitt_false(frame, pos);
  write_u16_le(frame + pos, crc);
  pos += 2;
  return pos;
}

static esp_err_t init_i2c_slave() {
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_SLAVE;
  conf.sda_io_num = I2C_SLAVE_SDA_IO;
  conf.scl_io_num = I2C_SLAVE_SCL_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.slave.addr_10bit_en = 0;
  conf.slave.slave_addr = I2C_SLAVE_ADDR;

  esp_err_t err = i2c_param_config(I2C_SLAVE_PORT, &conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
    return err;
  }

  err = i2c_driver_install(I2C_SLAVE_PORT,
                           conf.mode,
                           (int)I2C_SLAVE_RX_BUF_LEN,
                           (int)I2C_SLAVE_TX_BUF_LEN,
                           0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(TAG,
           "I2C slave ready: addr=0x%02X, SDA=%d, SCL=%d",
           I2C_SLAVE_ADDR,
           (int)I2C_SLAVE_SDA_IO,
           (int)I2C_SLAVE_SCL_IO);
  return ESP_OK;
}

static void adc_capture_task(void* arg) {
  uint16_t seq = 0;
  ServoSnapshot next = {};
  uint32_t last_debug_log_ms = 0;
  int adc_raw_cache[SERVO_ID_COUNT] = {0};
  int adc_norm_cache[SERVO_ID_COUNT] = {0};
  int adc_phase_cache[SERVO_ID_COUNT] = {0};
  bool valid_cache[SERVO_ID_COUNT] = {false};
  int16_t hold_send_deg_x10[SERVO_ID_COUNT] = {0};

  while (true) {
    next.seq = ++seq;
    next.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    for (size_t idx = 0; idx < SERVO_ID_COUNT; idx++) {
      float adc_deg = clampf(readChannel((uint8_t)idx), 0.0f, 360.0f);
      int adc_raw = adc_deg_to_code(adc_deg);
      int adc_norm = normalize_adc_code(idx, adc_raw);
      int adc_phase = wrap_code(adc_norm + ADC_PHASE_OFFSET[idx]);
      adc_raw_cache[idx] = adc_raw;
      adc_norm_cache[idx] = adc_norm;
      adc_phase_cache[idx] = adc_phase;

      float send_deg = 0.0f;
      bool valid = map_adc_code_to_send_deg(adc_phase, &send_deg);
      valid_cache[idx] = valid;
      if (valid) {
        int send_deg_x10 = (int)roundf(send_deg * 10.0f);
        hold_send_deg_x10[idx] = (int16_t)clampi(send_deg_x10, -900, 900);
      }
      // 无效区间保持上一有效值，不更新输出（屏蔽发送变化）
      next.send_deg_x10[idx] = hold_send_deg_x10[idx];
    }

    if (xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      s_latest_snapshot = next;
      xSemaphoreGive(s_snapshot_mutex);
    }

    if (next.uptime_ms - last_debug_log_ms >= ADC_DEBUG_LOG_INTERVAL_MS) {
      last_debug_log_ms = next.uptime_ms;
      char line[256] = {0};
      int n = snprintf(line, sizeof(line), "MAP seq=%u ", (unsigned)next.seq);
      size_t used = (n > 0) ? (size_t)n : 0U;
      for (size_t i = 0; i < DEBUG_SERVO_ID_COUNT; i++) {
        uint8_t sid = DEBUG_SERVO_IDS[i];
        int idx = (int)sid - (int)SERVO_ID_MIN;
        if (idx < 0 || idx >= (int)SERVO_ID_COUNT) continue;

        int adc_raw = adc_raw_cache[idx];
        int adc_norm = adc_norm_cache[idx];
        int adc_phase = adc_phase_cache[idx];
        float send_deg = next.send_deg_x10[idx] / 10.0f;
        if (valid_cache[idx]) {
          float servo_deg = send_deg_to_servo_deg(sid, send_deg);
          n = snprintf(line + used,
                       (used < sizeof(line)) ? (sizeof(line) - used) : 0,
                       "%u:%d/%d/%d->%.1f->%.1f ",
                       (unsigned)sid,
                       adc_raw,
                       adc_norm,
                       adc_phase,
                       servo_deg,
                       send_deg);
        } else {
          n = snprintf(line + used,
                       (used < sizeof(line)) ? (sizeof(line) - used) : 0,
                       "%u:%d/%d/%d->MASK(%.1f) ",
                       (unsigned)sid,
                       adc_raw,
                       adc_norm,
                       adc_phase,
                       send_deg);
        }
        if (n > 0) {
          size_t inc = (size_t)n;
          if (used + inc < sizeof(line)) {
            used += inc;
          } else {
            used = sizeof(line) - 1;
          }
        }

        if (i == 4) {
          ESP_LOGI(TAG, "%s", line);
          line[0] = '\0';
          used = 0;
        }
      }
      if (used > 0) {
        ESP_LOGI(TAG, "%s", line);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(ADC_CAPTURE_INTERVAL_MS));
  }
}

static void i2c_slave_task(void* arg) {
  uint8_t cmd_buf[16] = {0};
  uint8_t frame[FRAME_SIZE] = {0};

  while (true) {
    int rx_len = i2c_slave_read_buffer(
        I2C_SLAVE_PORT,
        cmd_buf,
        sizeof(cmd_buf),
        pdMS_TO_TICKS(I2C_CMD_WAIT_MS));

    if (rx_len <= 0) {
      continue;
    }

    for (int i = 0; i < rx_len; i++) {
      if (cmd_buf[i] != I2C_CMD_GET_FRAME) {
        continue;
      }

      size_t frame_len = build_adc_frame(frame, sizeof(frame));
      if (frame_len != FRAME_SIZE) {
        ESP_LOGW(TAG, "build frame failed, len=%u", (unsigned)frame_len);
        continue;
      }

      int tx_len = i2c_slave_write_buffer(
          I2C_SLAVE_PORT,
          frame,
          (int)frame_len,
          pdMS_TO_TICKS(20));

      if (tx_len != (int)frame_len) {
        ESP_LOGW(TAG, "i2c tx short write: tx=%d expect=%u", tx_len, (unsigned)frame_len);
      }
    }
  }
}

extern "C" void app_main(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  s_snapshot_mutex = xSemaphoreCreateMutex();
  if (!s_snapshot_mutex) {
    ESP_LOGE(TAG, "create snapshot mutex failed");
    return;
  }

  ensureMuxADC();

  if (init_i2c_slave() != ESP_OK) {
    return;
  }

  xTaskCreate(adc_capture_task, "adc_capture", 4096, nullptr, 5, nullptr);
  xTaskCreate(i2c_slave_task, "i2c_slave", 4096, nullptr, 6, nullptr);

  ESP_LOGI(TAG,
           "ESP32 bridge started: adc_channels=%u, servo_id_range=%u-%u",
           (unsigned)SERVO_ID_COUNT,
           (unsigned)SERVO_ID_MIN,
           (unsigned)SERVO_ID_MAX);
}
