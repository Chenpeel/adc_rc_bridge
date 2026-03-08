#include <inttypes.h>
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

// ====== 数据帧协议 ======
// 帧格式(共60字节, 小端):
// [0]   magic0      = 0xAA
// [1]   magic1      = 0x55
// [2]   version     = 1
// [3]   channel_cnt = 23
// [4:6] seq
// [6:10]uptime_ms
// [10]  servo_id_min=21
// [11]  reserved    =0
// [12:58] angles_x10[23] (uint16, 单位0.1度)
// [58:60] crc16_ccitt_false (对前58字节计算)
static const uint8_t FRAME_MAGIC0 = 0xAA;
static const uint8_t FRAME_MAGIC1 = 0x55;
static const uint8_t FRAME_VERSION = 1;
static const size_t FRAME_SIZE = 60;
static_assert(FRAME_SIZE == (12 + SERVO_ID_COUNT * 2 + 2), "unexpected frame size");

struct ServoSnapshot {
  uint16_t seq = 0;
  uint32_t uptime_ms = 0;
  uint16_t angles_x10[SERVO_ID_COUNT] = {0};
};

static ServoSnapshot s_latest_snapshot;
static SemaphoreHandle_t s_snapshot_mutex = nullptr;

static inline void write_u16_le(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
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
    write_u16_le(frame + pos, snap.angles_x10[i]);
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

  while (true) {
    next.seq = ++seq;
    next.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    for (size_t idx = 0; idx < SERVO_ID_COUNT; idx++) {
      float angle = readChannel((uint8_t)idx);
      int angle_x10 = (int)(angle * 10.0f + 0.5f);
      next.angles_x10[idx] = (uint16_t)clampi(angle_x10, 0, 3600);
    }

    if (xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      s_latest_snapshot = next;
      xSemaphoreGive(s_snapshot_mutex);
    }

    if (next.uptime_ms - last_debug_log_ms >= ADC_DEBUG_LOG_INTERVAL_MS) {
      last_debug_log_ms = next.uptime_ms;
      ESP_LOGI(TAG,
               "ADC seq=%u ch21=%.1f ch22=%.1f ch23=%.1f",
               (unsigned)next.seq,
               next.angles_x10[0] / 10.0f,
               next.angles_x10[1] / 10.0f,
               next.angles_x10[2] / 10.0f);
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
