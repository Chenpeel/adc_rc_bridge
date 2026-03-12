#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "adc_mux_sampler.h"
#include "bridge_protocol.h"

static const char *TAG = "esp_bridge";

// ================================================================
// 本文件实现 ESP32 端的“采样 -> 映射 -> 快照 -> I2C 从机回传”闭环：
//
// 1) adc_capture_task:
//    - 周期性读取 17 路逻辑通道（底层由 adc_mux_sampler 模块驱动 MUX + ADC oneshot）
//    - 将 ADC 码值做归一化/相位偏移（方便每路标定）
//    - 将处理后的码值映射成 send_deg（-90~90 度），并以 0.1 度单位量化成 int16
//    - 更新全局快照 s_latest_snapshot（受 mutex 保护）
//
// 2) i2c_slave_task:
//    - 作为 I2C 从机等待主机命令
//    - 收到 0xA5 后用 bridge_protocol 模块把最新快照打包成固定 48 字节帧并写入 TX buffer
//
// Debug 建议（先看这里，再看具体注释）：
// - 如果采样值抖动：优先调 adc_mux_sampler 的 settle_delay_us/discard_read_count/sample_count；其次调 ADC_CAPTURE_INTERVAL_MS
// - 如果某一路永远不变：核对“servo_id -> servo_index -> logical_channel”是否与硬件通道对应
// - 如果某一路方向反了：raspi/config.json 的 servo_reverse_map 表示“角度取反”，即 angle = -angle；
//   它不是简单的“把 -90 和 +90 对调”，而是对整个区间做符号翻转（30 -> -30, -45 -> +45）
// - 如果 I2C 读不到数据：先确认主机是否写入 0xA5，再发起 read；检查 SDA/SCL 上拉与地址 0x28
// ================================================================

// ====== ADC/舵机通道定义 ======
// 这里的“servo_id”是外部系统使用的舵机编号（21~37），而内部数组索引 idx 是 0~16。
// - servo_id = SERVO_ID_MIN + idx
// - 本工程将 idx 直接作为采样模块的 logical_channel：
//   idx 0~15 -> MUX1 通道 0~15
//   idx 16   -> MUX2 通道 0（logical_channel >= 16 时落到第二片 MUX）
static const uint8_t SERVO_ID_MIN = (uint8_t)BRIDGE_SERVO_ID_MIN;
static const uint8_t SERVO_ID_MAX = (uint8_t)BRIDGE_SERVO_ID_MAX;
static const size_t SERVO_COUNT = (size_t)BRIDGE_CHANNEL_COUNT; // 17 路
static_assert(SERVO_COUNT == (size_t)BRIDGE_CHANNEL_COUNT, "SERVO_COUNT mismatch");

// ====== I2C从机配置 ======
// I2C 交互约定（主机侧需要按这个时序来）：
// 1) 主机向从机地址 0x28 写入单字节命令 0xA5
// 2) 主机随后从同一地址读取 48 字节数据帧（BRIDGE_FRAME_SIZE）
static const i2c_port_t I2C_SLAVE_PORT = I2C_NUM_0;
static const gpio_num_t I2C_SLAVE_SDA_IO = GPIO_NUM_21;
static const gpio_num_t I2C_SLAVE_SCL_IO = GPIO_NUM_22;
static const uint8_t I2C_SLAVE_ADDR = 0x28;
static const uint8_t I2C_CMD_GET_FRAME = (uint8_t)BRIDGE_I2C_CMD_GET_FRAME;
static const size_t I2C_SLAVE_RX_BUF_LEN = 128;
static const size_t I2C_SLAVE_TX_BUF_LEN = 256;

// ====== 采样与任务配置 ======
// 采样任务节拍。注意：这是 vTaskDelay 的延时参数，不包含采样本身耗时；
// 实际周期 ~= 采样耗时 + ADC_CAPTURE_INTERVAL_MS。
static const uint32_t ADC_CAPTURE_INTERVAL_MS = 15; // ~66Hz
static const uint32_t I2C_CMD_WAIT_MS = 100;
static const uint32_t ADC_DEBUG_LOG_INTERVAL_MS = 1000;
static const float SEND_MIN_DEG = -90.0f;
static const float SEND_MAX_DEG = 90.0f;
static const int ADC_CODE_MAX = 4095;
// “圆周角度”的分段断点（单位：deg）。这里只用作折线映射的分段依据。
static const int CIRCLE_ANGLE_DEG_90 = 90;
static const int CIRCLE_ANGLE_DEG_180 = 180;
static const int CIRCLE_ANGLE_DEG_270 = 270;
static const int CIRCLE_ANGLE_DEG_360 = 360;
// 每路归一化区间（原始 ADC 码值，0~4095）
//
// 用途：
// - 不同通道的传感器/分压可能导致“有效范围”并非完整 0~4095
// - 这里允许你为每路指定一个 [min,max]，然后把它线性拉伸/压缩到 [0,4095]
//
// Debug：
// - 如果某路输出总在两端饱和，优先检查该路的 ADC_CALIB_MIN_CODE/ADC_CALIB_MAX_CODE 是否设置合理
static const int ADC_CALIB_MIN_CODE[SERVO_COUNT] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const int ADC_CALIB_MAX_CODE[SERVO_COUNT] = {
    4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
    4095, 4095, 4095, 4095, 4095};
// 每路相位偏移（单位：ADC码值，正值=向更大码值方向平移，负值相反）
// 用途：
// - 某些角度传感器是周期量（0/360 度等价），你可以用 phase offset 把“零位”对齐到期望位置
// - wrap_adc_code 会把结果包回 0~4095，避免越界
static const int ADC_PHASE_OFFSET_CODE[SERVO_COUNT] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// 定期打印这些舵机的映射链路（原始/归一化/相位/最终角度），用于现场快速定位是哪一段出了问题。
static const uint8_t DEBUG_SERVO_IDS[] = {21, 22, 23, 32, 33, 34, 35, 36, 37};
static const size_t DEBUG_SERVO_ID_COUNT = sizeof(DEBUG_SERVO_IDS) / sizeof(DEBUG_SERVO_IDS[0]);

// ====== 数据帧协议 ======
// 帧结构、字段偏移、CRC16 算法统一由 bridge_protocol 模块维护：
// - bridge_protocol.h
// - bridge_protocol.cpp
static_assert(BRIDGE_FRAME_SIZE == 48, "unexpected frame size");

static bridge_snapshot_t s_latest_snapshot;
static SemaphoreHandle_t s_snapshot_mutex = nullptr;

// 采样硬件模块句柄（只在采样任务里使用）。
static adc_mux_sampler_t s_adc_sampler;

static inline int clamp_int(int value, int min_value, int max_value)
{
  if (value < min_value)
    return min_value;
  if (value > max_value)
    return max_value;
  return value;
}

static inline int wrap_adc_code(int adc_code)
{
  // 将任意整数包回 [0, 4095]。
  // 用途：相位偏移后可能出现负数或超过 4095，这里通过取模实现“周期量”的 wrap。
  const int adc_code_range = ADC_CODE_MAX + 1;
  int wrapped_code = adc_code % adc_code_range;
  if (wrapped_code < 0)
    wrapped_code += adc_code_range;
  return wrapped_code;
}

static inline int normalize_adc_raw_code(size_t servo_index, int raw_adc_code)
{
  // 每路独立归一化：将 [ADC_CALIB_MIN_CODE[servo_index], ADC_CALIB_MAX_CODE[servo_index]] 映射到 [0,4095]。
  //
  // 关键点：
  // - lo/hi 会被夹到合法范围，并保证 hi >= lo+1，避免除 0
  // - raw_code 会先夹到 [lo,hi]，因此超出区间的值会在归一化后饱和到 0 或 4095
  const int range_min_code = clamp_int(ADC_CALIB_MIN_CODE[servo_index], 0, ADC_CODE_MAX - 1);
  const int range_max_code =
      clamp_int(ADC_CALIB_MAX_CODE[servo_index], range_min_code + 1, ADC_CODE_MAX);
  const int raw_code_clamped = clamp_int(raw_adc_code, range_min_code, range_max_code);
  const int normalized_code =
      (int)roundf((float)(raw_code_clamped - range_min_code) * (float)ADC_CODE_MAX /
                  (float)(range_max_code - range_min_code));
  return clamp_int(normalized_code, 0, ADC_CODE_MAX);
}

static inline bool map_circle_angle_to_send_deg(int adc_phase_code, float *out_send_deg)
{
  if (!out_send_deg)
    return false;

  // 这里的输入不是“机械角度”，而是一个周期量的相位表达：
  // - adc_phase_code 是 0~4095 的环形码值（0 与 4095 在物理意义上相邻）
  // - 后续会先线性换算为 circle_angle_deg（0~360），再进行折线映射
  const int adc_phase_code_clamped = clamp_int(adc_phase_code, 0, ADC_CODE_MAX);
  const float circle_angle_deg =
      (float)adc_phase_code_clamped * (float)CIRCLE_ANGLE_DEG_360 / (float)ADC_CODE_MAX;

  // 周期折线映射(类似正弦的折线近似)，折点如下：
  //  0deg  ->  90deg  ->  180deg  ->  270deg  ->  360deg
  //  0deg  ->  90deg  ->  0deg    -> -90deg  ->  0deg
  //
  // 结果：输出在 -90~90 之间往返摆动（“圆周量 -> 往复量”）。
  if (circle_angle_deg <= (float)CIRCLE_ANGLE_DEG_90)
  {
    // 0 -> 90 : 0 -> +90
    const float t = circle_angle_deg / (float)CIRCLE_ANGLE_DEG_90;
    *out_send_deg = 0.0f + t * SEND_MAX_DEG;
    return true;
  }
  if (circle_angle_deg <= (float)CIRCLE_ANGLE_DEG_180)
  {
    // 90 -> 180 : +90 -> 0
    const float t = (circle_angle_deg - (float)CIRCLE_ANGLE_DEG_90) /
                    (float)(CIRCLE_ANGLE_DEG_180 - CIRCLE_ANGLE_DEG_90);
    *out_send_deg = SEND_MAX_DEG + t * (0.0f - SEND_MAX_DEG);
    return true;
  }
  if (circle_angle_deg <= (float)CIRCLE_ANGLE_DEG_270)
  {
    // 180 -> 270 : 0 -> -90
    const float t = (circle_angle_deg - (float)CIRCLE_ANGLE_DEG_180) /
                    (float)(CIRCLE_ANGLE_DEG_270 - CIRCLE_ANGLE_DEG_180);
    *out_send_deg = 0.0f + t * SEND_MIN_DEG;
    return true;
  }

  // 270 -> 360 : -90 -> 0
  const float t = (circle_angle_deg - (float)CIRCLE_ANGLE_DEG_270) /
                  (float)(CIRCLE_ANGLE_DEG_360 - CIRCLE_ANGLE_DEG_270);
  *out_send_deg = SEND_MIN_DEG + t * (0.0f - SEND_MIN_DEG);
  return true;
}

static bool copy_latest_snapshot(bridge_snapshot_t *out_snapshot)
{
  // 复制最新快照（短临界区）。
  // 这里锁的时间非常短：只做结构体拷贝，避免阻塞采样任务或 I2C 任务太久。
  if (!out_snapshot || !s_snapshot_mutex)
    return false;
  if (xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(5)) != pdTRUE)
  {
    return false;
  }
  *out_snapshot = s_latest_snapshot;
  xSemaphoreGive(s_snapshot_mutex);
  return true;
}

static void reset_i2c_slave_fifos()
{
  // legacy I2C 从机驱动内部同时有硬件 FIFO 和软件 ring buffer。
  // 如果主机轮询比从机处理更快、或者上一次读事务没有按预期消费完整帧，
  // 旧数据可能残留在 TX/RX 缓冲里，下一次再写 48 字节就会出现“帧拼接”：
  //   [旧帧尾][新帧头]
  // 树莓派侧看起来就会表现为 magic/version 偶发错误，且日志常呈现周期性抖动。
  //
  // 因此这里在每次处理 GET_FRAME 命令前主动清 FIFO，确保：
  // - TX 里只保留“本次命令对应的一帧”
  // - RX 里不残留历史命令字节
  const esp_err_t tx_reset_err = i2c_reset_tx_fifo(I2C_SLAVE_PORT);
  if (tx_reset_err != ESP_OK)
  {
    ESP_LOGW(TAG, "i2c_reset_tx_fifo failed: %s", esp_err_to_name(tx_reset_err));
  }

  const esp_err_t rx_reset_err = i2c_reset_rx_fifo(I2C_SLAVE_PORT);
  if (rx_reset_err != ESP_OK)
  {
    ESP_LOGW(TAG, "i2c_reset_rx_fifo failed: %s", esp_err_to_name(rx_reset_err));
  }
}

static esp_err_t init_i2c_slave()
{
  // 初始化 I2C 从机。
  // Debug：
  // - 如果主机扫不到地址：优先检查 SDA/SCL 是否接反、是否有上拉、GPIO 是否与硬件一致
  // - 如果通信偶发错误：检查线长/上拉电阻/地线；必要时降低主机时钟
  i2c_config_t i2c_cfg = {};
  i2c_cfg.mode = I2C_MODE_SLAVE;
  i2c_cfg.sda_io_num = I2C_SLAVE_SDA_IO;
  i2c_cfg.scl_io_num = I2C_SLAVE_SCL_IO;
  i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.slave.addr_10bit_en = 0;
  i2c_cfg.slave.slave_addr = I2C_SLAVE_ADDR;

  esp_err_t err = i2c_param_config(I2C_SLAVE_PORT, &i2c_cfg);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
    return err;
  }

  err = i2c_driver_install(I2C_SLAVE_PORT,
                           i2c_cfg.mode,
                           (int)I2C_SLAVE_RX_BUF_LEN,
                           (int)I2C_SLAVE_TX_BUF_LEN,
                           0);
  if (err != ESP_OK)
  {
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

static void adc_capture_task(void *arg)
{
  (void)arg;

  // 周期采样任务：把“物理采样值(ADC原始码值)”转换为“可发送的角度快照”。
  //
  // 输出链路（每路）：
  // adc_mux_sampler_read_raw_code(servo_index) -> adc_raw_code(0~4095)
  //   -> normalize_adc_raw_code(servo_index, adc_raw_code) -> adc_normalized_code(0~4095)
  //   -> wrap_adc_code(adc_normalized_code + ADC_PHASE_OFFSET_CODE[servo_index]) -> adc_phase_code(0~4095)
  //   -> map_circle_angle_to_send_deg(adc_phase_code) -> send_angle_deg(-90~90)
  //   -> send_deg_x10(int16) -> next_snapshot.send_deg_x10[servo_index]
  //
  // 设计点：
  // - 直接读取 ADC 原始码值，避免“角度 -> 码值 -> 角度”的来回转换与舍入误差
  // - 读数失败/异常时，保持上一帧有效输出，避免主机侧控制量突然跳变
  uint16_t sample_seq = 0;
  bridge_snapshot_t next_snapshot = {};
  uint32_t last_debug_log_time_ms = 0;

  adc_mux_sampler_read_opts_t read_opts = adc_mux_sampler_default_read_opts();
  read_opts.sample_count = ADC_MUX_SAMPLER_DEFAULT_SAMPLE_COUNT;
  read_opts.trim_extremes = true;

  int adc_raw_code_cache[SERVO_COUNT] = {0};
  int adc_normalized_code_cache[SERVO_COUNT] = {0};
  int adc_phase_code_cache[SERVO_COUNT] = {0};
  int adc_read_err_cache[SERVO_COUNT] = {0}; // ESP_OK=0，其他为错误码
  int16_t last_send_deg_x10[SERVO_COUNT] = {0};

  while (true)
  {
    next_snapshot.seq = ++sample_seq;
    // esp_timer_get_time() 单位是 us，这里转成 ms 方便与日志/主机侧对齐。
    next_snapshot.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    for (size_t servo_index = 0; servo_index < SERVO_COUNT; servo_index++)
    {
      // servo_index 是 0~16，对应 servo_id = SERVO_ID_MIN + servo_index。
      // 采样函数会切换 MUX 地址线，因此该循环内不要做并发/阻塞操作，保持节拍稳定。
      int adc_raw_code = 0;
      const esp_err_t read_err = adc_mux_sampler_read_raw_code(
          &s_adc_sampler, (uint8_t)servo_index, &read_opts, &adc_raw_code);
      adc_read_err_cache[servo_index] = (int)read_err;

      if (read_err != ESP_OK)
      {
        // 读数失败：本次不更新该通道输出，直接保持上一有效值。
        next_snapshot.send_deg_x10[servo_index] = last_send_deg_x10[servo_index];
        continue;
      }

      adc_raw_code_cache[servo_index] = adc_raw_code;
      const int adc_normalized_code = normalize_adc_raw_code(servo_index, adc_raw_code);
      const int adc_phase_code =
          wrap_adc_code(adc_normalized_code + ADC_PHASE_OFFSET_CODE[servo_index]);
      adc_normalized_code_cache[servo_index] = adc_normalized_code;
      adc_phase_code_cache[servo_index] = adc_phase_code;

      float send_angle_deg = 0.0f;
      if (!map_circle_angle_to_send_deg(adc_phase_code, &send_angle_deg))
      {
        next_snapshot.send_deg_x10[servo_index] = last_send_deg_x10[servo_index];
        continue;
      }

      // 量化到 0.1 度（int16），并夹到 [-900,900]，避免异常值把帧结构搞乱。
      const int send_deg_x10_i32 = (int)roundf(send_angle_deg * 10.0f);
      const int16_t send_deg_x10 = (int16_t)clamp_int(send_deg_x10_i32, -900, 900);
      last_send_deg_x10[servo_index] = send_deg_x10;
      next_snapshot.send_deg_x10[servo_index] = send_deg_x10;
    }

    // 更新全局快照。拿不到锁就跳过本次更新（宁可丢一次更新，也不要长时间阻塞采样任务）。
    if (xSemaphoreTake(s_snapshot_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
      s_latest_snapshot = next_snapshot;
      xSemaphoreGive(s_snapshot_mutex);
    }

    if (next_snapshot.uptime_ms - last_debug_log_time_ms >= ADC_DEBUG_LOG_INTERVAL_MS)
    {
      last_debug_log_time_ms = next_snapshot.uptime_ms;

      // 定期输出关键通道的“链路中间态”，用于排查：
      // - ADC 读数是否成功（E(xxx)）
      // - 原始 ADC 是否正常波动（adc_raw_code）
      // - 归一化参数是否正确（adc_normalized_code）
      // - 相位偏移是否生效（adc_phase_code）
      // - 最终发送角度是否符合预期（send_angle_deg）
      char log_line[256] = {0};
      int written =
          snprintf(log_line, sizeof(log_line), "MAP seq=%u ", (unsigned)next_snapshot.seq);
      size_t used_len = (written > 0) ? (size_t)written : 0U;

      for (size_t debug_index = 0; debug_index < DEBUG_SERVO_ID_COUNT; debug_index++)
      {
        const uint8_t servo_id = DEBUG_SERVO_IDS[debug_index];
        const int servo_index = (int)servo_id - (int)SERVO_ID_MIN;
        if (servo_index < 0 || servo_index >= (int)SERVO_COUNT)
          continue;

        const int read_err = adc_read_err_cache[servo_index];
        const int adc_raw_code = adc_raw_code_cache[servo_index];
        const int adc_normalized_code = adc_normalized_code_cache[servo_index];
        const int adc_phase_code = adc_phase_code_cache[servo_index];
        const float send_angle_deg = next_snapshot.send_deg_x10[servo_index] / 10.0f;

        if (read_err != ESP_OK)
        {
          written = snprintf(log_line + used_len,
                             (used_len < sizeof(log_line)) ? (sizeof(log_line) - used_len) : 0,
                             "%u:E(%d) ",
                             (unsigned)servo_id,
                             read_err);
        }
        else
        {
          written = snprintf(log_line + used_len,
                             (used_len < sizeof(log_line)) ? (sizeof(log_line) - used_len) : 0,
                             "%u:%d/%d/%d->%.1f ",
                             (unsigned)servo_id,
                             adc_raw_code,
                             adc_normalized_code,
                             adc_phase_code,
                             send_angle_deg);
        }

        if (written > 0)
        {
          const size_t inc = (size_t)written;
          if (used_len + inc < sizeof(log_line))
          {
            used_len += inc;
          }
          else
          {
            used_len = sizeof(log_line) - 1;
          }
        }

        if (debug_index == 4)
        {
          // 分两行打印，避免单行太长导致日志截断（也便于肉眼对齐观察）。
          ESP_LOGI(TAG, "%s", log_line);
          log_line[0] = '\0';
          used_len = 0;
        }
      }

      if (used_len > 0)
      {
        ESP_LOGI(TAG, "%s", log_line);
      }
    }

    // 固定延时。注意：FreeRTOS 的延时精度受 tick 影响；若你对频率很敏感，可改用 vTaskDelayUntil。
    vTaskDelay(pdMS_TO_TICKS(ADC_CAPTURE_INTERVAL_MS));
  }
}

static void i2c_slave_task(void *arg)
{
  (void)arg;

  // I2C 从机服务任务：
  // - RX：接收主机写入的命令字节（可以一次写多个字节，因此这里扫描 cmd_bytes）
  // - TX：遇到命令 0xA5 时，构建最新数据帧并写入从机 TX buffer，等待主机读取
  uint8_t cmd_bytes[16] = {0};
  uint8_t tx_frame[BRIDGE_FRAME_SIZE] = {0};

  while (true)
  {
    const int rx_bytes = i2c_slave_read_buffer(
        I2C_SLAVE_PORT,
        cmd_bytes,
        sizeof(cmd_bytes),
        pdMS_TO_TICKS(I2C_CMD_WAIT_MS));

    if (rx_bytes <= 0)
    {
      continue;
    }

    bool has_get_frame_cmd = false;
    for (int byte_index = 0; byte_index < rx_bytes; byte_index++)
    {
      if (cmd_bytes[byte_index] == I2C_CMD_GET_FRAME)
      {
        has_get_frame_cmd = true;
        break;
      }
    }

    if (!has_get_frame_cmd)
    {
      continue;
    }

    // 收到请求后先清掉历史残留，确保主机本次 read 读到的是“完整且唯一”的最新一帧。
    reset_i2c_slave_fifos();

    // 只在收到命令时才构建帧，避免空转浪费 CPU。
    bridge_snapshot_t snapshot = {};
    if (!copy_latest_snapshot(&snapshot))
    {
      ESP_LOGW(TAG, "snapshot not ready");
      continue;
    }

    size_t tx_frame_len = 0;
    const esp_err_t frame_err =
        bridge_build_frame(&snapshot, tx_frame, sizeof(tx_frame), &tx_frame_len);
    if (frame_err != ESP_OK || tx_frame_len != BRIDGE_FRAME_SIZE)
    {
      ESP_LOGW(TAG,
               "build frame failed: err=%s len=%u",
               esp_err_to_name(frame_err),
               (unsigned)tx_frame_len);
      continue;
    }

    // i2c_slave_write_buffer 写入的是“从机内部 TX ring buffer”，不是直接发到总线上。
    // 主机随后发起 read 时会从这里取数据。
    const int tx_bytes = i2c_slave_write_buffer(
        I2C_SLAVE_PORT,
        tx_frame,
        (int)tx_frame_len,
        pdMS_TO_TICKS(20));

    if (tx_bytes != (int)tx_frame_len)
    {
      ESP_LOGW(TAG,
               "i2c tx short write: tx=%d expect=%u seq=%u",
               tx_bytes,
               (unsigned)tx_frame_len,
               (unsigned)snapshot.seq);
    }
  }
}

extern "C" void app_main(void)
{
  // ESP-IDF 入口。此处主要做资源初始化与任务启动。
  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  s_snapshot_mutex = xSemaphoreCreateMutex();
  if (!s_snapshot_mutex)
  {
    ESP_LOGE(TAG, "create snapshot mutex failed");
    return;
  }

  // 初始化采样硬件（GPIO + ADC oneshot）。
  // 若初始化失败，采样任务会一直读不到数据，直接在这里 fail-fast 更容易定位问题。
  adc_mux_sampler_config_t sampler_cfg = adc_mux_sampler_default_config();
  esp_err_t sampler_err = adc_mux_sampler_init(&s_adc_sampler, &sampler_cfg);
  if (sampler_err != ESP_OK)
  {
    ESP_LOGE(TAG, "adc sampler init failed: %s", esp_err_to_name(sampler_err));
    return;
  }

  if (init_i2c_slave() != ESP_OK)
  {
    return;
  }

  xTaskCreate(adc_capture_task, "adc_capture", 4096, nullptr, 5, nullptr);
  xTaskCreate(i2c_slave_task, "i2c_slave", 4096, nullptr, 6, nullptr);

  ESP_LOGI(TAG,
           "ESP32 bridge started: adc_channels=%u, servo_id_range=%u-%u",
           (unsigned)SERVO_COUNT,
           (unsigned)SERVO_ID_MIN,
           (unsigned)SERVO_ID_MAX);
}
