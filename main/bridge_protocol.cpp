// I2C 帧协议实现：序列化 + CRC16
#include "bridge_protocol.h"

#include <string.h>

// 计算 CRC-16/CCITT-FALSE（对齐 raspi/bridge.py 的实现）。
//
// 说明：
// - 这是协议内部实现细节，业务侧不应直接调用，因此不在头文件暴露符号。
// - 参数与算法固定：init=0xFFFF, poly=0x1021, refin/refout=false, xorout=0
static uint16_t bridge_crc16_ccitt_false(const uint8_t* data, size_t len) {
  if (!data && len != 0) return 0;

  uint16_t crc = 0xFFFF;
  for (size_t byte_index = 0; byte_index < len; byte_index++) {
    crc ^= (uint16_t)data[byte_index] << 8;
    for (int bit_index = 0; bit_index < 8; bit_index++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

static inline void write_u16_le(uint8_t* dst, uint16_t value) {
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)((value >> 8) & 0xFF);
}

static inline void write_i16_le(uint8_t* dst, int16_t value) {
  write_u16_le(dst, (uint16_t)value);
}

static inline void write_u32_le(uint8_t* dst, uint32_t value) {
  dst[0] = (uint8_t)(value & 0xFF);
  dst[1] = (uint8_t)((value >> 8) & 0xFF);
  dst[2] = (uint8_t)((value >> 16) & 0xFF);
  dst[3] = (uint8_t)((value >> 24) & 0xFF);
}

extern "C" {

esp_err_t bridge_build_frame(const bridge_snapshot_t* snapshot,
                            uint8_t* out_frame,
                            size_t out_capacity,
                            size_t* out_len) {
  if (out_len) *out_len = 0;
  if (!snapshot || !out_frame) return ESP_ERR_INVALID_ARG;
  if (out_capacity < BRIDGE_FRAME_SIZE) return ESP_ERR_INVALID_ARG;

  // 先清零，确保 reserved 字段与未覆盖区域稳定可预期（便于抓包比对）。
  memset(out_frame, 0, BRIDGE_FRAME_SIZE);

  out_frame[BRIDGE_FRAME_OFFSET_MAGIC0] = (uint8_t)BRIDGE_FRAME_MAGIC0;
  out_frame[BRIDGE_FRAME_OFFSET_MAGIC1] = (uint8_t)BRIDGE_FRAME_MAGIC1;
  out_frame[BRIDGE_FRAME_OFFSET_VERSION] = (uint8_t)BRIDGE_FRAME_VERSION;
  out_frame[BRIDGE_FRAME_OFFSET_CHANNEL_COUNT] = (uint8_t)BRIDGE_CHANNEL_COUNT;

  write_u16_le(out_frame + BRIDGE_FRAME_OFFSET_SEQ, snapshot->seq);
  write_u32_le(out_frame + BRIDGE_FRAME_OFFSET_UPTIME_MS, snapshot->uptime_ms);

  out_frame[BRIDGE_FRAME_OFFSET_SERVO_ID_MIN] = (uint8_t)BRIDGE_SERVO_ID_MIN;
  out_frame[BRIDGE_FRAME_OFFSET_RESERVED] = 0;

  // 写入每路角度（0.1 度单位）。
  for (int i = 0; i < BRIDGE_CHANNEL_COUNT; i++) {
    const size_t off = BRIDGE_FRAME_OFFSET_SEND_DEG_X10 + (size_t)i * 2;
    write_i16_le(out_frame + off, snapshot->send_deg_x10[i]);
  }

  // CRC 覆盖前 46 字节。
  const uint16_t crc16 = bridge_crc16_ccitt_false(out_frame, (size_t)BRIDGE_FRAME_OFFSET_CRC16);
  write_u16_le(out_frame + BRIDGE_FRAME_OFFSET_CRC16, crc16);

  if (out_len) *out_len = BRIDGE_FRAME_SIZE;
  return ESP_OK;
}

} // extern "C"
