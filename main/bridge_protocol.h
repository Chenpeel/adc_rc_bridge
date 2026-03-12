#pragma once
// ================================================================
// I2C 数据帧协议（ESP32 <-> Raspberry Pi）
//
// 目标：
// - 将“帧格式/字段偏移/CRC 算法”集中管理，避免散落在业务代码里难以维护
// - API 标准化：函数返回 esp_err_t，数据通过 out 参数返回
//
// 帧格式（固定 48 字节，小端）：
// [0]     magic0        = 0xAA
// [1]     magic1        = 0x55
// [2]     version       = 2
// [3]     channel_cnt   = 17
// [4:6]   seq           uint16
// [6:10]  uptime_ms     uint32
// [10]    servo_id_min  = 21
// [11]    reserved      = 0
// [12:46] send_deg_x10  int16[channel_cnt]  (单位 0.1 度，范围 -900~900)
// [46:48] crc16         CRC-16/CCITT-FALSE，对前 46 字节计算
//
// CRC 参数：
// - init=0xFFFF
// - poly=0x1021
// - refin=false, refout=false
// - xorout=0x0000
//
// 注意：
// - 这份协议需要与 raspi/bridge.py 中的解析逻辑保持一致。
// ================================================================

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Servo id 区间（与 raspi 侧一致）。
enum {
  BRIDGE_SERVO_ID_MIN = 21,
  BRIDGE_SERVO_ID_MAX = 37,
  BRIDGE_CHANNEL_COUNT = (BRIDGE_SERVO_ID_MAX - BRIDGE_SERVO_ID_MIN + 1), // 17
};

// 帧常量。
enum {
  BRIDGE_FRAME_MAGIC0 = 0xAA,
  BRIDGE_FRAME_MAGIC1 = 0x55,
  BRIDGE_FRAME_VERSION = 2,
  BRIDGE_FRAME_SIZE = 48,
  BRIDGE_FRAME_HEADER_SIZE = 12,
};

// I2C 交互命令字。
// 主机向从机写入该字节后，再读取固定 48 字节数据帧。
enum {
  BRIDGE_I2C_CMD_GET_FRAME = 0xA5,
};

// 字段偏移（用于抓包/断点对齐）。
enum {
  BRIDGE_FRAME_OFFSET_MAGIC0 = 0,
  BRIDGE_FRAME_OFFSET_MAGIC1 = 1,
  BRIDGE_FRAME_OFFSET_VERSION = 2,
  BRIDGE_FRAME_OFFSET_CHANNEL_COUNT = 3,
  BRIDGE_FRAME_OFFSET_SEQ = 4,
  BRIDGE_FRAME_OFFSET_UPTIME_MS = 6,
  BRIDGE_FRAME_OFFSET_SERVO_ID_MIN = 10,
  BRIDGE_FRAME_OFFSET_RESERVED = 11,
  BRIDGE_FRAME_OFFSET_SEND_DEG_X10 = 12,
  BRIDGE_FRAME_OFFSET_CRC16 = 46,
};

// 采样快照：业务侧生成该结构体，然后通过 bridge_build_frame() 序列化成 I2C 帧。
typedef struct bridge_snapshot_t {
  uint16_t seq;
  uint32_t uptime_ms;
  int16_t send_deg_x10[BRIDGE_CHANNEL_COUNT];
} bridge_snapshot_t;

// 构建固定长度 I2C 帧（48 字节）。
//
// - snapshot: 输入快照
// - out_frame/out_capacity: 输出 buffer（容量必须 >= BRIDGE_FRAME_SIZE）
// - out_len: 输出实际写入长度（成功时为 BRIDGE_FRAME_SIZE）
esp_err_t bridge_build_frame(const bridge_snapshot_t* snapshot,
                            uint8_t* out_frame,
                            size_t out_capacity,
                            size_t* out_len);

#ifdef __cplusplus
} // extern "C"
#endif
