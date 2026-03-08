# RasPi Bridge

该目录提供 Raspberry Pi OS 端桥接程序：

- 从 ESP32(I2C 从机)读取 ADC 采样帧
- 做滤波/死区/增量计算
- 通过 WebSocket 转发为 `servo_control`
- 默认启用较强抖动抑制（死区 + EMA + 连续确认帧）

## 1. 安装依赖

```bash
cd raspi
uv sync --frozen
```

## 2. 运行

```bash
cp config.example.json config.json
uv run bridge.py --config config.json
```

可选覆盖参数：

```bash
uv run bridge.py --config config.json --ws-uri ws://192.168.0.100:9102/ --client-name rpi-body
```

## 3. I2C 协议

- ESP32 从机地址: `0x28`
- `config.json` 的 `i2c_addr` 推荐写 `40`（即 `0x28`），不要写 `28`
- 主机请求命令: `0xA5`
- 响应帧长度: `60 bytes`
- 协议版本: `2`
- 数据内容: `21~43` 共 `23` 路发送角, `int16` 小端, 单位 `0.1 deg`，范围 `-90.0~90.0`

## 4. ADC 到命令映射

- ADC 到发送角 (`-90~90`) 的映射已在 ESP32 侧完成并通过 I2C 直接上报。
- RasPi 侧仅做滤波/死区，再将发送角换算为舵机命令 `p`：
  - `21~34`: `0~240deg -> p=0~1000`
  - `35~43`: `0~270deg -> p=500~2500`

## 5. WebSocket 发送格式

- `type=servo_control`
- `content` 为 JSON 字符串，数组元素结构与协议一致：

```json
{
  "character_name": "jiyuan",
  "web_servo": {
    "is_bus_servo": true,
    "servo_id": 21,
    "position": 173,
    "speed": 100
  }
}
```

## 6. 常见问题

1. `PermissionError: /dev/i2c-1`
   - 将用户加入 `i2c` 组，或使用 root 运行。
2. `I2C帧CRC错误` / `I2C帧magic错误`
   - 检查线序和地线，适当提高 `i2c_request_gap_ms`（如 `5 -> 10`）。
3. `等待绑定`
   - 确认 WebSocket 服务端在线列表中存在非本机客户端。
4. 抖动明显
   - 增大 `filter_deadzone_deg`、`filter_confirm_frames`，并提高 `send_min_delta_deg`。
