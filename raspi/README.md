# RasPi Bridge

该目录提供 Raspberry Pi OS 端桥接程序：

- 从 ESP32(I2C 从机)读取 ADC 采样帧
- 做滤波/死区/增量计算
- 通过 WebSocket 转发为 `servo_control`

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
- 数据内容: `21~43` 共 `23` 路角度, 单位 `0.1 deg`

## 4. 常见问题

1. `PermissionError: /dev/i2c-1`
   - 将用户加入 `i2c` 组，或使用 root 运行。
2. `I2C帧CRC错误`
   - 检查线序和地线，适当提高 `i2c_request_gap_ms`。
3. `等待绑定`
   - 确认 WebSocket 服务端在线列表中存在非本机客户端。
