# RasPi Bridge

提供 Raspberry Pi OS 端桥接：

- 从 ESP32(I2C 从机)读取 ADC 采样帧
- 做滤波/死区/增量计算
- 通过 WebSocket 转发为 `servo_control`
- 默认启用较强抖动抑制（死区 + EMA + 连续确认帧）

## 1. 本地运行

```bash
cd raspi
uv sync --frozen
cp config.example.json config.json
uv run bridge.py --config config.json
```

可选覆盖参数：

```bash
uv run bridge.py --config config.json --ws-uri ws://192.168.0.100:9102/ --client-name rpi-body
```

## 2. Docker Compose 运行

### 2.1 准备配置

```bash
cd raspi
cp config.example.json config.json
```

### 2.2 前台验证启动

```bash
docker compose up --build
```

### 2.3 后台运行

```bash
docker compose up -d --build
```

说明：

- 容器使用 `host network`，WebSocket 连接直接复用树莓派网络栈。
- 通过 `devices: /dev/i2c-1` 映射 I2C 设备给容器。
- 配置 `restart: unless-stopped`


## 3. I2C 协议

需将 `${HOME}` 的用户加入i2c组

或使用root用户拉起

```bash
sudo usermod -aG i2c "$USER"
sudo reboot
```


- ESP32 从机地址: `0x28`
- `config.json` 的 `i2c_addr` 推荐写 `40`（即 `0x28`），不要写 `28`
- 主机请求命令: `0xA5`
- 响应帧长度: `60 bytes`
- 协议版本: `2`
- 数据内容: `21~43` 共 `23` 路发送角, `int16` 小端, 单位 `0.1 deg`，范围 `-90.0~90.0`


## 4. WebSocket 发送格式

- `type=servo_control`
- `content` 为 JSON 字符串，数组元素结构与协议一致：

```json
{
  "character_name": "jiyuan",
  "web_servo": {
    "is_bus_servo": true,
    "servo_id": 21,
    "position": -90,
    "speed": 100
  }
}
```

