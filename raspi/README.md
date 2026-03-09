# RasPi Bridge

该目录提供 Raspberry Pi OS 端桥接程序：

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
- 已配置 `restart: unless-stopped`，宿主机重启后 Docker daemon 会自动恢复容器。

## 3. 开机自启动

这个场景不需要额外包一层 `systemd`。  
只要宿主机的 Docker daemon 本身开机启动，且容器已经创建过一次，后续开机就会自动恢复。

```bash
cd raspi
sudo systemctl enable docker
docker compose up -d --build
```

说明：

- 第一次执行 `docker compose up -d --build` 会创建并启动容器。
- 之后树莓派重启时，Docker 会根据 `restart: unless-stopped` 自动拉起它。
- 如果你执行过 `docker compose down`，容器会被删除，之后需要再次 `docker compose up -d`。

常用命令：

```bash
sudo systemctl status docker --no-pager
docker compose logs -f
docker compose ps
docker compose restart
docker compose stop
docker compose up -d
```

如果修改了 `bridge.py` 或 `Dockerfile`，请重新构建并启动：

```bash
cd raspi
docker compose up -d --build
```

## 4. I2C 协议

- ESP32 从机地址: `0x28`
- `config.json` 的 `i2c_addr` 推荐写 `40`（即 `0x28`），不要写 `28`
- 主机请求命令: `0xA5`
- 响应帧长度: `60 bytes`
- 协议版本: `2`
- 数据内容: `21~43` 共 `23` 路发送角, `int16` 小端, 单位 `0.1 deg`，范围 `-90.0~90.0`

## 5. ADC 到命令映射

- ADC 到发送角 (`-90~90`) 的映射已在 ESP32 侧完成并通过 I2C 直接上报。
- RasPi 侧仅做滤波/死区，`web_servo.position` 直接发送 `-90~90` 角度值。

## 6. WebSocket 发送格式

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

## 7. 常见问题

1. `PermissionError: /dev/i2c-1`
   - 本地运行时将用户加入 `i2c` 组，或使用 root 运行。
   - Docker 运行时确认 `compose.yaml` 已映射 `/dev/i2c-1`，并且宿主机已开启 I2C。
2. `I2C帧CRC错误` / `I2C帧magic错误`
   - 检查线序和地线，适当提高 `i2c_request_gap_ms`（如 `5 -> 10`）。
3. `等待绑定`
   - 确认 WebSocket 服务端在线列表中存在非本机客户端。
4. 抖动明显
   - 增大 `filter_deadzone_deg`、`filter_confirm_frames`，并提高 `send_min_delta_deg`。
5. 开机后服务未拉起
   - 执行 `sudo systemctl status docker --no-pager` 确认 Docker daemon 已正常启动。
   - 执行 `docker compose ps` 确认容器是否已存在。
   - 执行 `docker compose logs -f` 查看容器日志。
