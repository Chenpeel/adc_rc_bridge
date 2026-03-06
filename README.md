# ADC_Remote_Control 

本文档覆盖 Linux / Windows 双平台的环境配置、编译、烧录和排障流程。

## 1. 工程概览

目录：

```text
.
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   ├── main.cpp
│   ├── gain.h
│   ├── send.h
│   └── idf_component.yml
├── sdkconfig.defaults
└── dependencies.lock
```

关键点：

- 使用 `main/idf_component.yml` 拉取第三方组件。
- `dependencies.lock` 锁定依赖版本。默认目标是 `esp32`。

## 2. 版本建议

- 建议 ESP-IDF 版本：`v6.1.x`
- 已使用组件管理器（Component Manager）依赖方式

如果你已有其他版本 ESP-IDF，建议先用 `idf.py --version` 确认版本，再决定是否切换。

## 3. Linux 环境配置

### 3.1 安装系统依赖（Ubuntu/Debian）

```bash
sudo apt update
sudo apt install -y git wget flex bison gperf python3 python3-pip python3-venv \
  cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

### 3.2 安装 ESP-IDF

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v6.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
```

### 3.3 加载环境变量

每次新开终端都需要执行：

```bash
source ~/esp/esp-idf/export.sh
```

建议写入环境变量

```bash
echo "source ~/esp/esp-idf/export.sh" >> ~/.bashrc
```



### 3.4 校验环境

```bash
idf.py --version
echo "$IDF_PATH"
test -d "$IDF_PATH" && echo "IDF_PATH OK" || echo "IDF_PATH INVALID"
```

如果 `IDF_PATH` 指向了不存在目录（例如历史残留路径），请重新 `source` 正确的 `export.sh`。

## 4. Windows 环境配置

推荐两种方式，二选一即可。

### 4.1 方式 A：官方安装器

1. 安装 Espressif 官方 Windows Installer（ESP-IDF Tools Installer）。
2. 安装完成后，使用开始菜单里的 `ESP-IDF PowerShell` 或 `ESP-IDF CMD` 打开终端。
3. 终端内执行：

```bat
idf.py --version
```

### 4.2 方式 B：手动安装

```bat
mkdir C:\esp
cd /d C:\esp
git clone -b v6.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
install.bat esp32
export.bat
idf.py --version
```

## 5. 获取项目并准备依赖

```bash
git clone https://github.com/Chenpeel/adc_rc_bridge.git adc_rc_bridge
cd adc_rc_bridge
```

首次构建时，ESP-IDF 会根据 `main/idf_component.yml` 自动下载组件。  
需要确保网络可访问 `components.espressif.com`。

可选：显式指定组件缓存目录（在受限环境里很有用）：

- Linux/macOS:

```bash
export IDF_COMPONENT_CACHE_PATH="$PWD/.cm_cache"
```

- Windows CMD:

```bat
set IDF_COMPONENT_CACHE_PATH=%CD%\.cm_cache
```

- Windows PowerShell:

```powershell
$env:IDF_COMPONENT_CACHE_PATH="$PWD\.cm_cache"
```

## 6. 编译（Linux / Windows 通用）

在工程根目录执行：

```bash
idf.py set-target esp32
idf.py build
```

编译产物：

- `build/remote_control.bin`
- `build/bootloader/bootloader.bin`
- `build/partition_table/partition-table.bin`

## 7. 烧录与串口监控

### 7.1 查找串口

- Linux:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

- Windows（PowerShell）:

```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID,Description
```

### 7.2 一键烧录并监控

- Linux 示例：

```bash
idf.py -p /dev/ttyUSB0 -b 460800 flash monitor
```

- Windows 示例：

```bat
idf.py -p COM3 -b 460800 flash monitor
```

退出串口监控：`Ctrl + ]`

### 7.3 仅擦除再烧录（可选）

```bash
idf.py -p <PORT> erase-flash flash monitor
```

## 8. 常用命令速查

```bash
# 清理中间产物（保留配置）
idf.py clean

# 全量清理（清掉 CMake 缓存等）
idf.py fullclean

# 重新配置
idf.py reconfigure

# 只编译 app
idf.py app
```

## 9. 项目运行时参数

可在 `main/main.cpp` 中修改：

- 默认 WiFi：
  - `DEFAULT_SSID = "gaoda"`
  - `DEFAULT_PASS = "gaoda123"`
- 默认设备名：`DEFAULT_NAME = "body"`
- 目标设备名：`TARGET_NAME = "esp32"`
- WebSocket 地址：`ws://192.168.0.100:9102/`
- 发送周期：`SEND_INTERVAL_MS = 50`（20Hz）

WiFi 连接失败时，设备会进入 AP 配网模式：

- AP 名称：`ESP_32_AP`
- AP 密码：`00000000`
- 配网页面：`http://<AP_IP>/`

## 10. 常见问题排查

### 10.1 `Failed to resolve component 'esp_websocket_client'` / `cjson`

检查项：

1. 是否在正确 ESP-IDF 环境终端中执行（先 `idf.py --version`）。
2. 是否存在 `main/idf_component.yml`。
3. 网络是否可访问 `components.espressif.com`。
4. 重新执行：

```bash
idf.py reconfigure
idf.py build
```

### 10.2 `IDF_PATH` 指向错误目录

现象：构建找不到 `project.cmake` 或 IDF 脚本路径错误。

处理：

1. 重新加载正确安装目录下的 `export.sh` / `export.bat`。
2. 再次检查：

```bash
echo "$IDF_PATH"
```

### 10.3 Linux 串口权限不足（`Permission denied`）

```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

或临时使用 `sudo`。

### 10.4 串口找不到设备

1. 检查 USB 线是否支持数据传输（非纯充电线）。
2. 检查驱动（CH340 / CP210x / FTDI）。
3. 更换 USB 口、线材后重试。
