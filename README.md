# ADC_Remote_Control 

本文档覆盖 `rpi-esp` 架构下的环境配置、编译、烧录与联调流程。

> [!WARNING]
> 由于 `ESP32` 的 Wi-Fi 模块稳定性较差，本项目已转用
> [Raspi + ESP32](https://github.com/Chenpeel/adc_rc_bridge/tree/rpi-esp)
> 模式。
> 请优先参考该方案进行部署、烧录和调试，避免继续沿用当前纯
> `ESP32` Wi-Fi 方案。

## 1. 工程概览

目录：

```text
.
├── CMakeLists.txt
├── idf_component.yml
├── main/
│   ├── CMakeLists.txt
│   ├── adc_mux_sampler.cpp
│   ├── adc_mux_sampler.h
│   ├── bridge_protocol.cpp
│   ├── bridge_protocol.h
│   ├── idf_component.yml
│   └── main.cpp
├── raspi/
│   ├── README.md
│   ├── bridge.py
│   ├── compose.yaml
│   ├── config.example.json
│   ├── Dockerfile
│   ├── pyproject.toml
│   ├── test_bridge_reverse.py
│   └── uv.lock
├── sdkconfig.defaults
└── dependencies.lock
```

关键点：

- ESP32 负责 `ADC + I2C 从机`。
- Raspberry Pi OS 负责 `I2C 主机读取 + WebSocket 通信`，参考[RasPi Bridge](./raspi/README.md)
- 默认目标芯片为 `esp32`。

## 2. Linux 环境配置

### 2.1 安装系统依赖（Ubuntu/Debian）

```bash
sudo apt update
sudo apt install -y git wget flex bison gperf python3 python3-pip python3-venv \
  cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

### 2.2 安装 ESP-IDF

```bash
mkdir -p ~/esp
cd ~/esp
git clone -b v6.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
```

### 2.3 加载环境变量

每次新开终端都需要执行：

```bash
source ~/esp/esp-idf/export.sh
```

建议写入环境变量

```bash
echo "source ~/esp/esp-idf/export.sh" >> ~/.bashrc
```



### 2.4 校验环境

```bash
idf.py --version
echo "$IDF_PATH"
test -d "$IDF_PATH" && echo "IDF_PATH OK" || echo "IDF_PATH INVALID"
```

如果 `IDF_PATH` 指向了不存在目录（例如历史残留路径），请重新 `source` 正确的 `export.sh`。

## 3. Windows 环境配置

推荐两种方式，二选一即可。

### 3.1 方式 A：官方安装器

1. 安装 Espressif 官方 Windows Installer（ESP-IDF Tools Installer）。
2. 安装完成后，使用开始菜单里的 `ESP-IDF PowerShell` 或 `ESP-IDF CMD` 打开终端。
3. 终端内执行：

```bat
idf.py --version
```

### 3.2 方式 B：手动安装

```bat
mkdir C:\esp
cd /d C:\esp
git clone -b v6.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
install.bat esp32
export.bat
idf.py --version
```

## 4. 获取项目并准备依赖

```bash
git clone https://github.com/Chenpeel/adc_rc_bridge.git adc_rc_bridge
cd adc_rc_bridge
```

本分支不依赖额外三方 ESP-IDF 组件，离线构建更稳定。

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

## 5. 编译（Linux / Windows 通用）

在工程根目录执行：

```bash
idf.py set-target esp32
idf.py build
```

编译产物：

- `build/remote_control.bin`
- `build/bootloader/bootloader.bin`
- `build/partition_table/partition-table.bin`

### 5.1 查找串口

- Linux:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

- Windows（PowerShell）:

```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID,Description
```

### 5.2 一键烧录并监控

- Linux 示例：

```bash
idf.py -p /dev/ttyUSB0 -b 460800 flash monitor
```

- Windows 示例：

```bat
idf.py -p COM3 -b 460800 flash monitor
```

退出串口监控：`Ctrl + ]`
