# VOC传感器集成指南

## 概述

本项目现在支持两种不同的VOC传感器：
1. **21VOC五合一传感器** - 检测VOC、甲醛、eCO2、温度、湿度
2. **VOC-CO2-HCHO三合一传感器** - 检测TVOC、甲醛、CO₂

两种传感器可以同时启用，也可以单独使用。

## 传感器配置

### 编译时配置

在 `platformio.ini` 中通过 `build_flags` 控制传感器启用：

```ini
build_flags = 
    -DENABLE_BMP280=1           # BMP280气压温度传感器
    -DENABLE_MQ135=1            # MQ135空气质量传感器
    -DENABLE_21VOC=1            # 21VOC五合一传感器
    -DENABLE_VOC_CO2_HCHO=1     # VOC-CO2-HCHO三合一传感器
    -DENABLE_HW181_MIC=1        # HW181-MIC分贝检测模块
```

### 硬件连接

#### 21VOC五合一传感器
```
21VOC传感器 -> ESP32C3
VCC        -> 3.3V (或5V)
GND        -> GND
TX         -> GPIO 1 (UART1 RX)
RX         -> GPIO 0 (UART1 TX)
```

#### VOC-CO2-HCHO三合一传感器
```
VOC-CO2-HCHO传感器 -> ESP32C3
VCC               -> 3.3V (或5V)
GND               -> GND
TX                -> GPIO 1 (UART2 RX)
RX                -> GPIO 0 (UART2 TX)
```

**注意**: 两种传感器使用不同的UART端口，可以同时连接。

## 数据格式

### 21VOC五合一传感器数据
- **VOC空气质量**: μg/m³ (整数)
- **甲醛浓度**: μg/m³ (整数)
- **eCO2浓度**: ppm (整数)
- **温度**: °C (浮点数)
- **湿度**: %RH (浮点数)

### VOC-CO2-HCHO三合一传感器数据
- **TVOC浓度**: mg/m³ (浮点数，精度0.001)
- **甲醛浓度**: mg/m³ (浮点数，精度0.001)
- **CO₂浓度**: mg/m³ (浮点数，精度0.001)

## OneNET数据上传

### 21VOC传感器字段
```json
{
  "params": {
    "voc_ugm3": {"value": 50, "time": 1640995200000},
    "ch2o_ugm3": {"value": 30, "time": 1640995200000},
    "eco2_ppm": {"value": 400, "time": 1640995200000},
    "temperature": {"value": 25.5, "time": 1640995200000},
    "humidity_rh": {"value": 60.0, "time": 1640995200000}
  }
}
```

### VOC-CO2-HCHO传感器字段
```json
{
  "params": {
    "voc": {"value": 0.300, "time": 1640995200000},
    "ch2o": {"value": 0.030, "time": 1640995200000},
    "co2": {"value": 0.180, "time": 1640995200000}
  }
}
```

## 串口命令

### 21VOC传感器命令
- `show_tvoc` 或 `tvoc` - 显示21VOC传感器数据
- `debug_tvoc` 或 `debug` - 调试21VOC传感器
- `test_tvoc` 或 `test` - 测试21VOC传感器连接

### VOC-CO2-HCHO传感器命令
- `show_voc` 或 `voc` - 显示VOC-CO2-HCHO传感器数据
- `debug_voc` 或 `dvoc` - 调试VOC-CO2-HCHO传感器
- `test_voc` 或 `tvoc_test` - 测试VOC-CO2-HCHO传感器连接

### 通用命令
- `help` - 显示所有可用命令
- `wifi_status` - 显示WiFi状态
- `mqtt_status` - 显示MQTT状态

## 使用示例

### 1. 编译和上传
```bash
# 编译项目
pio run

# 上传到ESP32C3
pio run --target upload

# 监控串口输出
pio run --target monitor
```

### 2. 运行时监控
```
=== 传感器数据 ===
--- 21VOC五合一传感器数据 ---
VOC空气质量: 50 μg/m³
甲醛浓度: 30 μg/m³
eCO2浓度: 400 ppm
温度: 25.5°C
湿度: 60.0%RH

--- VOC-CO2-HCHO三合一传感器数据 ---
TVOC浓度: 0.300 mg/m³
甲醛浓度: 0.030 mg/m³
CO₂浓度: 0.180 mg/m³
==================
```

### 3. 单独测试传感器
```bash
# 测试21VOC传感器
show_tvoc

# 测试VOC-CO2-HCHO传感器
show_voc

# 调试连接问题
debug_voc
test_voc
```

## 独立测试程序

### VOC-CO2-HCHO传感器独立测试
位置: `test/test_VOC-CO2-HCHO-Sensor.cpp`

使用专用配置编译：
```bash
pio run -c platformio_21voc.ini --target upload
pio run -c platformio_21voc.ini --target monitor
```

## 故障排除

### 1. 传感器无数据
- 检查硬件连接
- 确认传感器供电正常
- 使用debug命令检查UART状态
- 检查波特率设置 (9600)

### 2. 数据解析失败
- 检查数据帧格式
- 确认协议匹配
- 查看原始数据输出

### 3. OneNET上传失败
- 检查WiFi连接
- 确认MQTT连接状态
- 验证物模型配置

## 技术参数

### 通信参数
- **波特率**: 9600 bps
- **数据格式**: 8N1
- **21VOC数据帧**: 12字节
- **VOC-CO2-HCHO数据帧**: 9字节

### 内存使用
- **RAM**: 12.3% (40,228 bytes)
- **Flash**: 66.3% (869,224 bytes)

## 扩展功能

可以在此基础上添加：
- 更多传感器类型
- 本地数据存储
- Web界面监控
- 报警和通知功能
- 数据分析和趋势图

## 注意事项

1. **传感器预热**: 两种传感器都需要预热2-3分钟
2. **数据单位**: 注意21VOC使用μg/m³，VOC-CO2-HCHO使用mg/m³
3. **UART冲突**: 确保两种传感器使用不同的UART端口
4. **电源要求**: 确保电源供电稳定，支持多传感器同时工作

## 技术支持

如遇到问题，请检查：
1. 硬件连接是否正确
2. 传感器供电是否稳定
3. UART通信参数是否匹配
4. 传感器是否充分预热
5. OneNET物模型配置是否正确
