# BMP280气压记录系统

这是一个基于ESP32C3的气压记录系统，使用BMP280传感器采集温度、气压和海拔数据，并将数据上传到中国移动OneNET物联网平台。

## 硬件要求

- ESP32C3开发板 (合宙Air32C3)
- BMP280气压传感器模块
- 杜邦线若干

## 接线说明

BMP280传感器与ESP32C3的连接：

| BMP280 | ESP32C3 |
|--------|---------|
| VCC    | 3.3V    |
| GND    | GND     |
| SCL    | GPIO5   |
| SDA    | GPIO4   |

## 软件配置

### 1. 配置密钥文件

编辑 `include/secrets.h` 文件，填入您的实际配置信息：

```cpp
// WiFi配置
#define WIFI_SSID "your_wifi_ssid"              // 替换为您的WiFi名称
#define WIFI_PASSWORD "your_wifi_password"       // 替换为您的WiFi密码

// OneNET MQTT配置
#define ONENET_PRODUCT_ID "your_product_id"      // OneNET产品ID
#define ONENET_DEVICE_NAME "your_device_name"    // OneNET设备名称
#define ONENET_ACCESS_KEY "your_access_key"      // OneNET访问密钥

// MQTT服务器配置（智能域名）
#define MQTT_SERVER "your_product_id.mqtts.acc.cmcconenet.cn"  // 智能域名
#define MQTT_PORT 1883                           // MQTT端口
```

### 2. OneNET平台配置

1. 访问 [中国移动OneNET平台](https://open.iot.10086.cn/)
2. 注册账号并登录
3. 创建产品（选择MQTT协议接入）
4. 创建设备并获取设备信息
5. 在产品的物模型中定义以下属性：
   - `temperature` - 温度 (float, °C)
   - `pressure` - 气压 (float, hPa)
   - `altitude` - 海拔 (float, m)
6. 获取产品ID、设备名称和访问密钥，填入secrets.h文件
7. 获取智能域名（格式：`产品ID.mqtts.acc.cmcconenet.cn`）

## 功能特性

- **实时数据采集**: 每30秒采集一次BMP280传感器数据
- **WiFi连接**: 自动连接WiFi网络，支持断线重连
- **MQTT通信**: 使用MQTT协议与OneNET平台通信
- **Token认证**: 自动生成HMAC-SHA1签名进行身份认证
- **物模型支持**: 支持OneNET物模型属性上报
- **串口监控**: 通过串口输出详细的运行日志
- **错误处理**: 包含WiFi连接失败、MQTT连接失败、传感器初始化失败等错误处理

## 编译和上传

1. 安装PlatformIO IDE或VSCode + PlatformIO插件
2. 打开项目文件夹
3. 配置secrets.h文件
4. 连接ESP32C3开发板
5. 点击上传按钮或运行命令：
   ```
   pio run --target upload
   ```

## 串口监控

上传完成后，可以通过串口监控查看运行状态：

```
pio device monitor
```

## 数据格式

发布到OneNET的数据格式为OneJSON物模型格式：

```json
{
  "id": "123456789",
  "version": "1.0",
  "params": {
    "temperature": {
      "value": 25.6,
      "time": 1234567890
    },
    "pressure": {
      "value": 1013.25,
      "time": 1234567890
    },
    "altitude": {
      "value": 123.45,
      "time": 1234567890
    }
  }
}
```

**MQTT主题**：
- 属性上报：`$sys/{product_id}/{device_name}/thing/property/post`
- 属性设置：`$sys/{product_id}/{device_name}/thing/property/set`

## 故障排除

### 传感器初始化失败
- 检查BMP280模块接线是否正确
- 确认传感器I2C地址（默认0x76，部分模块为0x77）
- 检查电源供电是否稳定

### WiFi连接失败
- 确认WiFi名称和密码是否正确
- 检查WiFi信号强度
- 确认WiFi网络支持2.4GHz频段

### MQTT连接失败

- 检查OneNET产品ID、设备名称和访问密钥是否正确
- 确认智能域名格式正确（`产品ID.mqtts.acc.cmcconenet.cn`）
- 检查Token生成算法是否正确
- 确认设备在OneNET平台中状态正常
- 检查网络连接是否稳定

### 数据发布失败

- 确认MQTT连接状态正常
- 检查物模型属性定义是否与代码中的属性名称一致
- 确认OneJSON数据格式正确

## 许可证

MIT License
