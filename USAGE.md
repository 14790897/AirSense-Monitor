# BMP280气压记录系统使用指南

## 快速开始

### 1. 硬件准备
- ESP32C3开发板（合宙Air32C3）
- BMP280气压传感器模块
- 杜邦线4根

### 2. 接线连接
```
BMP280    ESP32C3
VCC   ->  3.3V
GND   ->  GND
SCL   ->  GPIO5 (I2C时钟线)
SDA   ->  GPIO4 (I2C数据线)
```

### 3. 配置OneNET平台

#### 3.1 创建产品
1. 登录 [OneNET平台](https://open.iot.10086.cn/)
2. 进入"产品开发" -> "创建产品"
3. 选择"MQTT协议"接入方式
4. 记录产品ID（如：11ijEEhVAe）

#### 3.2 创建设备
1. 在产品中创建设备
2. 设备名称：pressure_sensor_01（可自定义）
3. 记录访问密钥（Access Key）

#### 3.3 定义物模型
在产品的"物模型定义"中添加以下属性：

| 属性标识符 | 数据类型 | 单位 | 描述 |
|-----------|---------|------|------|
| temperature | float | °C | 温度 |
| pressure | float | hPa | 气压 |
| altitude | float | m | 海拔 |

#### 3.4 获取智能域名
在产品详情页面获取智能接入域名：
格式：`{产品ID}.mqtts.acc.cmcconenet.cn`

### 4. 配置代码

编辑 `include/secrets.h` 文件：

```cpp
// WiFi配置
#define WIFI_SSID "你的WiFi名称"
#define WIFI_PASSWORD "你的WiFi密码"

// OneNET MQTT配置
#define ONENET_PRODUCT_ID "你的产品ID"
#define ONENET_DEVICE_NAME "你的设备名称"
#define ONENET_ACCESS_KEY "你的访问密钥"

// MQTT服务器配置（智能域名）
#define MQTT_SERVER "你的产品ID.mqtts.acc.cmcconenet.cn"
#define MQTT_PORT 1883
```

### 5. 编译和上传

```bash
# 编译
pio run

# 上传到设备
pio run --target upload

# 监控串口输出
pio device monitor
```

## 运行效果

### 串口输出示例
```
BMP280 气压记录系统启动...
BMP280传感器初始化成功！
连接WiFi...
WiFi连接成功！
IP地址: 192.168.1.100
尝试连接MQTT服务器...
MQTT连接成功！
订阅主题: $sys/11ijEEhVAe/pressure_sensor_01/thing/property/set
系统初始化完成，开始数据采集...

=== 传感器数据 ===
温度: 25.60 °C
气压: 1013.25 hPa
海拔: 123.45 m
==================
发布数据到OneNET...
主题: $sys/11ijEEhVAe/pressure_sensor_01/thing/property/post
数据: {"id":"123456","version":"1.0","params":{"temperature":{"value":25.6,"time":123456},"pressure":{"value":1013.25,"time":123456},"altitude":{"value":123.45,"time":123456}}}
数据发布成功！
```

### OneNET平台查看数据
1. 登录OneNET平台
2. 进入设备详情页面
3. 查看"设备数据"或"数据流"
4. 可以看到实时的温度、气压、海拔数据

## 故障排除

### 常见问题

**1. 传感器初始化失败**
- 检查接线是否正确
- 确认BMP280模块工作正常
- 尝试更换I2C地址（0x76或0x77）

**2. WiFi连接失败**
- 检查WiFi名称和密码
- 确认WiFi信号强度
- 确认使用2.4GHz频段

**3. MQTT连接失败**
- 检查OneNET配置信息
- 确认智能域名格式正确
- 检查Token生成算法

**4. 数据发布失败**
- 确认MQTT连接正常
- 检查物模型属性定义
- 确认OneJSON格式正确

### 调试技巧
1. 使用串口监控查看详细日志
2. 检查OneNET平台设备状态
3. 验证MQTT主题格式
4. 确认数据格式符合OneJSON规范

## 自定义配置

### 修改数据上传间隔
在 `main.cpp` 中修改：
```cpp
const unsigned long UPLOAD_INTERVAL = 60000; // 改为60秒
```

### 添加新的传感器属性
1. 在OneNET平台物模型中添加新属性
2. 在代码中添加数据读取逻辑
3. 在 `publishSensorData()` 函数中添加新属性的数据

### 启用TLS加密
修改 `platformio.ini`：
```ini
build_flags = 
    -DCONFIG_NETWORK_TLS=1
    -DIOT_MQTT_SERVER_ADDR_TLS=\"11ijEEhVAe.mqttstls.acc.cmcconenet.cn\"
    -DIOT_MQTT_SERVER_PORT_TLS=8883
```

## 技术支持

如遇到问题，请检查：
1. 硬件连接是否正确
2. OneNET平台配置是否完整
3. 代码配置是否正确
4. 网络连接是否正常

更多技术文档请参考：
- [OneNET开发文档](https://open.iot.10086.cn/doc/)
- [BMP280传感器手册](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp280/)
- [ESP32C3开发指南](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/)
