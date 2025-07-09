#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>
#include <time.h>
#include "secrets.h"
#include "onenet_token.h"

// BMP280传感器对象
Adafruit_BMP280 bmp;

// WiFi和MQTT客户端
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// 数据上传间隔 (毫秒)
const unsigned long UPLOAD_INTERVAL = 30000; // 30秒
unsigned long lastUploadTime = 0;

// MQTT主题
String propertyPostTopic;
String propertySetTopic;
String propertyPostReplyTopic;

// 函数声明
void connectWiFi();
void connectMQTT();
void readSensorData();
void publishSensorData(float temperature, float pressure, float altitude);
void mqttCallback(char* topic, byte* payload, unsigned int length);
uint32_t getCurrentTimestamp();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("BMP280 气压记录系统启动...");

  // 初始化I2C
  Wire.begin();

  // 初始化BMP280传感器
  if (!bmp.begin(0x76)) { // BMP280默认I2C地址是0x76，有些模块是0x77
    Serial.println("无法找到BMP280传感器，请检查接线！");
    while (1) delay(10);
  }

  // 配置BMP280传感器
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // 工作模式
                  Adafruit_BMP280::SAMPLING_X2,     // 温度过采样
                  Adafruit_BMP280::SAMPLING_X16,    // 气压过采样
                  Adafruit_BMP280::FILTER_X16,      // 滤波
                  Adafruit_BMP280::STANDBY_MS_500); // 待机时间

  Serial.println("BMP280传感器初始化成功！");

  // 连接WiFi
  connectWiFi();

  // 配置NTP时间同步
  Serial.println("正在同步时间...");
  configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  // 等待时间同步完成
  time_t now = time(nullptr);
  int timeout = 0;
  while (now < 8 * 3600 * 2 && timeout < 30)
  {
    delay(1000);
    now = time(nullptr);
    timeout++;
    Serial.print(".");
  }
  Serial.println();

  if (now < 8 * 3600 * 2)
  {
    Serial.println("⚠️ 时间同步失败，将使用相对时间");
  }
  else
  {
    Serial.println("✅ 时间同步成功");
    Serial.println("当前时间: " + String(ctime(&now)));
  }

  // 配置MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // 设置MQTT主题
  propertyPostTopic = "$sys/" + String(ONENET_PRODUCT_ID) + "/" + String(ONENET_DEVICE_NAME) + "/thing/property/post";
  propertySetTopic = "$sys/" + String(ONENET_PRODUCT_ID) + "/" + String(ONENET_DEVICE_NAME) + "/thing/property/set";
  propertyPostReplyTopic = "$sys/" + String(ONENET_PRODUCT_ID) + "/" + String(ONENET_DEVICE_NAME) + "/thing/property/post/reply";

  // 连接MQTT
  connectMQTT();

  Serial.println("系统初始化完成，开始数据采集...");
}

void loop() {
  // 检查WiFi连接状态
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi连接断开，尝试重新连接...");
    Serial.println("当前WiFi状态: " + String(WiFi.status()));
    connectWiFi();
  }

  // 检查MQTT连接状态
  if (!mqttClient.connected()) {
    Serial.println("⚠️ MQTT连接断开，尝试重新连接...");
    Serial.println("当前MQTT状态: " + String(mqttClient.state()));
    connectMQTT();
  }

  // 处理MQTT消息
  mqttClient.loop();

  // 定时上传数据
  if (millis() - lastUploadTime >= UPLOAD_INTERVAL) {
    Serial.println("⏰ 到达上传时间间隔，开始读取传感器数据...");
    readSensorData();
    lastUploadTime = millis();
    Serial.println("下次上传时间: " + String((lastUploadTime + UPLOAD_INTERVAL) / 1000) + "秒后");
  }

  delay(1000); // 1秒延时
}

void connectWiFi() {
  Serial.println("=== 开始WiFi连接 ===");
  Serial.println("SSID: " + String(WIFI_SSID));
  Serial.println("密码长度: " + String(strlen(WIFI_PASSWORD)));

  // 可选：设置静态IP（如果不需要可以注释掉）
  // IPAddress local_IP(192, 168, 1, 100);    // 你想要的IP地址
  // IPAddress gateway(192, 168, 1, 1);       // 路由器IP
  // IPAddress subnet(255, 255, 255, 0);      // 子网掩码
  // IPAddress primaryDNS(8, 8, 8, 8);        // 主DNS
  // IPAddress secondaryDNS(8, 8, 4, 4);      // 备用DNS
  // if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
  //   Serial.println("静态IP配置失败");
  // }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("连接WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;

    // 每5次尝试打印一次状态
    if (attempts % 5 == 0)
    {
      Serial.println();
      Serial.print("尝试次数: " + String(attempts) + "/20, WiFi状态: ");
      switch (WiFi.status())
      {
      case WL_IDLE_STATUS:
        Serial.println("WL_IDLE_STATUS");
        break;
      case WL_NO_SSID_AVAIL:
        Serial.println("WL_NO_SSID_AVAIL - 找不到SSID");
        break;
      case WL_SCAN_COMPLETED:
        Serial.println("WL_SCAN_COMPLETED");
        break;
      case WL_CONNECTED:
        Serial.println("WL_CONNECTED");
        break;
      case WL_CONNECT_FAILED:
        Serial.println("WL_CONNECT_FAILED - 连接失败");
        break;
      case WL_CONNECTION_LOST:
        Serial.println("WL_CONNECTION_LOST - 连接丢失");
        break;
      case WL_DISCONNECTED:
        Serial.println("WL_DISCONNECTED - 未连接");
        break;
      default:
        Serial.println("未知状态: " + String(WiFi.status()));
      }
      Serial.print("继续连接");
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("🎉 WiFi连接成功！");
    Serial.print("IP地址: ");
    Serial.println(WiFi.localIP());
    Serial.print("网关: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("DNS: ");
    Serial.println(WiFi.dnsIP());
    Serial.print("信号强度: ");
    Serial.println(WiFi.RSSI());
    Serial.println("=== WiFi连接完成 ===");
  } else {
    Serial.println();
    Serial.println("❌ WiFi连接失败！");
    Serial.println("最终状态: " + String(WiFi.status()));
  }
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.println("=== 开始MQTT连接流程 ===");
    Serial.println("MQTT服务器: " + String(MQTT_SERVER) + ":" + String(MQTT_PORT));

    // 使用固定的Token过期时间（2028年1月1日）
    uint32_t currentTime = getCurrentTimestamp();
    uint32_t expireTime = TOKEN_EXPIRE_TIME;

    // 打印配置信息
    Serial.println("--- OneNET配置信息 ---");
    Serial.println("认证模式: 设备级认证（一机一密）");
    Serial.println("产品ID: " + String(ONENET_PRODUCT_ID));
    Serial.println("设备名称: " + String(ONENET_DEVICE_NAME));
    Serial.println("设备密钥长度: " + String(strlen(ONENET_DEVICE_SECRET)));
    Serial.println("当前时间戳: " + String(currentTime));
    Serial.println("Token过期时间戳(et): " + String(expireTime));
    Serial.println("Token过期时间: 2028年1月1日 (固定时间)");
    Serial.println("预期res格式: products/" + String(ONENET_PRODUCT_ID) + "/devices/" + String(ONENET_DEVICE_NAME));

    // 使用官方Token生成函数 - 设备级认证（一机一密）
    char token[512] = {0};
    Serial.println("--- 开始生成Token ---");
    Serial.println("使用认证模式: 设备级认证（一机一密）");
    int result = onenet_token_generate(
        token,
        SIG_METHOD_SHA1, // 使用SHA1签名方法
        expireTime,
        ONENET_PRODUCT_ID,
        ONENET_DEVICE_NAME,    // 使用设备名，res格式为 products/{产品ID}/devices/{设备名}
        ONENET_DEVICE_SECRET); // 使用设备密钥

    Serial.println("Token生成结果码: " + String(result));
    if (result != 0)
    {
      Serial.println("❌ Token生成失败！错误码: " + String(result));
      delay(5000);
      continue;
    }

    Serial.println("完整Token: " + String(token));

    String password = String(token);

    String clientId = String(ONENET_DEVICE_NAME); // 使用简单方式

    Serial.println("--- MQTT连接参数 ---");
    Serial.println("ClientID: " + clientId);
    Serial.println("Username: " + String(ONENET_PRODUCT_ID));
    Serial.println("Password: " + password);

    Serial.println("--- 尝试MQTT连接 ---");
    bool connectResult = mqttClient.connect(clientId.c_str(), ONENET_PRODUCT_ID, password.c_str());

    Serial.println("MQTT连接函数返回: " + String(connectResult ? "true" : "false"));
    Serial.println("MQTT状态码: " + String(mqttClient.state()));

    if (connectResult && mqttClient.connected())
    {
      Serial.println("🎉 MQTT连接成功！");

      // 订阅属性设置主题
      bool subResult1 = mqttClient.subscribe(propertySetTopic.c_str());
      Serial.println("订阅属性设置主题结果: " + String(subResult1 ? "成功" : "失败"));
      Serial.println("订阅主题: " + propertySetTopic);

      // 订阅属性上报回执主题
      bool subResult2 = mqttClient.subscribe(propertyPostReplyTopic.c_str());
      Serial.println("订阅上报回执主题结果: " + String(subResult2 ? "成功" : "失败"));
      Serial.println("订阅主题: " + propertyPostReplyTopic);

      Serial.println("=== MQTT连接流程完成 ===");
    }
    else
    {
      Serial.println("❌ MQTT连接失败！");
      Serial.print("详细错误码: ");
      int state = mqttClient.state();
      Serial.println(state);

      // 详细的错误码说明
      switch (state)
      {
      case -4:
        Serial.println("错误: MQTT_CONNECTION_TIMEOUT - 服务器没有在keepalive时间内响应");
        break;
      case -3:
        Serial.println("错误: MQTT_CONNECTION_LOST - 网络连接中断");
        break;
      case -2:
        Serial.println("错误: MQTT_CONNECT_FAILED - 网络连接失败");
        break;
      case -1:
        Serial.println("错误: MQTT_DISCONNECTED - 客户端未连接");
        break;
      case 1:
        Serial.println("错误: MQTT_CONNECT_BAD_PROTOCOL - 服务器不支持请求的MQTT协议版本");
        break;
      case 2:
        Serial.println("错误: MQTT_CONNECT_BAD_CLIENT_ID - 服务器拒绝了客户端标识符");
        break;
      case 3:
        Serial.println("错误: MQTT_CONNECT_UNAVAILABLE - 服务器不可用");
        break;
      case 4:
        Serial.println("错误: MQTT_CONNECT_BAD_CREDENTIALS - 用户名或密码错误");
        break;
      case 5:
        Serial.println("错误: MQTT_CONNECT_UNAUTHORIZED - 客户端未授权连接");
        break;
      default:
        Serial.println("未知错误码");
      }

      Serial.println("等待5秒后重试...");
      delay(5000);
    }
  }
}

void readSensorData() {
  // 读取传感器数据
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F; // 转换为hPa
  float altitude = bmp.readAltitude(1013.25); // 海平面气压1013.25hPa

  // 打印传感器数据
  Serial.println("=== 传感器数据 ===");
  Serial.printf("温度: %.2f °C\n", temperature);
  Serial.printf("气压: %.2f hPa\n", pressure);
  Serial.printf("海拔: %.2f m\n", altitude);
  Serial.println("==================");

  // 发布到OneNET
  if (mqttClient.connected()) {
    publishSensorData(temperature, pressure, altitude);
  } else {
    Serial.println("MQTT未连接，跳过数据上传");
  }
}

void publishSensorData(float temperature, float pressure, float altitude) {
  Serial.println("=== 开始发布传感器数据 ===");

  // 创建OneJSON格式的数据
  JsonDocument doc;

  // 获取当前时间戳（毫秒）- OneNET要求13位毫秒时间戳
  unsigned long long currentTimeSeconds = getCurrentTimestamp();
  unsigned long long timestamp = currentTimeSeconds * 1000ULL;

  // 确保时间戳是13位数字（毫秒级）
  if (timestamp < 1000000000000ULL)
  {
    // 如果时间戳太小，使用一个合理的基准时间
    timestamp = 1704067200000ULL + millis(); // 2024年1月1日 + 运行时间
  }

  Serial.println("当前Unix时间戳(秒): " + String(currentTimeSeconds));
  Serial.println("时间戳(毫秒): " + String(timestamp));
  Serial.println("时间戳长度: " + String(String(timestamp).length()) + " 位");

  // 构建标准OneNET JSON格式
  doc["id"] = String(random(100000, 999999)); // 6位随机消息ID
  doc["version"] = "1.0";

  // 构建params对象，每个属性包含value和time
  // 将浮点数转换为整数以符合OneNET步长要求
  doc["params"]["temperature"]["value"] = (int)round(temperature);
  doc["params"]["temperature"]["time"] = timestamp;

  doc["params"]["pressure"]["value"] = (int)round(pressure);
  doc["params"]["pressure"]["time"] = timestamp;

  doc["params"]["altitude"]["value"] = (int)round(altitude);
  doc["params"]["altitude"]["time"] = timestamp;

  String payload;
  serializeJson(doc, payload);

  Serial.println("--- 发布信息 ---");
  Serial.println("主题: " + propertyPostTopic);
  Serial.println("数据长度: " + String(payload.length()));
  Serial.println("JSON数据: " + payload);

  Serial.println("--- MQTT连接状态检查 ---");
  Serial.println("MQTT连接状态: " + String(mqttClient.connected() ? "已连接" : "未连接"));
  Serial.println("MQTT状态码: " + String(mqttClient.state()));

  if (mqttClient.publish(propertyPostTopic.c_str(), payload.c_str())) {
    Serial.println("✅ 数据发布成功！");
  } else {
    Serial.println("❌ 数据发布失败！");
    Serial.println("MQTT当前状态: " + String(mqttClient.state()));
    Serial.println("主题长度: " + String(propertyPostTopic.length()));
    Serial.println("负载长度: " + String(payload.length()));
  }
  Serial.println("=== 数据发布流程完成 ===");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.println("=== 收到MQTT消息 ===");
  Serial.print("主题: ");
  Serial.println(topic);

  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("消息内容: " + message);

  // 判断消息类型
  String topicStr = String(topic);
  if (topicStr.endsWith("/thing/property/post/reply"))
  {
    Serial.println("📨 这是属性上报回执消息");

    // 解析JSON回执
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, message);

    if (!error)
    {
      if (doc.containsKey("code"))
      {
        int code = doc["code"];
        if (code == 200)
        {
          Serial.println("✅ 数据上报成功！平台已确认接收");
        }
        else
        {
          Serial.println("❌ 数据上报失败，错误码: " + String(code));
          if (doc.containsKey("msg"))
          {
            Serial.println("错误信息: " + String(doc["msg"].as<String>()));
          }
        }
      }
    }
    else
    {
      Serial.println("⚠️ 回执消息JSON解析失败");
    }
  }
  else if (topicStr.endsWith("/thing/property/set"))
  {
    Serial.println("📥 这是属性设置消息");
  }
  else
  {
    Serial.println("❓ 未知消息类型");
  }

  Serial.println("=== 消息处理完成 ===");
}

// 获取当前时间戳（Unix时间戳）
uint32_t getCurrentTimestamp()
{
  time_t now = time(nullptr);

  // 如果时间同步失败，使用相对时间（从启动开始的秒数）
  if (now < 8 * 3600 * 2)
  {
    // 时间同步失败，使用一个基准时间戳 + 运行时间
    // 使用2024年1月1日作为基准时间戳: 1704067200
    uint32_t baseTimestamp = 1704067200;
    uint32_t runningSeconds = millis() / 1000;
    return baseTimestamp + runningSeconds;
  }

  return (uint32_t)now;
}
