#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>
#include <mbedtls/md.h>
#include "secrets.h"

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

// 函数声明
void connectWiFi();
void connectMQTT();
void readSensorData();
void publishSensorData(float temperature, float pressure, float altitude);
String generateToken(const String& productId, const String& deviceName, const String& accessKey, long expireTime);
String hmacSha1(const String& key, const String& data);
void mqttCallback(char* topic, byte* payload, unsigned int length);

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

  // 配置MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // 设置MQTT主题
  propertyPostTopic = "$sys/" + String(ONENET_PRODUCT_ID) + "/" + String(ONENET_DEVICE_NAME) + "/thing/property/post";
  propertySetTopic = "$sys/" + String(ONENET_PRODUCT_ID) + "/" + String(ONENET_DEVICE_NAME) + "/thing/property/set";

  // 连接MQTT
  connectMQTT();

  Serial.println("系统初始化完成，开始数据采集...");
}

void loop() {
  // 检查WiFi连接状态
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi连接断开，尝试重新连接...");
    connectWiFi();
  }

  // 检查MQTT连接状态
  if (!mqttClient.connected()) {
    Serial.println("MQTT连接断开，尝试重新连接...");
    connectMQTT();
  }

  // 处理MQTT消息
  mqttClient.loop();

  // 定时上传数据
  if (millis() - lastUploadTime >= UPLOAD_INTERVAL) {
    readSensorData();
    lastUploadTime = millis();
  }

  delay(1000); // 1秒延时
}

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("连接WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi连接成功！");
    Serial.print("IP地址: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi连接失败！");
  }
}

void connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.println("尝试连接MQTT服务器...");

    // 生成Token
    String token = generateToken(ONENET_PRODUCT_ID, ONENET_DEVICE_NAME, ONENET_ACCESS_KEY, TOKEN_EXPIRE_TIME);

    // 构建ClientID
    String clientId = String(ONENET_DEVICE_NAME) + "|securemode=2,signmethod=hmacsha1,timestamp=" + String(TOKEN_EXPIRE_TIME) + "|";

    Serial.println("ClientID: " + clientId);
    Serial.println("Username: " + String(ONENET_PRODUCT_ID));
    Serial.println("Password: " + token);

    if (mqttClient.connect(clientId.c_str(), ONENET_PRODUCT_ID, token.c_str())) {
      Serial.println("MQTT连接成功！");

      // 订阅属性设置主题
      mqttClient.subscribe(propertySetTopic.c_str());
      Serial.println("订阅主题: " + propertySetTopic);

    } else {
      Serial.print("MQTT连接失败，错误码: ");
      Serial.println(mqttClient.state());
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
  // 创建OneJSON格式的数据
  JsonDocument doc;

  // 获取当前时间戳（毫秒）
  unsigned long timestamp = millis();

  // 构建属性数据
  doc["id"] = String(millis()); // 消息ID
  doc["version"] = "1.0";
  doc["params"]["temperature"]["value"] = temperature;
  doc["params"]["temperature"]["time"] = timestamp;
  doc["params"]["pressure"]["value"] = pressure;
  doc["params"]["pressure"]["time"] = timestamp;
  doc["params"]["altitude"]["value"] = altitude;
  doc["params"]["altitude"]["time"] = timestamp;

  String payload;
  serializeJson(doc, payload);

  Serial.println("发布数据到OneNET...");
  Serial.println("主题: " + propertyPostTopic);
  Serial.println("数据: " + payload);

  if (mqttClient.publish(propertyPostTopic.c_str(), payload.c_str())) {
    Serial.println("数据发布成功！");
  } else {
    Serial.println("数据发布失败！");
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("收到MQTT消息，主题: ");
  Serial.println(topic);

  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("消息内容: " + message);
}

String generateToken(const String& productId, const String& deviceName, const String& accessKey, long expireTime) {
  // 构建签名字符串
  String stringToSign = "products/" + productId + "/devices/" + deviceName + expireTime;

  // 计算HMAC-SHA1
  String signature = hmacSha1(accessKey, stringToSign);

  return signature;
}

String hmacSha1(const String& key, const String& data) {
  // 使用mbedtls计算HMAC-SHA1
  unsigned char hmac_result[20];

  mbedtls_md_context_t ctx;
  mbedtls_md_type_t md_type = MBEDTLS_MD_SHA1;

  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char*)key.c_str(), key.length());
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)data.c_str(), data.length());
  mbedtls_md_hmac_finish(&ctx, hmac_result);
  mbedtls_md_free(&ctx);

  // 转换为Base64
  String encoded = "";
  const char* chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  for (int i = 0; i < 20; i += 3) {
    unsigned int val = (hmac_result[i] << 16) |
                       ((i + 1 < 20) ? (hmac_result[i + 1] << 8) : 0) |
                       ((i + 2 < 20) ? hmac_result[i + 2] : 0);

    encoded += chars[(val >> 18) & 0x3F];
    encoded += chars[(val >> 12) & 0x3F];
    encoded += (i + 1 < 20) ? chars[(val >> 6) & 0x3F] : '=';
    encoded += (i + 2 < 20) ? chars[val & 0x3F] : '=';
  }

  return encoded;
}