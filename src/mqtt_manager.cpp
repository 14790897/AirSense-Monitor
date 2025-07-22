#include "mqtt_manager.h"

// 构造函数
MQTTManager::MQTTManager() {
  // 设置MQTT回调
  mqttClient.onConnect([this](bool sessionPresent) { this->onMqttConnect(sessionPresent); });
  mqttClient.onDisconnect([this](AsyncMqttClientDisconnectReason reason) { this->onMqttDisconnect(reason); });
  mqttClient.onMessage([this](char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) { 
    this->onMqttMessage(topic, payload, properties, len, index, total); 
  });
  mqttClient.onPublish([this](uint16_t packetId) { this->onMqttPublish(packetId); });
  mqttClient.onSubscribe([this](uint16_t packetId, uint8_t qos) { this->onMqttSubscribe(packetId, qos); });
  
  // 设置MQTT主题
  propertyPostTopic = "$sys/" + String(ONENET_PRODUCT_ID) + "/" + String(ONENET_DEVICE_NAME) + "/thing/property/post";
  propertySetTopic = "$sys/" + String(ONENET_PRODUCT_ID) + "/" + String(ONENET_DEVICE_NAME) + "/thing/property/set";
  propertyPostReplyTopic = "$sys/" + String(ONENET_PRODUCT_ID) + "/" + String(ONENET_DEVICE_NAME) + "/thing/property/post/reply";
  
  // 打印主题信息
  Serial.println("=== MQTT主题配置 ===");
  Serial.println("属性上报主题: " + propertyPostTopic);
  Serial.println("属性设置主题: " + propertySetTopic);
  Serial.println("属性回执主题: " + propertyPostReplyTopic);
  Serial.println("====================");
}

// 析构函数
MQTTManager::~MQTTManager() {
  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }
  WiFi.disconnect();
}

// MQTT事件回调
void MQTTManager::onMqttConnect(bool sessionPresent) {
  Serial.println("🎉 MQTT连接成功！");
  Serial.println("Session present: " + String(sessionPresent));
  mqttConnected = true;

  // 订阅主题
  Serial.println("=== 开始订阅主题 ===");
  
  // 订阅属性上报回执主题（最重要的，用于获取上传结果）
  Serial.println("正在订阅属性回执主题...");
  uint16_t packetIdSub1 = mqttClient.subscribe(propertyPostReplyTopic.c_str(), 0);
  Serial.println("订阅属性回执主题，包ID: " + String(packetIdSub1));
  
  // 订阅属性设置主题
  Serial.println("正在订阅属性设置主题...");
  uint16_t packetIdSub2 = mqttClient.subscribe(propertySetTopic.c_str(), 0);
  Serial.println("订阅属性设置主题，包ID: " + String(packetIdSub2));
  
  Serial.println("=== 主题订阅完成 ===");
}

void MQTTManager::onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("❌ MQTT连接断开！");
  Serial.print("断开原因: ");
  switch (reason) {
    case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:
      Serial.println("TCP连接断开");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
      Serial.println("MQTT协议版本不可接受");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED:
      Serial.println("MQTT客户端标识符被拒绝");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:
      Serial.println("MQTT服务器不可用");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS:
      Serial.println("MQTT凭据格式错误");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:
      Serial.println("MQTT未授权");
      break;
    default:
      Serial.println("未知原因");
      break;
  }
  
  mqttConnected = false;
  
  // 如果WiFi仍然连接，尝试重新连接MQTT
  if (WiFi.isConnected()) {
    Serial.println("WiFi仍然连接，5秒后重新连接MQTT...");
    // 注意：这里不应该立即重连，应该在主循环中处理
  }
}

void MQTTManager::onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // 处理消息（可能是分片的）
  static String completeMessage = "";
  
  // 将payload转为字符串（注意：payload不是null结尾的）
  String currentChunk = "";
  for (size_t i = 0; i < len; i++) {
    currentChunk += (char)payload[i];
  }
  
  if (index == 0) {
    completeMessage = currentChunk;
  } else {
    completeMessage += currentChunk;
  }
  
  // 如果这是最后一个分片，处理完整消息
  if (index + len == total) {
    Serial.println();
    Serial.println("📩 =========================");
    Serial.println("📨 收到MQTT消息");
    Serial.println("📩 =========================");
    Serial.print("📍 主题: ");
    Serial.println(topic);
    Serial.println("📏 消息长度: " + String(total));
    Serial.println("📄 消息内容: " + completeMessage);

    // 判断消息类型并处理
    String topicStr = String(topic);
    if (topicStr.endsWith("/thing/property/post/reply")) {
      Serial.println("🎯 这是属性上报回执消息");
      Serial.println("🔍 正在解析OneNET平台回执...");

      // 解析JSON回执
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, completeMessage);

      if (!error) {
        Serial.println("✅ JSON解析成功");
        
        // 检查回执代码
        if (doc["code"].is<int>()) {
          int code = doc["code"];
          Serial.println("📊 平台回执代码: " + String(code));
          
          if (code == 200) {
            Serial.println("🎉 ✅ 数据上报成功！OneNET平台已确认接收");
            Serial.println("🌟 设备数据已成功存储到OneNET云平台");
          } else {
            Serial.println("❌ 数据上报失败！OneNET平台返回错误");
            Serial.println("🚨 错误码: " + String(code));
            
            // 显示错误信息
            if (doc["msg"].is<String>()) {
              String errorMsg = doc["msg"].as<String>();
              Serial.println("🔍 错误详情: " + errorMsg);
              
              // 常见错误码解释
              switch (code) {
                case 400:
                  Serial.println("💡 错误原因: 请求参数错误，检查JSON格式和物模型定义");
                  break;
                case 401:
                  Serial.println("💡 错误原因: 认证失败，检查设备密钥和Token");
                  break;
                case 403:
                  Serial.println("💡 错误原因: 权限不足，检查设备权限设置");
                  break;
                case 404:
                  Serial.println("💡 错误原因: 设备或属性不存在，检查产品ID和设备名");
                  break;
                case 500:
                  Serial.println("💡 错误原因: 服务器内部错误，稍后重试");
                  break;
                default:
                  Serial.println("💡 请查阅OneNET文档获取错误码详情");
              }
            }
          }
        } else {
          Serial.println("⚠️ 回执消息中没有找到code字段");
          Serial.println("🔍 完整回执内容: " + completeMessage);
        }
        
        // 显示消息ID（用于追踪）
        if (doc["id"].is<String>()) {
          Serial.println("🆔 消息ID: " + String(doc["id"].as<String>()));
        }
        
      } else {
        Serial.println("❌ 回执消息JSON解析失败");
        Serial.println("🚨 JSON错误: " + String(error.c_str()));
        Serial.println("🔍 原始消息: " + completeMessage);
      }
      
    } else if (topicStr.endsWith("/thing/property/set")) {
      Serial.println("📥 这是属性设置消息（平台下发）");
      Serial.println("🔧 平台正在设置设备属性");
      // TODO: 在这里可以处理平台下发的属性设置命令
      
    } else {
      Serial.println("❓ 未知消息类型");
      Serial.println("🔍 完整主题: " + topicStr);
      Serial.println("🔍 完整消息: " + completeMessage);
    }

    Serial.println("📩 =========================");
    Serial.println("✅ 消息处理完成");
    Serial.println("📩 =========================");
    Serial.println();
    
    // 重置消息缓冲区
    completeMessage = "";
  }
}

void MQTTManager::onMqttPublish(uint16_t packetId) {
  Serial.println("✅ 消息发布确认，包ID: " + String(packetId));
}

void MQTTManager::onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("✅ 订阅确认，包ID: " + String(packetId) + ", QoS: " + String(qos));
}

// WiFi连接
bool MQTTManager::connectWiFi() {
  Serial.println("=== 开始WiFi连接 ===");
  Serial.println("SSID: " + String(WIFI_SSID));
  Serial.println("密码长度: " + String(strlen(WIFI_PASSWORD)));

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("连接WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;

    // 每5次尝试打印一次状态
    if (attempts % 5 == 0) {
      Serial.println();
      Serial.print("尝试次数: " + String(attempts) + "/20, WiFi状态: " + getWiFiStatus());
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
    return true;
  } else {
    Serial.println();
    Serial.println("❌ WiFi连接失败！");
    Serial.println("最终状态: " + getWiFiStatus());
    return false;
  }
}

// 检查WiFi连接状态
bool MQTTManager::isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

// 检查WiFi连接并重连
void MQTTManager::checkWiFiConnection() {
  if (!isWiFiConnected()) {
    Serial.println("⚠️ WiFi连接断开，尝试重新连接...");
    Serial.println("当前WiFi状态: " + getWiFiStatus());
    connectWiFi();
  }
}

// MQTT连接
bool MQTTManager::connectMQTT() {
  if (!isWiFiConnected()) {
    Serial.println("❌ WiFi未连接，无法连接MQTT");
    return false;
  }

  if (mqttClient.connected()) {
    Serial.println("MQTT已经连接");
    return true;
  }

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

  // 使用官方Token生成函数
  char token[512] = {0};
  Serial.println("--- 开始生成Token ---");
  Serial.println("使用认证模式: 设备级认证（一机一密）");
  int result = onenet_token_generate(
      token,
      SIG_METHOD_SHA1,
      expireTime,
      ONENET_PRODUCT_ID,
      ONENET_DEVICE_NAME,
      ONENET_DEVICE_SECRET);

  Serial.println("Token生成结果码: " + String(result));
  if (result != 0) {
    Serial.println("❌ Token生成失败！错误码: " + String(result));
    return false;
  }

  Serial.println("完整Token: " + String(token));

  String password = String(token);
  String clientId = String(ONENET_DEVICE_NAME);

  Serial.println("--- MQTT连接参数 ---");
  Serial.println("ClientID: " + clientId);
  Serial.println("Username: " + String(ONENET_PRODUCT_ID));
  Serial.println("Password: " + password);

  // 配置AsyncMqttClient
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setClientId(clientId.c_str());
  mqttClient.setCredentials(ONENET_PRODUCT_ID, password.c_str());
  mqttClient.setKeepAlive(60);
  mqttClient.setCleanSession(true);

  Serial.println("--- 尝试MQTT连接 ---");
  mqttClient.connect();
  
  // 等待连接完成
  unsigned long startTime = millis();
  while (!mqttClient.connected() && (millis() - startTime) < 10000) {
    delay(100);
  }
  
  if (mqttClient.connected()) {
    Serial.println("🎉 MQTT连接建立成功！");
    return true;
  } else {
    Serial.println("❌ MQTT连接超时失败！");
    return false;
  }
}

// 检查MQTT连接状态
bool MQTTManager::isMQTTConnected() {
  return mqttClient.connected() && mqttConnected;
}

// 检查MQTT连接并重连
void MQTTManager::checkMQTTConnection() {
  if (!isMQTTConnected()) {
    Serial.println("⚠️ MQTT连接断开，尝试重新连接...");
    connectMQTT();
  }
}

// MQTT循环处理
void MQTTManager::loopMQTT() {
  // AsyncMqttClient是异步的，不需要主动调用loop
  // 但我们可以在这里检查连接状态
  if (!mqttClient.connected() && mqttConnected) {
    mqttConnected = false;
    Serial.println("检测到MQTT连接丢失");
  }
}

// 发布传感器数据
bool MQTTManager::publishSensorData(const SensorData& data) {
  if (!isMQTTConnected()) {
    Serial.println("❌ MQTT未连接，跳过数据上传");
    return false;
  }

  Serial.println("=== 开始发布传感器数据 ===");

  // 创建OneJSON格式的数据
  JsonDocument doc;

  // 获取当前时间戳（毫秒）
  unsigned long long currentTimeSeconds = getCurrentTimestamp();
  unsigned long long timestamp = currentTimeSeconds * 1000ULL;

  Serial.println("当前Unix时间戳(秒): " + String(currentTimeSeconds));
  Serial.println("时间戳(毫秒): " + String(timestamp));

  // 构建标准OneNET JSON格式
  doc["id"] = String(random(100000, 999999));
  doc["version"] = "1.0";

  // 构建params对象 - 使用完整字段名，保持和原有OneNET物模型一致
#if ENABLE_BMP280
  doc["params"]["temperature"]["value"] = data.temperature;  // 浮点数
  doc["params"]["temperature"]["time"] = timestamp;

  doc["params"]["pressure"]["value"] = data.pressure;  // 浮点数
  doc["params"]["pressure"]["time"] = timestamp;

  doc["params"]["altitude"]["value"] = data.altitude;  // 浮点数
  doc["params"]["altitude"]["time"] = timestamp;
#endif

#if ENABLE_MQ135
  // 添加MQ135空气质量数据（如果传感器已校准）
  if (data.mq135_calibrated) {
    doc["params"]["air_quality_ppm"]["value"] = (int)round(data.air_quality_ppm);  // ppm保持整数
    doc["params"]["air_quality_ppm"]["time"] = timestamp;

    doc["params"]["air_quality_corrected_ppm"]["value"] = (int)round(data.air_quality_corrected_ppm);  // ppm保持整数
    doc["params"]["air_quality_corrected_ppm"]["time"] = timestamp;
  }
#endif

#if ENABLE_21VOC
  // 添加21VOC五合一传感器数据（如果有效）
  if (data.tvoc_data.valid) {
    doc["params"]["voc_ugm3"]["value"] = data.tvoc_data.voc_ugm3;  // μg/m³保持整数（传感器原始值）
    doc["params"]["voc_ugm3"]["time"] = timestamp;

    doc["params"]["ch2o_ugm3"]["value"] = data.tvoc_data.ch2o_ugm3;  // μg/m³保持整数（传感器原始值）
    doc["params"]["ch2o_ugm3"]["time"] = timestamp;

    doc["params"]["eco2_ppm"]["value"] = data.tvoc_data.eco2_ppm;  // ppm保持整数
    doc["params"]["eco2_ppm"]["time"] = timestamp;

    // 使用21VOC的温湿度数据覆盖BMP280的温度（如果可用）
    doc["params"]["temperature"]["value"] = data.tvoc_data.temperature_c;  // 浮点数
    doc["params"]["temperature"]["time"] = timestamp;

    doc["params"]["humidity_rh"]["value"] = data.tvoc_data.humidity_rh;  // 浮点数
    doc["params"]["humidity_rh"]["time"] = timestamp;
  }
#endif

#if ENABLE_VOC_CO2_HCHO
  // 添加VOC-CO2-HCHO三合一传感器数据（如果有效）
  if (data.voc_co2_hcho_data.valid) {
    // 将mg/m³转换为μg/m³ (1 mg/m³ = 1000 μg/m³)
    doc["params"]["voc_ugm3"]["value"] = data.voc_co2_hcho_data.tvoc_mgm3 * 1000;  // TVOC浓度转换为 μg/m³
    doc["params"]["voc_ugm3"]["time"] = timestamp;

    doc["params"]["ch2o_ugm3"]["value"] = data.voc_co2_hcho_data.ch2o_mgm3 * 1000;  // 甲醛浓度转换为 μg/m³
    doc["params"]["ch2o_ugm3"]["time"] = timestamp;

    doc["params"]["eco2_ppm"]["value"] = data.voc_co2_hcho_data.co2_mgm3 * 1000;  // CO₂浓度转换为 μg/m³
    doc["params"]["eco2_ppm"]["time"] = timestamp;
  }
#endif

#if ENABLE_HW181_MIC
  // 添加HW181-MIC分贝检测数据（如果有效且已校准）
  if (data.mic_data.valid && data.mic_data.calibrated) {
    doc["params"]["decibels"]["value"] = data.mic_data.decibels;  // 浮点数
    doc["params"]["decibels"]["time"] = timestamp;

    doc["params"]["sound_detected"]["value"] = data.mic_data.sound_detected ? 1 : 0;  // 布尔值转整数
    doc["params"]["sound_detected"]["time"] = timestamp;

    doc["params"]["sound_voltage"]["value"] = data.mic_data.sound_voltage;  // 声音电压值（浮点数）
    doc["params"]["sound_voltage"]["time"] = timestamp;

  }
#endif

  String payload;
  serializeJson(doc, payload);

  Serial.println("--- 发布信息 ---");
  Serial.println("发布主题: " + propertyPostTopic);
  Serial.println("期望回执主题: " + propertyPostReplyTopic);
  Serial.println("数据长度: " + String(payload.length()) + " 字节");
  Serial.println("JSON数据: " + payload);

  Serial.println("--- MQTT连接状态检查 ---");
  Serial.println("MQTT连接状态: " + String(isMQTTConnected() ? "已连接" : "未连接"));
  Serial.println("WiFi连接状态: " + String(WiFi.status()));
  Serial.println("WiFi RSSI: " + String(WiFi.RSSI()) + " dBm");
  Serial.println("可用内存: " + String(ESP.getFreeHeap()) + " 字节");

  // 使用AsyncMqttClient发布消息
  uint16_t packetId = mqttClient.publish(propertyPostTopic.c_str(), 0, false, payload.c_str());
  
  if (packetId != 0) {
    Serial.println("✅ 数据发布成功！包ID: " + String(packetId));
    Serial.println("⏳ 等待OneNET平台回执消息...");
    Serial.println("📡 回执消息将通过主题接收: " + propertyPostReplyTopic);
    Serial.println("💡 提示：如果长时间没有收到回执，请检查：");
    Serial.println("   1. 网络连接是否稳定");
    Serial.println("   2. OneNET设备是否在线");
    Serial.println("   3. 物模型属性是否配置正确");
    Serial.println("   4. 负载格式是否正确");
  } else {
    Serial.println("❌ 数据发布失败！");
    Serial.println("负载长度: " + String(payload.length()) + " 字节");
    Serial.println("可能的原因：");
    Serial.println("  - MQTT连接不稳定");
    Serial.println("  - 网络连接问题");
    Serial.println("  - OneNET服务器拒绝连接");
    Serial.println("  - JSON格式不正确");
    
    // 尝试重新连接
    if (!isMQTTConnected()) {
      Serial.println("检测到MQTT连接断开，尝试重新连接...");
      connectMQTT();
    }
  }
  
  Serial.println("=== 数据发布流程完成 ===");
  return packetId != 0;
}

// 发布测试消息
bool MQTTManager::publishTestMessage(const String& payload) {
  if (!isMQTTConnected()) {
    Serial.println("❌ MQTT未连接，无法发布测试消息");
    return false;
  }

  Serial.println("=== 发布测试消息 ===");
  Serial.println("主题: " + propertyPostTopic);
  Serial.println("负载: " + payload);
  Serial.println("负载大小: " + String(payload.length()) + " 字节");

  uint16_t packetId = mqttClient.publish(propertyPostTopic.c_str(), 0, false, payload.c_str());
  
  if (packetId != 0) {
    Serial.println("✅ 测试消息发布成功！包ID: " + String(packetId));
  } else {
    Serial.println("❌ 测试消息发布失败！");
    Serial.println("MQTT状态: " + getMQTTStatus());
  }
  
  return packetId != 0;
}

// 初始化
bool MQTTManager::initialize() {
  // 首先连接WiFi
  if (!connectWiFi()) {
    return false;
  }

  // WiFi连接成功后，配置NTP时间同步
  Serial.println("正在同步时间...");
  configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  // 等待时间同步完成
  time_t now = time(nullptr);
  int timeout = 0;
  while (now < 8 * 3600 * 2 && timeout < 30) {
    delay(1000);
    now = time(nullptr);
    timeout++;
    Serial.print(".");
  }
  Serial.println();

  if (now < 8 * 3600 * 2) {
    Serial.println("⚠️ 时间同步失败，将使用相对时间");
  } else {
    Serial.println("✅ 时间同步成功");
    Serial.println("当前时间: " + String(ctime(&now)));
  }

  // 最后连接MQTT
  return connectMQTT();
}

// 获取当前时间戳
uint32_t MQTTManager::getCurrentTimestamp() {
  time_t now = time(nullptr);

  if (now < 8 * 3600 * 2) {
    uint32_t baseTimestamp = 1704067200;
    uint32_t runningSeconds = millis() / 1000;
    return baseTimestamp + runningSeconds;
  }

  return (uint32_t)now;
}

// 获取WiFi状态字符串
String MQTTManager::getWiFiStatus() {
  switch (WiFi.status()) {
    case WL_IDLE_STATUS:
      return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL:
      return "WL_NO_SSID_AVAIL - 找不到SSID";
    case WL_SCAN_COMPLETED:
      return "WL_SCAN_COMPLETED";
    case WL_CONNECTED:
      return "WL_CONNECTED";
    case WL_CONNECT_FAILED:
      return "WL_CONNECT_FAILED - 连接失败";
    case WL_CONNECTION_LOST:
      return "WL_CONNECTION_LOST - 连接丢失";
    case WL_DISCONNECTED:
      return "WL_DISCONNECTED - 未连接";
    default:
      return "未知状态: " + String(WiFi.status());
  }
}

// 获取MQTT状态字符串
String MQTTManager::getMQTTStatus() {
  if (mqttClient.connected()) {
    return "MQTT_CONNECTED - 已连接";
  } else {
    return "MQTT_DISCONNECTED - 未连接";
  }
}

// 测试订阅功能
void MQTTManager::testSubscription() {
  if (!isMQTTConnected()) {
    Serial.println("❌ MQTT未连接，无法测试订阅");
    return;
  }

  Serial.println("=== 测试MQTT订阅功能 ===");
  
  // 模拟OneNET平台的回执消息格式
  String testTopic = propertyPostReplyTopic;
  String testMessage = "{\"id\":\"test123\",\"code\":200,\"msg\":\"success\",\"timestamp\":" + String(millis()) + "}";
  
  Serial.println("🧪 发送模拟回执消息到: " + testTopic);
  Serial.println("📄 测试消息内容: " + testMessage);
  Serial.println("⏰ 等待3秒，然后发送测试消息...");
  
  delay(3000); // 等待3秒确保准备好
  
  uint16_t packetId = mqttClient.publish(testTopic.c_str(), 0, false, testMessage.c_str());
  if (packetId != 0) {
    Serial.println("✅ 测试消息发送成功，包ID: " + String(packetId));
    Serial.println("🔍 如果订阅正常，应该会立即收到回调消息");
    Serial.println("📡 请观察上方的MQTT回调信息");
  } else {
    Serial.println("❌ 测试消息发送失败");
    Serial.println("🔍 MQTT状态: " + getMQTTStatus());
  }
  
  Serial.println("=== 订阅测试完成 ===");
  Serial.println("💡 提示：如果没有收到回调消息，说明订阅功能有问题");
}
