#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include "sensors.h"
#include "secrets.h"
#include "onenet_token.h"

// MQTT管理类
class MQTTManager {
private:
  AsyncMqttClient mqttClient;
  
  // MQTT主题
  String propertyPostTopic;
  String propertySetTopic;
  String propertyPostReplyTopic;
  
  // 连接状态
  bool mqttConnected = false;
  
  // 内部方法
  void onMqttConnect(bool sessionPresent);
  void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
  void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
  void onMqttPublish(uint16_t packetId);
  void onMqttSubscribe(uint16_t packetId, uint8_t qos);
  
  uint32_t getCurrentTimestamp();

public:
  MQTTManager();
  ~MQTTManager();
  
  // WiFi连接管理
  bool connectWiFi();
  bool isWiFiConnected();
  void checkWiFiConnection();
  
  // MQTT连接管理
  bool connectMQTT();
  bool isMQTTConnected();
  void checkMQTTConnection();
  void loopMQTT();
  
  // 数据发布
  bool publishSensorData(const SensorData& data);
  
  // 测试和调试
  void testSubscription(); // 测试订阅是否正常工作
  bool publishTestMessage(const String& payload); // 发布测试消息
  
  // 初始化
  bool initialize();
  
  // 获取连接状态信息
  String getWiFiStatus();
  String getMQTTStatus();
};

#endif // MQTT_MANAGER_H
