#include <Arduino.h>
#include "sensors.h"
#include "mqtt_manager.h"

// 传感器管理器
SensorManager sensorManager;

// MQTT管理器
MQTTManager mqttManager;

// 数据上传间隔 (毫秒)
const unsigned long UPLOAD_INTERVAL = 30000; // 30秒
unsigned long lastUploadTime = 0;

// 函数声明
void readSensorData();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("BMP280 气压记录系统启动...");

  // 初始化传感器
  Serial.println("开始初始化传感器...");
  if (!sensorManager.initAllSensors()) {
    Serial.println("⚠️ 部分传感器初始化失败，但系统继续运行");
  }

  // 初始化MQTT和WiFi
  Serial.println("开始初始化网络连接...");
  if (!mqttManager.initialize()) {
    Serial.println("⚠️ 网络连接初始化失败");
  }

  Serial.println("系统初始化完成，开始数据采集...");
}

void loop() {
  // 检查WiFi连接状态
  mqttManager.checkWiFiConnection();

  // 检查MQTT连接状态
  mqttManager.checkMQTTConnection();

  // 处理MQTT消息
  mqttManager.loopMQTT();

#if ENABLE_MQ135
  // 处理MQ135非阻塞校准
  sensorManager.processMQ135Calibration();
#endif

#if ENABLE_21VOC
  // 定时更新21VOC传感器数据
  sensorManager.update21VOCData();
#endif

  // 定时上传数据
  if (millis() - lastUploadTime >= UPLOAD_INTERVAL) {
    Serial.println("⏰ 到达上传时间间隔，开始读取传感器数据...");
    readSensorData();
    lastUploadTime = millis();
    Serial.println("下次上传时间: " + String((lastUploadTime + UPLOAD_INTERVAL) / 1000) + "秒后");
  }

  // 处理串口命令（用于调试和管理）
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.equals("calibrate") || command.equals("cal")) {
#if ENABLE_MQ135
      if (sensorManager.getCalibrationState() == CAL_IDLE) {
        Serial.println("开始重新校准MQ135传感器...");
        sensorManager.calibrateMQ135();
      } else {
        Serial.println("校准正在进行中，请等待完成");
        sensorManager.showCalibrationStatus();
      }
#else
      Serial.println("⚠️ MQ135传感器已禁用");
#endif
    }
    else if (command.equals("delete_cal") || command.equals("del")) {
#if ENABLE_MQ135
      sensorManager.deleteCalibration();
      Serial.println("请重启设备以重新校准");
#else
      Serial.println("⚠️ MQ135传感器已禁用");
#endif
    }
    else if (command.equals("show_cal") || command.equals("show")) {
#if ENABLE_MQ135
      sensorManager.showCalibration();
#else
      Serial.println("⚠️ MQ135传感器已禁用");
#endif
    }
    else if (command.equals("show_tvoc") || command.equals("tvoc")) {
#if ENABLE_21VOC
      sensorManager.show21VOCData();
#else
      Serial.println("⚠️ 21VOC传感器已禁用");
#endif
    }
    else if (command.equals("cal_status") || command.equals("status")) {
#if ENABLE_MQ135
      sensorManager.showCalibrationStatus();
#else
      Serial.println("⚠️ MQ135传感器已禁用");
#endif
    }
    else if (command.equals("mic_cal") || command.equals("mcal")) {
#if ENABLE_HW181_MIC
      Serial.println("开始HW181-MIC传感器校准...");
      sensorManager.calibrateHW181MIC();
#else
      Serial.println("⚠️ HW181-MIC传感器已禁用");
#endif
    }
    else if (command.equals("show_mic") || command.equals("mic")) {
#if ENABLE_HW181_MIC
      sensorManager.showMicData();
#else
      Serial.println("⚠️ HW181-MIC传感器已禁用");
#endif
    }
    else if (command.equals("mic_status") || command.equals("mstatus")) {
#if ENABLE_HW181_MIC
      sensorManager.showMicCalibrationStatus();
#else
      Serial.println("⚠️ HW181-MIC传感器已禁用");
#endif
    }
    else if (command.equals("delete_mic_cal") || command.equals("dmic")) {
#if ENABLE_HW181_MIC
      sensorManager.deleteMicCalibration();
      Serial.println("请重启设备以重新校准");
#else
      Serial.println("⚠️ HW181-MIC传感器已禁用");
#endif
    }
    else if (command.equals("help")) {
      Serial.println("=== 可用命令 ===");
      Serial.println("calibrate 或 cal - 开始校准MQ135传感器");
      Serial.println("delete_cal 或 del - 删除保存的校准值");
      Serial.println("show_cal 或 show - 显示当前保存的校准值");
      Serial.println("show_tvoc 或 tvoc - 显示21VOC传感器数据");
      Serial.println("cal_status 或 status - 显示校准状态");
      Serial.println("mic_cal 或 mcal - 开始HW181-MIC传感器校准");
      Serial.println("show_mic 或 mic - 显示HW181-MIC传感器数据");
      Serial.println("mic_status 或 mstatus - 显示HW181-MIC校准状态");
      Serial.println("delete_mic_cal 或 dmic - 删除HW181-MIC校准数据");
      Serial.println("debug_tvoc 或 debug - 调试21VOC传感器");
      Serial.println("test_tvoc 或 test - 测试21VOC传感器连接");
      Serial.println("wifi_status - 显示WiFi状态");
      Serial.println("mqtt_status - 显示MQTT状态");
      Serial.println("test_sub - 测试MQTT订阅功能");
      Serial.println("help - 显示此帮助信息");
      Serial.println("===============");
    }
    else if (command.equals("debug_tvoc") || command.equals("debug")) {
      sensorManager.debug21VOCSensor();
    }
    else if (command.equals("test_tvoc") || command.equals("test")) {
      sensorManager.test21VOCConnection();
    }
    else if (command.equals("wifi_status")) {
      Serial.println("WiFi状态: " + mqttManager.getWiFiStatus());
      if (mqttManager.isWiFiConnected()) {
        Serial.println("IP地址: " + WiFi.localIP().toString());
        Serial.println("信号强度: " + String(WiFi.RSSI()));
      }
    }
    else if (command.equals("mqtt_status")) {
      Serial.println("MQTT状态: " + mqttManager.getMQTTStatus());
    }
    else if (command.equals("test_sub")) {
      mqttManager.testSubscription();
    }
    else if (command.equals("test_simple") || command.equals("simple")) {
      Serial.println("=== 测试简化MQTT数据发布 ===");
      if (mqttManager.isMQTTConnected()) {
        // 发布一个非常简单的测试消息
        String testPayload = "{\"id\":\"123\",\"params\":{\"test\":25}}";
        Serial.println("发布测试负载: " + testPayload);
        Serial.println("负载大小: " + String(testPayload.length()) + " 字节");
        
        bool result = mqttManager.publishTestMessage(testPayload);
        if (result) {
          Serial.println("✅ 简化测试发布成功");
        } else {
          Serial.println("❌ 简化测试发布失败");
        }
      } else {
        Serial.println("❌ MQTT未连接");
      }
    }
    else if (command.equals("read_now") || command.equals("read")) {
      Serial.println("=== 立即读取传感器数据 ===");
      readSensorData();
    }
  }

  delay(1000); // 1秒延时
}



void readSensorData() {
  // 读取所有传感器数据
  SensorData data = sensorManager.readAllSensors();

  // 打印传感器数据
  Serial.println("=== 传感器数据 ===");
  
#if ENABLE_BMP280
  Serial.printf("温度: %.2f °C\n", data.temperature);
  Serial.printf("气压: %.2f hPa\n", data.pressure);
  Serial.printf("海拔: %.2f m\n", data.altitude);
#endif

#if ENABLE_MQ135
  if (data.mq135_calibrated) {
    Serial.printf("空气质量(标准): %.1f ppm\n", data.air_quality_ppm);
    Serial.printf("空气质量(校正): %.1f ppm\n", data.air_quality_corrected_ppm);
    Serial.print("空气质量等级: ");
    if (data.air_quality_corrected_ppm < 50) {
      Serial.println("优秀");
    } else if (data.air_quality_corrected_ppm < 100) {
      Serial.println("良好");
    } else if (data.air_quality_corrected_ppm < 200) {
      Serial.println("一般");
    } else if (data.air_quality_corrected_ppm < 400) {
      Serial.println("较差");
    } else if (data.air_quality_corrected_ppm < 1000) {
      Serial.println("很差");
    } else {
      Serial.println("极差");
    }
  } else {
    Serial.println("MQ135: 未校准或未初始化");
  }
#endif

#if ENABLE_21VOC
  // 显示21VOC传感器数据
  if (data.tvoc_data.valid) {
    Serial.println("--- 21VOC五合一传感器数据 ---");
    Serial.printf("VOC空气质量: %d μg/m³\n", data.tvoc_data.voc_ugm3);
    Serial.printf("甲醛浓度: %d μg/m³\n", data.tvoc_data.ch2o_ugm3);
    Serial.printf("eCO2浓度: %d ppm\n", data.tvoc_data.eco2_ppm);
    Serial.printf("温度: %.1f°C\n", data.tvoc_data.temperature_c);
    Serial.printf("湿度: %.1f%%RH\n", data.tvoc_data.humidity_rh);
    Serial.println("使用21VOC温湿度补偿MQ135");
  } else {
    Serial.println("21VOC: 无有效数据");
  }
#endif

  Serial.println("==================");

  // 发布到OneNET
  if (mqttManager.isMQTTConnected()) {
    mqttManager.publishSensorData(data);
  } else {
    Serial.println("MQTT未连接，跳过数据上传");
  }
}


