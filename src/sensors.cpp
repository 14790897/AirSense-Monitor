#include "sensors.h"
#include <time.h>

// 静态成员初始化
#if ENABLE_MQ135
const float SensorManager::MANUAL_RZERO_VALUE = 76.63;
#endif

// 构造函数
SensorManager::SensorManager() 
#if ENABLE_21VOC
: sensorSerial(1)
#endif
{
#if ENABLE_21VOC
  lastTVOCReadTime = 0;
  lastTVOCReading.valid = false;
  lastTVOCReading.timestamp = 0;
#endif

#if ENABLE_MQ135
  mq135_sensor_ptr = nullptr;
  ambient_temperature = 31.0;
  ambient_humidity = 60.0;
  calibratedRZero = 0.0;
  mq135_calibrated = false;
  calibrationState = CAL_IDLE;
  calibrationStartTime = 0;
  calibrationLastUpdate = 0;
  currentSample = 0;
  rZeroSum = 0;
  tempSensorPtr = nullptr;
#endif

#if ENABLE_HW181_MIC
  mic_calibrated = false;
  mic_baseline_value = 0;
  mic_change_threshold = 50;
  last_db_value = MIN_DB;
  last_analog_value = 0;
  min_db_in_period = MAX_DB;
  avg_db_sum = 0;
  db_sample_count = 0;
  sound_detected_count = 0;
  total_readings = 0;
  lastMicReading.valid = false;
  lastMicReading.timestamp = 0;
  
  // 初始化声音检测改进算法相关变量
  recent_values_index = 0;
  consecutive_sound_count = 0;
  baseline_moving_avg = 0.0;
  for (int i = 0; i < SOUND_DETECTION_WINDOW; i++) {
    recent_analog_values[i] = 0;
  }
#endif
}

// 析构函数
SensorManager::~SensorManager() {
#if ENABLE_MQ135
  if (mq135_sensor_ptr) {
    delete mq135_sensor_ptr;
    mq135_sensor_ptr = nullptr;
  }
  if (tempSensorPtr) {
    delete tempSensorPtr;
    tempSensorPtr = nullptr;
  }
  preferences.end();
#endif
}

// 获取当前时间戳（内部辅助函数）
uint32_t getCurrentTimestamp() {
  time_t now = time(nullptr);
  if (now < 8 * 3600 * 2) {
    uint32_t baseTimestamp = 1704067200;
    uint32_t runningSeconds = millis() / 1000;
    return baseTimestamp + runningSeconds;
  }
  return (uint32_t)now;
}

// 初始化BMP280传感器
bool SensorManager::initBMP280() {
#if ENABLE_BMP280
  Serial.println("=== 初始化BMP280传感器 ===");
  
  Wire.begin();
  
  if (!bmp.begin(0x76)) { // BMP280默认I2C地址是0x76，有些模块是0x77
    Serial.println("❌ 无法找到BMP280传感器，请检查接线！");
    return false;
  }

  // 配置BMP280传感器
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // 工作模式
                  Adafruit_BMP280::SAMPLING_X2,     // 温度过采样
                  Adafruit_BMP280::SAMPLING_X16,    // 气压过采样
                  Adafruit_BMP280::FILTER_X16,      // 滤波
                  Adafruit_BMP280::STANDBY_MS_500); // 待机时间

  Serial.println("✅ BMP280传感器初始化成功！");
  return true;
#else
  Serial.println("⚠️  BMP280传感器已禁用");
  return true;
#endif
}

// 初始化21VOC传感器
bool SensorManager::init21VOCSensor() {
#if ENABLE_21VOC
  Serial.println("=== 初始化21VOC五合一传感器 ===");
  Serial.printf("RX引脚: GPIO%d\n", UART_RX_PIN);
  Serial.printf("TX引脚: GPIO%d\n", UART_TX_PIN);
  Serial.printf("波特率: %d\n", UART_BAUD_RATE);
  
  // 配置UART参数: 波特率, 数据位, 停止位, 校验位
  sensorSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  // 等待串口稳定
  delay(100);
  
  if (sensorSerial) {
    Serial.println("✅ 21VOC传感器UART初始化成功");
    
    // 清空接收缓冲区
    while (sensorSerial.available()) {
      sensorSerial.read();
    }
    
    // 初始化数据结构
    lastTVOCReading.valid = false;
    lastTVOCReading.timestamp = 0;
    
    Serial.println("=== 21VOC传感器初始化完成 ===");
    return true;
  } else {
    Serial.println("❌ 21VOC传感器UART初始化失败");
    return false;
  }
#else
  Serial.println("⚠️  21VOC传感器已禁用");
  return true;
#endif
}

// 初始化MQ135传感器
bool SensorManager::initMQ135() {
#if ENABLE_MQ135
  Serial.println("=== MQ135 空气质量传感器初始化 ===");

  // 初始化Preferences
  if (!initPreferences()) {
    Serial.println("⚠️ Preferences初始化失败，MQ135校准值将无法保存");
  }

  // 检查引脚连接
  int adcValue = analogRead(MQ135_AO_PIN);
  float voltage = (adcValue / 4095.0) * 3.3;

  Serial.print("MQ135引脚检测 - ADC: ");
  Serial.print(adcValue);
  Serial.print(" (");
  Serial.print(voltage, 2);
  Serial.println("V)");

  if (voltage < 0.1) {
    Serial.println("❌ MQ135传感器未检测到信号，请检查连接");
    return false;
  }

  if (voltage > 3.2) {
    Serial.println("❌ MQ135传感器电压异常，请检查电路");
    return false;
  }

  Serial.println("✅ MQ135传感器信号正常");

  // 尝试从Preferences加载已保存的RZero值
  float savedRZero = loadRZeroFromPreferences();
  if (savedRZero > 0 && savedRZero >= 50 && savedRZero <= 150) {
    Serial.println("=== 发现已保存的校准值 ===");
    Serial.print("已保存的RZero值: ");
    Serial.println(savedRZero, 2);
    Serial.println("是否使用已保存的校准值？");
    Serial.println("发送 'y' 使用保存值，发送 'n' 重新校准，10秒后自动使用保存值");
    
    // 等待用户输入，最多10秒
    unsigned long startTime = millis();
    String userInput = "";
    
    while (millis() - startTime < 10000) { // 10秒超时
      if (Serial.available()) {
        userInput = Serial.readStringUntil('\n');
        userInput.trim();
        break;
      }
      delay(100);
    }
    
    if (userInput.equals("y") || userInput.equals("Y")) {
      // 使用已保存的校准值
      calibratedRZero = savedRZero;
      mq135_sensor_ptr = new MQ135(MQ135_AO_PIN, calibratedRZero);
      mq135_calibrated = true;
      Serial.println("✅ 使用已保存的校准值");
      Serial.println("=== MQ135初始化完成 ===");
      return true;
    } else if (userInput.equals("n") || userInput.equals("N")) {
      Serial.println("用户选择重新校准");
    } else {
      // 超时，使用保存的值
      calibratedRZero = savedRZero;
      mq135_sensor_ptr = new MQ135(MQ135_AO_PIN, calibratedRZero);
      mq135_calibrated = true;
      Serial.println("✅ 超时，使用已保存的校准值");
      Serial.println("=== MQ135初始化完成 ===");
      return true;
    }
  } else {
    Serial.println("未找到有效的已保存校准值，开始新的校准过程");
  }

  // 开始校准过程
  startCalibration();
  return true;
#else
  Serial.println("⚠️  MQ135传感器已禁用");
  return true;
#endif
}

// 初始化所有传感器
bool SensorManager::initAllSensors() {
  bool success = true;
  
#if ENABLE_BMP280
  success &= initBMP280();
#endif

#if ENABLE_21VOC
  success &= init21VOCSensor();
#endif

#if ENABLE_MQ135
  success &= initMQ135();
#endif

#if ENABLE_HW181_MIC
  success &= initHW181MIC();
#endif
  
  return success;
}

// 读取所有传感器数据
SensorData SensorManager::readAllSensors() {
  SensorData data;
  data.timestamp = millis();
  
#if ENABLE_BMP280
  // 读取BMP280传感器数据
  data.temperature = bmp.readTemperature();
  data.pressure = bmp.readPressure() / 100.0F; // 转换为hPa
  data.altitude = bmp.readAltitude(1013.25);   // 海平面气压1013.25hPa
#endif

#if ENABLE_MQ135
  // 读取MQ135传感器数据
  data.air_quality_ppm = 0.0;
  data.air_quality_corrected_ppm = 0.0;
  data.mq135_calibrated = mq135_calibrated;

  if (mq135_sensor_ptr != nullptr && mq135_calibrated) {
    // 使用21VOC传感器的温湿度数据进行MQ135补偿（如果可用）
#if ENABLE_21VOC
    if (lastTVOCReading.valid) {
      ambient_temperature = lastTVOCReading.temperature_c;
      ambient_humidity = lastTVOCReading.humidity_rh;
    } else {
#endif
#if ENABLE_BMP280
      // 如果21VOC数据不可用，使用BMP280的温度数据
      ambient_temperature = data.temperature;
#endif
#if ENABLE_21VOC
    }
#endif

    data.air_quality_ppm = mq135_sensor_ptr->getPPM();
    data.air_quality_corrected_ppm = mq135_sensor_ptr->getCorrectedPPM(ambient_temperature, ambient_humidity);
  }
#endif

#if ENABLE_HW181_MIC
  // 读取HW181-MIC传感器数据
  readHW181MICSensor(data.mic_data);
#endif

#if ENABLE_21VOC
  // 21VOC数据
  data.tvoc_data = lastTVOCReading;
#endif

  return data;
}

// 定时更新21VOC数据
void SensorManager::update21VOCData() {
#if ENABLE_21VOC
  if (millis() - lastTVOCReadTime >= TVOC_READ_INTERVAL) {
    lastTVOCReadTime = millis();
    
    TVOCData currentTVOCReading;
    if (read21VOCSensor(currentTVOCReading)) {
      if (currentTVOCReading.valid && validate21VOCData(currentTVOCReading)) {
        lastTVOCReading = currentTVOCReading;
#if ENABLE_MQ135
        // 更新MQ135使用的环境参数
        ambient_temperature = currentTVOCReading.temperature_c;
        ambient_humidity = currentTVOCReading.humidity_rh;
#endif
        print21VOCReading(currentTVOCReading);
      }
    } else {
      Serial.println("❌ 未接收到有效的21VOC传感器数据");
    }
  }
#endif
}

#if ENABLE_21VOC
// 读取21VOC传感器数据
bool SensorManager::read21VOCSensor(TVOCData &data) {
#if ENABLE_21VOC
  uint8_t buffer[64];
  int bytesRead = 0;
  unsigned long startTime = millis();
  
  // 清空接收缓冲区中的旧数据
  while (sensorSerial.available()) {
    sensorSerial.read();
    delay(1);
  }
  
  // 等待数据到达，超时2秒
  while (bytesRead < sizeof(buffer) && (millis() - startTime) < 2000) {
    if (sensorSerial.available()) {
      buffer[bytesRead] = sensorSerial.read();
      bytesRead++;
      
      // 如果连续没有新数据超过100ms，认为一帧数据接收完成
      unsigned long lastByteTime = millis();
      while (!sensorSerial.available() && (millis() - lastByteTime) < 100) {
        delay(5);
      }
      if (!sensorSerial.available()) {
        break;
      }
    }
    delay(10);
  }

  if (bytesRead > 0) {
    // 调试：显示接收到的原始数据
    Serial.printf("21VOC接收到 %d 字节: ", bytesRead);
    for (int i = 0; i < bytesRead; i++) {
      Serial.printf("%02X ", buffer[i]);
    }
    Serial.println();
    
    // 解析接收到的数据
    parse21VOCData(buffer, bytesRead, data);
    data.timestamp = millis();
    return true;
  } else {
    Serial.println("21VOC: 超时，未接收到数据");
  }

  return false;
#else
  // 如果21VOC禁用，返回默认数据
  data.valid = false;
  return false;
#endif
}

// 解析21VOC五合一传感器数据
void SensorManager::parse21VOCData(uint8_t* buffer, int length, TVOCData &data) {
#if ENABLE_21VOC
  // 初始化数据结构
  data.voc_ugm3 = 0;
  data.ch2o_ugm3 = 0;
  data.eco2_ppm = 0;
  data.temperature_c = 0.0;
  data.humidity_rh = 0.0;
  data.valid = false;
  
  Serial.printf("解析21VOC数据: 长度=%d ", length);
  
  // 检查数据帧长度
  if (length < 12) {
    Serial.printf("数据长度不足（需要12字节，实际%d字节）\n", length);
    return;
  }
  
  // 寻找正确的数据帧起始位置 (0x2C开头)
  int frameStart = -1;
  for (int i = 0; i <= length - 12; i++) {
    if (buffer[i] == 0x2C) {
      frameStart = i;
      Serial.printf("找到帧头0x2C在位置%d ", i);
      break;
    }
  }
  
  if (frameStart == -1) {
    Serial.println("未找到帧头0x2C");
    // 尝试其他可能的帧头
    for (int i = 0; i <= length - 12; i++) {
      if (buffer[i] == 0xFF || buffer[i] == 0x42 || buffer[i] == 0x4D) {
        Serial.printf("发现可能的帧头0x%02X在位置%d\n", buffer[i], i);
      }
    }
    return;
  }
  
  // 检查是否有足够的字节
  if (frameStart + 12 > length) {
    Serial.println("帧后数据不足");
    return;
  }
  
  // 提取数据帧
  uint8_t* frame = &buffer[frameStart];
  
  Serial.print("完整帧数据: ");
  for (int i = 0; i < 12; i++) {
    Serial.printf("%02X ", frame[i]);
  }
  Serial.println();
  
  // 校验和计算
  uint8_t checksum = 0;
  for (int i = 0; i < 11; i++) {
    checksum += frame[i];
  }
  checksum = (~checksum) + 1;
  
  Serial.printf("计算校验和: 0x%02X, 接收校验和: 0x%02X ", checksum, frame[11]);
  
  // 验证校验和
  if (frame[11] != checksum) {
    Serial.println("校验和不匹配，但继续解析");
  } else {
    Serial.println("校验和正确");
  }
  
  // 按照协议解析数据
  data.voc_ugm3 = frame[1] * 256 + frame[2];
  data.ch2o_ugm3 = frame[3] * 256 + frame[4];
  data.eco2_ppm = frame[5] * 256 + frame[6];
  
  // 温度解析
  uint16_t temp_raw = frame[7] * 256 + frame[8];
  if (temp_raw > 0x8000) {
    temp_raw = 0x10000 - temp_raw;
    data.temperature_c = -(float)temp_raw * 0.1;
  } else {
    data.temperature_c = (float)temp_raw * 0.1;
  }
  
  // 湿度解析
  uint16_t humidity_raw = frame[9] * 256 + frame[10];
  data.humidity_rh = (float)humidity_raw * 0.1;
  
  Serial.printf("解析结果: VOC=%d, CH2O=%d, eCO2=%d, 温度=%.1f, 湿度=%.1f\n", 
                data.voc_ugm3, data.ch2o_ugm3, data.eco2_ppm, 
                data.temperature_c, data.humidity_rh);
  
  data.valid = true;
#else
  // 如果21VOC禁用，不进行解析
  data.valid = false;
#endif
}

// 验证21VOC传感器数据的合理性
bool SensorManager::validate21VOCData(const TVOCData &data) {
#if ENABLE_21VOC
  if (!data.valid) return false;
  
  if (data.voc_ugm3 > 65535) return false;
  if (data.ch2o_ugm3 > 65535) return false;
  if (data.eco2_ppm > 65535) return false;
  if (data.temperature_c < -40 || data.temperature_c > 80) return false;
  if (data.humidity_rh < 0 || data.humidity_rh > 100) return false;
  
  return true;
#else
  return false;
#endif
}

// 打印21VOC传感器读数
void SensorManager::print21VOCReading(const TVOCData &data) {
#if ENABLE_21VOC
  if (!data.valid) return;
  
  Serial.println("=== 21VOC五合一传感器读数 ===");
  Serial.printf("VOC空气质量: %d μg/m³\n", data.voc_ugm3);
  Serial.printf("甲醛浓度: %d μg/m³\n", data.ch2o_ugm3);
  Serial.printf("eCO2浓度: %d ppm\n", data.eco2_ppm);
  Serial.printf("温度: %.1f°C\n", data.temperature_c);
  Serial.printf("湿度: %.1f%%RH\n", data.humidity_rh);
  
  // 空气质量评级
  Serial.print("VOC等级: ");
  if (data.voc_ugm3 < 65) {
    Serial.println("优秀");
  } else if (data.voc_ugm3 < 220) {
    Serial.println("良好");
  } else if (data.voc_ugm3 < 660) {
    Serial.println("轻度污染");
  } else if (data.voc_ugm3 < 2200) {
    Serial.println("中度污染");
  } else {
    Serial.println("重度污染");
  }
  
  Serial.print("甲醛等级: ");
  if (data.ch2o_ugm3 < 100) {
    Serial.println("优秀");
  } else if (data.ch2o_ugm3 < 300) {
    Serial.println("良好");
  } else if (data.ch2o_ugm3 < 800) {
    Serial.println("轻度污染");
  } else {
    Serial.println("重度污染");
  }
  
  Serial.println("================================");
#endif
}

// MQ135相关方法实现...
void SensorManager::calibrateMQ135() {
#if ENABLE_MQ135
  startCalibration();
#else
  Serial.println("⚠️  MQ135传感器已禁用");
#endif
}

void SensorManager::processMQ135Calibration() {
#if ENABLE_MQ135
  processCalibration();
#endif
}

MQ135CalibrationState SensorManager::getCalibrationState() {
#if ENABLE_MQ135
  return calibrationState;
#else
  return CAL_IDLE;
#endif
}

void SensorManager::deleteCalibration() {
#if ENABLE_MQ135
  deleteRZeroFromPreferences();
#else
  Serial.println("⚠️  MQ135传感器已禁用");
#endif
}

void SensorManager::showCalibration() {
#if ENABLE_MQ135
  float savedRZero = loadRZeroFromPreferences();
  if (savedRZero > 0) {
    Serial.print("当前保存的RZero值: ");
    Serial.println(savedRZero, 2);
  } else {
    Serial.println("未找到保存的校准值");
  }
#else
  Serial.println("⚠️  MQ135传感器已禁用");
#endif
}

void SensorManager::showCalibrationStatus() {
#if ENABLE_MQ135
  Serial.printf("校准状态: %s\n", 
    calibrationState == CAL_IDLE ? "空闲" :
    calibrationState == CAL_PREHEATING ? "预热中" :
    calibrationState == CAL_SAMPLING ? "采样中" : "处理中");
  
  if (calibrationState == CAL_PREHEATING) {
    unsigned long elapsed = millis() - calibrationStartTime;
    unsigned long remaining = PREHEAT_DURATION - elapsed;
    if (remaining > PREHEAT_DURATION) remaining = 0;
    Serial.printf("预热剩余时间: %lu秒\n", remaining/1000);
  } else if (calibrationState == CAL_SAMPLING) {
    Serial.printf("采样进度: %d/%d\n", currentSample, totalSamples);
  }
#else
  Serial.println("⚠️  MQ135传感器已禁用");
#endif
}

void SensorManager::show21VOCData() {
#if ENABLE_21VOC
  if (lastTVOCReading.valid) {
    print21VOCReading(lastTVOCReading);
    Serial.printf("数据时间: %lu ms前\n", millis() - lastTVOCReading.timestamp);
  } else {
    Serial.println("未找到有效的21VOC传感器数据");
  }
#else
  Serial.println("⚠️  21VOC传感器已禁用");
#endif
}
#endif

#if ENABLE_MQ135
// MQ135校准相关私有方法实现
// 开始非阻塞校准过程
void SensorManager::startCalibration() {
#if ENABLE_MQ135
  if (calibrationState != CAL_IDLE) {
    Serial.println("校准已在进行中");
    return;
  }
  
  Serial.println("=== 开始MQ135校准过程 ===");
  Serial.println("注意: MQ135是综合气体传感器，检测空气中的多种气体");
  Serial.println("包括: CO2、氨气、苯、酒精、烟雾等有害气体");
  Serial.println("传感器需要在清洁空气中校准");
  Serial.printf("校准过程需要约%d秒，请确保环境稳定\n", PREHEAT_DURATION/1000 + totalSamples*2);
  Serial.println("在校准期间，系统将继续正常运行其他功能");

  // 创建临时传感器对象用于校准
  if (tempSensorPtr != nullptr) {
    delete tempSensorPtr;
  }
  tempSensorPtr = new MQ135(MQ135_AO_PIN);

  // 初始化校准状态
  calibrationState = CAL_PREHEATING;
  calibrationStartTime = millis();
  calibrationLastUpdate = millis();
  currentSample = 0;
  rZeroSum = 0;
  
  Serial.println("🔥 传感器预热开始...");
  Serial.printf("预热时间: %d秒\n", PREHEAT_DURATION/1000);
#endif
}

// 非阻塞校准处理函数
void SensorManager::processCalibration() {
#if ENABLE_MQ135
  if (calibrationState == CAL_IDLE) {
    return;
  }
  
  unsigned long currentTime = millis();
  
  switch (calibrationState) {
    case CAL_PREHEATING:
      // 每5秒显示一次预热进度
      if (currentTime - calibrationLastUpdate >= 5000) {
        unsigned long elapsed = currentTime - calibrationStartTime;
        unsigned long remaining = PREHEAT_DURATION - elapsed;
        
        if (remaining > PREHEAT_DURATION) remaining = 0;
        
        Serial.printf("🔥 预热进行中... 剩余: %lu秒\n", remaining/1000);
        calibrationLastUpdate = currentTime;
      }
      
      // 检查预热是否完成
      if (currentTime - calibrationStartTime >= PREHEAT_DURATION) {
        Serial.println("✅ 预热完成，开始校准采样...");
        calibrationState = CAL_SAMPLING;
        calibrationLastUpdate = currentTime;
        currentSample = 0;
        rZeroSum = 0;
      }
      break;
      
    case CAL_SAMPLING:
      // 每2秒采集一个样本
      if (currentTime - calibrationLastUpdate >= 2000) {
        if (currentSample < totalSamples) {
          float rzero = tempSensorPtr->getCorrectedRZero(ambient_temperature, ambient_humidity);
          rZeroValues[currentSample] = rzero;
          rZeroSum += rzero;
          currentSample++;
          
          Serial.printf("📊 样本 %d/%d: RZero = %.2f\n", currentSample, totalSamples, rzero);
          calibrationLastUpdate = currentTime;
        } else {
          calibrationState = CAL_COMPLETED;
        }
      }
      break;
      
    case CAL_COMPLETED:
      {
        // 处理校准结果
        calibratedRZero = rZeroSum / totalSamples;
        
        // 计算标准差
        float variance = 0;
        for (int i = 0; i < totalSamples; i++) {
          variance += pow(rZeroValues[i] - calibratedRZero, 2);
        }
        variance /= totalSamples;
        float standardDeviation = sqrt(variance);
        float coefficientOfVariation = (standardDeviation / calibratedRZero) * 100;
      
      Serial.println();
      Serial.println("=== 校准结果 ===");
      Serial.printf("平均RZero值: %.2f\n", calibratedRZero);
      Serial.printf("标准差: %.2f kΩ\n", standardDeviation);
      Serial.printf("变异系数: %.1f%%\n", coefficientOfVariation);
      
      if (coefficientOfVariation > 15.0) {
        Serial.println("⚠️ 校准变异较大，但继续使用");
      }
      
      if (calibratedRZero < 50 || calibratedRZero > 150) {
        Serial.println("⚠️ RZero值超出正常范围，使用标准值");
        calibratedRZero = MANUAL_RZERO_VALUE;
        Serial.printf("使用标准RZero值: %.2f\n", calibratedRZero);
      }
      
      // 创建最终的传感器对象
      if (mq135_sensor_ptr != nullptr) {
        delete mq135_sensor_ptr;
      }
      
      if (USE_MANUAL_RZERO) {
        Serial.println("使用手动设置的标准RZero值: " + String(MANUAL_RZERO_VALUE));
        mq135_sensor_ptr = new MQ135(MQ135_AO_PIN, MANUAL_RZERO_VALUE);
        calibratedRZero = MANUAL_RZERO_VALUE;
      } else {
        mq135_sensor_ptr = new MQ135(MQ135_AO_PIN, calibratedRZero);
      }
      
      mq135_calibrated = true;
      
      // 保存校准值到Preferences
      saveRZeroToPreferences(calibratedRZero);
      
      // 清理临时对象
      if (tempSensorPtr != nullptr) {
        delete tempSensorPtr;
        tempSensorPtr = nullptr;
      }
      
      Serial.println("✅ MQ135传感器校准完成并准备就绪");
      Serial.println("✅ 校准值已保存到Preferences");
      Serial.println("=== MQ135校准流程完成 ===");
      
        // 重置校准状态
        calibrationState = CAL_IDLE;
      }
      break;
      
    default:
      calibrationState = CAL_IDLE;
      if (tempSensorPtr != nullptr) {
        delete tempSensorPtr;
        tempSensorPtr = nullptr;
      }
      break;
  }
#endif
}

// 自动校准HW181-MIC传感器（快速版本）
bool SensorManager::performAutoMicCalibration() {
  Serial.println("=== 开始HW181-MIC自动校准 ===");
  Serial.println("📊 正在采集基线数据，请稍候...");
  
  int totalSum = 0;
  int totalVariation = 0;
  int lastVal = analogRead(MIC_ANALOG_PIN);
  int sampleCount = 0;
  const int AUTO_CALIBRATION_SAMPLES = 300; // 减少到15秒（300*50ms）
  
  unsigned long startTime = millis();
  
  // 采集15秒数据进行快速校准
  while (sampleCount < AUTO_CALIBRATION_SAMPLES) {
    delay(50); // 每50ms采集一次
    int analogVal = analogRead(MIC_ANALOG_PIN);
    
    totalSum += analogVal;
    int variation = abs(analogVal - lastVal);
    totalVariation += variation;
    lastVal = analogVal;
    sampleCount++;
    
    // 每5秒打印一次进度
    if (sampleCount % 100 == 0) {
      int progress = (sampleCount * 100) / AUTO_CALIBRATION_SAMPLES;
      Serial.print("📈 自动校准进度: ");
      Serial.print(progress);
      Serial.println("%");
    }
  }
  
  // 计算校准结果
  mic_baseline_value = totalSum / AUTO_CALIBRATION_SAMPLES;
  int avgVariation = totalVariation / AUTO_CALIBRATION_SAMPLES;
  mic_change_threshold = max(avgVariation * 3, 30);
  
  float baselineVoltage = adcToVoltage(mic_baseline_value);
  
  Serial.println("✅ 自动校准完成!");
  Serial.print("📊 基线值: ");
  Serial.print(mic_baseline_value);
  Serial.print(" (");
  Serial.print(baselineVoltage, 3);
  Serial.println("V)");
  Serial.print("🎚️ 变化阈值: ");
  Serial.println(mic_change_threshold);
  
  // 验证校准结果的合理性
  if (mic_baseline_value < 100 || mic_baseline_value > 4000) {
    Serial.println("⚠️ 校准结果异常，基线值不在合理范围内");
    Serial.println("💡 建议检查传感器连接或使用 'mic_cal' 进行手动校准");
    return false;
  }
  
  // 保存校准数据
  saveMicCalibrationData(mic_baseline_value, mic_change_threshold);
  mic_calibrated = true;
  
  Serial.println("💾 校准数据已保存到Flash存储器");
  Serial.println("🔊 传感器现在可以正常检测声音分贝");
  
  return true;
}

// Preferences相关方法实现
bool SensorManager::initPreferences() {
#if ENABLE_MQ135
  Serial.println("=== 初始化Preferences ===");
  
  bool result = preferences.begin("mq135", false);
  
  if (!result) {
    Serial.println("❌ Preferences初始化失败");
    return false;
  }
  
  Serial.println("✅ Preferences初始化成功");
  return true;
#else
  return true;
#endif
}

void SensorManager::saveRZeroToPreferences(float rzero) {
#if ENABLE_MQ135
  Serial.println("=== 保存RZero值到Preferences ===");
  
  size_t result1 = preferences.putFloat("rzero", rzero);
  size_t result2 = preferences.putUInt("timestamp", getCurrentTimestamp());
  size_t result3 = preferences.putFloat("temperature", ambient_temperature);
  size_t result4 = preferences.putFloat("humidity", ambient_humidity);
  size_t result5 = preferences.putString("version", "1.0");
  
  if (result1 > 0 && result2 > 0) {
    Serial.println("✅ RZero值保存成功");
    Serial.print("保存的RZero值: ");
    Serial.println(rzero, 2);
  } else {
    Serial.println("❌ 保存失败");
  }
#endif
}

float SensorManager::loadRZeroFromPreferences() {
#if ENABLE_MQ135
  if (!preferences.isKey("rzero")) {
    return -1.0;
  }
  
  float rzero = preferences.getFloat("rzero", -1.0);
  uint32_t timestamp = preferences.getUInt("timestamp", 0);
  
  if (rzero <= 0) {
    return -1.0;
  }
  
  if (timestamp > 0) {
    uint32_t currentTime = getCurrentTimestamp();
    uint32_t ageInDays = (currentTime - timestamp) / (24 * 3600);
    
    if (ageInDays > 30) {
      Serial.println("⚠️ 校准数据较旧(>30天)，建议重新校准");
    }
  }
  
  return rzero;
#else
  return -1.0;
#endif
}

void SensorManager::deleteRZeroFromPreferences() {
#if ENABLE_MQ135
  Serial.println("=== 删除保存的RZero值 ===");
  
  bool result = true;
  
  if (preferences.isKey("rzero")) {
    result &= preferences.remove("rzero");
  }
  if (preferences.isKey("timestamp")) {
    result &= preferences.remove("timestamp");
  }
  if (preferences.isKey("temperature")) {
    result &= preferences.remove("temperature");
  }
  if (preferences.isKey("humidity")) {
    result &= preferences.remove("humidity");
  }
  if (preferences.isKey("version")) {
    result &= preferences.remove("version");
  }
  
  if (result) {
    Serial.println("✅ 校准数据删除成功");
  } else {
    Serial.println("❌ 校准数据删除失败");
  }
#endif
}
#endif

// 调试21VOC传感器
void SensorManager::debug21VOCSensor() {
#if ENABLE_21VOC
  Serial.println("=== 21VOC传感器调试模式 ===");
  
  Serial.printf("UART配置: RX=%d, TX=%d, 波特率=%d\n", UART_RX_PIN, UART_TX_PIN, UART_BAUD_RATE);
  
  if (sensorSerial) {
    Serial.println("✅ UART已初始化");
  } else {
    Serial.println("❌ UART未初始化");
    return;
  }
  
  // 清空缓冲区
  int discarded = 0;
  while (sensorSerial.available()) {
    sensorSerial.read();
    discarded++;
  }
  if (discarded > 0) {
    Serial.printf("清空了%d字节旧数据\n", discarded);
  }
  
  // 连续读取10秒的原始数据
  Serial.println("开始10秒原始数据监控...");
  unsigned long startTime = millis();
  int totalBytes = 0;
  
  while (millis() - startTime < 10000) {
    if (sensorSerial.available()) {
      uint8_t byte = sensorSerial.read();
      Serial.printf("%02X ", byte);
      totalBytes++;
      
      if (totalBytes % 16 == 0) {
        Serial.println();
      }
    }
    delay(1);
  }
  
  Serial.println();
  Serial.printf("10秒内总共接收到 %d 字节数据\n", totalBytes);
  
  if (totalBytes == 0) {
    Serial.println("❌ 没有接收到任何数据");
    Serial.println("可能的问题:");
    Serial.println("1. 传感器未连接或未上电");
    Serial.println("2. RX/TX引脚接线错误");
    Serial.println("3. 波特率不匹配");
    Serial.println("4. 传感器损坏");
  } else if (totalBytes < 12) {
    Serial.println("⚠️ 数据量较少，可能是间歇性问题");
  } else {
    Serial.println("✅ 接收到数据，检查数据格式");
  }
  
  Serial.println("=== 调试完成 ===");
#else
  Serial.println("⚠️  21VOC传感器已禁用");
#endif
}

// 测试21VOC传感器连接
void SensorManager::test21VOCConnection() {
#if ENABLE_21VOC
  Serial.println("=== 21VOC传感器连接测试 ===");
  
  // 测试不同波特率
  int baudRates[] = {9600, 4800, 19200, 38400, 115200};
  int numRates = sizeof(baudRates) / sizeof(baudRates[0]);
  
  for (int i = 0; i < numRates; i++) {
    Serial.printf("测试波特率: %d\n", baudRates[i]);
    
    // 重新初始化UART
    sensorSerial.end();
    delay(100);
    sensorSerial.begin(baudRates[i], SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    delay(500);
    
    // 清空缓冲区
    while (sensorSerial.available()) {
      sensorSerial.read();
    }
    
    // 监控2秒
    unsigned long testStart = millis();
    int bytesReceived = 0;
    bool hasValidFrame = false;
    
    while (millis() - testStart < 2000) {
      if (sensorSerial.available()) {
        uint8_t byte = sensorSerial.read();
        bytesReceived++;
        
        if (byte == 0x2C || byte == 0xFF || byte == 0x42 || byte == 0x4D) {
          hasValidFrame = true;
        }
      }
      delay(1);
    }
    
    Serial.printf("  接收字节: %d", bytesReceived);
    if (hasValidFrame) {
      Serial.println(" ✅ 发现可能的帧头");
    } else {
      Serial.println();
    }
    
    if (bytesReceived > 10 && hasValidFrame) {
      Serial.printf("  🎯 波特率 %d 可能正确！\n", baudRates[i]);
      break;
    }
  }
  
  // 恢复原始波特率
  sensorSerial.end();
  delay(100);
  sensorSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  delay(500);
  
  Serial.println("=== 连接测试完成 ===");
#else
  Serial.println("⚠️  21VOC传感器已禁用");
#endif
}

// ===== 获取环境参数方法实现 =====
float SensorManager::getAmbientTemperature() const {
#if ENABLE_MQ135
  return ambient_temperature;
#else
  return 25.0; // 默认温度
#endif
}

float SensorManager::getAmbientHumidity() const {
#if ENABLE_MQ135
  return ambient_humidity;
#else
  return 50.0; // 默认湿度
#endif
}

bool SensorManager::isMQ135Calibrated() const {
#if ENABLE_MQ135
  return mq135_calibrated;
#else
  return false;
#endif
}

const TVOCData& SensorManager::getLastTVOCReading() const {
#if ENABLE_21VOC
  return lastTVOCReading;
#else
  static TVOCData emptyData = {0, 0, 0, 0.0, 0.0, false, 0};
  return emptyData;
#endif
}

const MicData& SensorManager::getLastMicReading() const {
#if ENABLE_HW181_MIC
  return lastMicReading;
#else
  static MicData emptyData = {0.0, 0.0, 0, false, 0.0, 0.0, 0.0, false, false, 0};
  return emptyData;
#endif
}

#if ENABLE_HW181_MIC
// HW181-MIC传感器初始化
bool SensorManager::initHW181MIC() {
  Serial.println("=== 初始化HW181-MIC分贝检测模块 ===");
  Serial.println("引脚配置: GPIO" + String(MIC_ANALOG_PIN) + " (模拟输入)");
  
  // 加载校准数据
  loadMicCalibrationData();
  
  if (!mic_calibrated) {
    Serial.println("⚠️ HW181-MIC传感器未校准，开始自动校准...");
    Serial.println("💡 请保持环境相对安静，校准将自动进行");
    
    // 执行自动校准
    if (performAutoMicCalibration()) {
      Serial.println("🎉 HW181-MIC传感器自动校准成功！");
    } else {
      Serial.println("❌ HW181-MIC传感器自动校准失败，可使用 'mic_cal' 手动校准");
    }
  } else {
    Serial.println("✅ HW181-MIC传感器已校准，基线值: " + String(mic_baseline_value));
  }
  
  // 初始化数据结构
  lastMicReading.valid = false;
  lastMicReading.calibrated = mic_calibrated;
  lastMicReading.timestamp = millis();
  
  return true;
}

// 读取HW181-MIC传感器数据
bool SensorManager::readHW181MICSensor(MicData &data) {
  // 读取模拟值
  int analogValue = analogRead(MIC_ANALOG_PIN);
  float voltage = adcToVoltage(analogValue);
  
  // 计算分贝值
  float currentDb = calculateDecibels(analogValue, voltage);
  float smoothedDb = smoothDecibels(currentDb, last_db_value, DB_SMOOTH_FACTOR);
  last_db_value = smoothedDb;
  
  // 使用改进的声音检测算法
  bool soundDetected = detectSoundImproved(analogValue);
  
  if (soundDetected) {
    sound_detected_count++;
  }
  total_readings++;
  
  // 更新统计
  updateDecibelStatistics(smoothedDb);
  
  // 填充数据结构
  data.decibels = smoothedDb;
  data.sound_voltage = voltage;
  data.analog_value = analogValue;
  data.sound_detected = soundDetected;
  data.min_db = min_db_in_period;
  data.avg_db = (db_sample_count > 0) ? (avg_db_sum / db_sample_count) : MIN_DB;
  data.calibrated = mic_calibrated;
  data.valid = true;
  data.timestamp = millis();
  
  // 保存到内部变量
  lastMicReading = data;
  last_analog_value = analogValue;
  
  return true;
}

// ADC值转电压
float SensorManager::adcToVoltage(int adcValue) {
  return (adcValue * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
}

// 计算分贝值
float SensorManager::calculateDecibels(int analogValue, float voltage) {
  if (voltage < VOLTAGE_THRESHOLD) voltage = VOLTAGE_THRESHOLD;
  
  // 基于校准基线的分贝计算
  float baselineVoltage = adcToVoltage(mic_baseline_value);
  if (baselineVoltage < VOLTAGE_THRESHOLD) baselineVoltage = VOLTAGE_THRESHOLD;
  
  // 使用电压比值计算分贝增量
  float voltageRatio = voltage / baselineVoltage;
  float dbFromVoltage = DB_BASELINE + 20.0 * log10(voltageRatio);
  
  // 基于模拟值变化的线性映射
  float analogRatio = (float)analogValue / (float)max(mic_baseline_value, 1);
  float dbFromAnalog = DB_BASELINE + DB_SENSITIVITY * (analogRatio - 1.0);
  
  // 基于信号变化量的快速响应
  int currentChange = abs(analogValue - last_analog_value);
  float changeBoost = 0;
  if (currentChange > mic_change_threshold) {
    changeBoost = 10.0 * (float)currentChange / (float)max(mic_change_threshold, 1);
    if (changeBoost > 30.0) changeBoost = 30.0;
  }
  
  // 结合计算结果
  float calculatedDb = dbFromVoltage * 0.4 + dbFromAnalog * 0.4 + changeBoost * 0.2;
  
  // 限制范围
  if (calculatedDb < MIN_DB) calculatedDb = MIN_DB;
  if (calculatedDb > MAX_DB) calculatedDb = MAX_DB;
  
  return calculatedDb;
}

// 平滑分贝值
float SensorManager::smoothDecibels(float currentDb, float lastDb, float smoothFactor) {
  float dbDifference = abs(currentDb - lastDb);
  
  if (dbDifference > 5.0) {
    // 大变化时，使用更高的响应度
    smoothFactor = min(smoothFactor + 0.3, 1.0);
  } else if (dbDifference < 1.0) {
    // 小变化时，使用更多平滑
    smoothFactor = smoothFactor * 0.5;
  }
  
  return lastDb + smoothFactor * (currentDb - lastDb);
}

// 更新分贝统计
void SensorManager::updateDecibelStatistics(float dbValue) {
  if (dbValue < min_db_in_period) min_db_in_period = dbValue;
  
  avg_db_sum += dbValue;
  db_sample_count++;
}

// 执行MIC校准
bool SensorManager::performMicCalibration() {
  Serial.println("=== 开始HW181-MIC校准 ===");
  Serial.println("请保持环境安静，校准将持续60秒...");
  delay(3000);
  
  int totalSum = 0;
  int totalVariation = 0;
  int lastVal = analogRead(MIC_ANALOG_PIN);
  int sampleCount = 0;
  
  unsigned long startTime = millis();
  unsigned long nextPrintTime = startTime + 10000;
  
  // 采集60秒数据
  while (sampleCount < CALIBRATION_SAMPLES) {
    delay(50); // 每50ms采集一次
    int analogVal = analogRead(MIC_ANALOG_PIN);
    
    totalSum += analogVal;
    int variation = abs(analogVal - lastVal);
    totalVariation += variation;
    lastVal = analogVal;
    sampleCount++;
    
    // 打印进度
    unsigned long currentTime = millis();
    if (currentTime >= nextPrintTime) {
      int progress = (sampleCount * 100) / CALIBRATION_SAMPLES;
      Serial.print("校准进度: ");
      Serial.print(progress);
      Serial.println("%");
      nextPrintTime += 10000;
    }
  }
  
  // 计算校准结果
  mic_baseline_value = totalSum / CALIBRATION_SAMPLES;
  int avgVariation = totalVariation / CALIBRATION_SAMPLES;
  mic_change_threshold = max(avgVariation * 3, 30);
  
  float baselineVoltage = adcToVoltage(mic_baseline_value);
  
  Serial.println("✅ 校准完成!");
  Serial.print("基线值: ");
  Serial.print(mic_baseline_value);
  Serial.print(" (");
  Serial.print(baselineVoltage, 3);
  Serial.println("V)");
  Serial.print("变化阈值: ");
  Serial.println(mic_change_threshold);
  
  // 保存校准数据
  saveMicCalibrationData(mic_baseline_value, mic_change_threshold);
  mic_calibrated = true;
  
  return true;
}

// 加载MIC校准数据
void SensorManager::loadMicCalibrationData() {
  Serial.println("=== 加载HW181-MIC校准数据 ===");
  
  Preferences micPrefs;
  bool dataLoaded = false;
  
  // 首先尝试从专用命名空间加载
  if (micPrefs.begin("hw181_calib", true)) {
    Serial.println("📁 专用命名空间访问成功");
    
    if (micPrefs.isKey("mic_baseline") && micPrefs.isKey("mic_threshold")) {
      mic_baseline_value = micPrefs.getInt("mic_baseline", 0);
      mic_change_threshold = micPrefs.getInt("mic_threshold", 50);
      mic_calibrated = true;
      dataLoaded = true;
      
      Serial.println("📁 从专用命名空间加载校准数据成功");
      Serial.printf("基线值: %d, 阈值: %d\n", mic_baseline_value, mic_change_threshold);
    } else {
      Serial.println("📁 专用命名空间中未发现校准数据");
    }
    micPrefs.end();
  } else {
    Serial.println("📁 无法访问专用命名空间");
  }
  
  // 如果专用命名空间没有数据，尝试从默认命名空间加载
  if (!dataLoaded && micPrefs.begin("nvs", true)) {
    Serial.println("📁 尝试从默认命名空间加载...");
    
    if (micPrefs.isKey("hw181_baseline") && micPrefs.isKey("hw181_threshold")) {
      mic_baseline_value = micPrefs.getInt("hw181_baseline", 0);
      mic_change_threshold = micPrefs.getInt("hw181_threshold", 50);
      mic_calibrated = true;
      dataLoaded = true;
      
      Serial.println("📁 从默认命名空间加载校准数据成功");
      Serial.printf("基线值: %d, 阈值: %d\n", mic_baseline_value, mic_change_threshold);
    } else {
      Serial.println("📁 默认命名空间中也未发现校准数据");
    }
    micPrefs.end();
  }
  
  if (!dataLoaded) {
    Serial.println("📁 未发现任何HW181-MIC校准数据");
    mic_calibrated = false;
    mic_baseline_value = 0;
    mic_change_threshold = 50;
  }
}

// 保存MIC校准数据
void SensorManager::saveMicCalibrationData(int baseline, int threshold) {
  Serial.println("=== 保存HW181-MIC校准数据 ===");
  
  // 先尝试初始化Preferences
  Preferences micPrefs;
  if (micPrefs.begin("hw181_calib", false)) {
    Serial.println("📁 Preferences命名空间初始化成功");
    
    size_t result1 = micPrefs.putInt("mic_baseline", baseline);
    size_t result2 = micPrefs.putInt("mic_threshold", threshold);
    
    micPrefs.end();
    
    if (result1 > 0 && result2 > 0) {
      Serial.println("✅ HW181-MIC校准数据已保存");
      Serial.printf("保存的基线值: %d\n", baseline);
      Serial.printf("保存的阈值: %d\n", threshold);
    } else {
      Serial.println("❌ 数据写入失败");
      Serial.printf("基线值写入结果: %d字节\n", result1);
      Serial.printf("阈值写入结果: %d字节\n", result2);
    }
  } else {
    Serial.println("❌ 无法初始化Preferences命名空间");
    Serial.println("💡 可能的原因:");
    Serial.println("   1. Flash存储空间不足");
    Serial.println("   2. 命名空间名称冲突");
    Serial.println("   3. NVS分区损坏");
    
    // 尝试使用默认命名空间
    Serial.println("🔧 尝试使用默认命名空间...");
    if (micPrefs.begin("nvs", false)) {
      size_t result1 = micPrefs.putInt("hw181_baseline", baseline);
      size_t result2 = micPrefs.putInt("hw181_threshold", threshold);
      micPrefs.end();
      
      if (result1 > 0 && result2 > 0) {
        Serial.println("✅ 使用默认命名空间保存成功");
      } else {
        Serial.println("❌ 默认命名空间也保存失败");
      }
    } else {
      Serial.println("❌ 默认命名空间也无法初始化");
    }
  }
}

// 公共方法实现
void SensorManager::calibrateHW181MIC() {
  performMicCalibration();
}

void SensorManager::showMicData() {
  if (lastMicReading.valid) {
    Serial.println("=== HW181-MIC传感器数据 ===");
    Serial.print("分贝值: ");
    Serial.print(lastMicReading.decibels, 1);
    Serial.println(" dB");
    Serial.print("电压: ");
    Serial.print(lastMicReading.sound_voltage, 3);
    Serial.println(" V");
    Serial.print("模拟值: ");
    Serial.println(lastMicReading.analog_value);
    Serial.print("声音检测: ");
    Serial.println(lastMicReading.sound_detected ? "是" : "否");
    Serial.print("校准状态: ");
    Serial.println(lastMicReading.calibrated ? "已校准" : "未校准");
    
    if (total_readings > 0) {
      Serial.print("检测率: ");
      Serial.print((float)sound_detected_count / total_readings * 100, 1);
      Serial.println("%");
    }
    Serial.println("========================");
  } else {
    Serial.println("⚠️ 没有有效的HW181-MIC数据");
  }
}

void SensorManager::showMicCalibrationStatus() {
  Serial.print("HW181-MIC校准状态: ");
  Serial.println(mic_calibrated ? "已校准" : "未校准");
  
  if (mic_calibrated) {
    Serial.print("基线值: ");
    Serial.print(mic_baseline_value);
    Serial.print(" (");
    Serial.print(adcToVoltage(mic_baseline_value), 3);
    Serial.println("V)");
    Serial.print("变化阈值: ");
    Serial.println(mic_change_threshold);
  } else {
    Serial.println("请使用 'mic_cal' 命令进行校准");
  }
}

void SensorManager::deleteMicCalibration() {
  Serial.println("=== 删除HW181-MIC校准数据 ===");
  
  Preferences micPrefs;
  bool deleted = false;
  
  // 从专用命名空间删除
  if (micPrefs.begin("hw181_calib", false)) {
    if (micPrefs.isKey("mic_baseline")) {
      micPrefs.remove("mic_baseline");
      deleted = true;
    }
    if (micPrefs.isKey("mic_threshold")) {
      micPrefs.remove("mic_threshold");
      deleted = true;
    }
    micPrefs.end();
    
    if (deleted) {
      Serial.println("✅ 从专用命名空间删除校准数据成功");
    }
  }
  
  // 从默认命名空间删除
  if (micPrefs.begin("nvs", false)) {
    if (micPrefs.isKey("hw181_baseline")) {
      micPrefs.remove("hw181_baseline");
      deleted = true;
    }
    if (micPrefs.isKey("hw181_threshold")) {
      micPrefs.remove("hw181_threshold");
      deleted = true;
    }
    micPrefs.end();
    
    if (deleted) {
      Serial.println("✅ 从默认命名空间删除校准数据成功");
    }
  }
  
  if (deleted) {
    Serial.println("✅ HW181-MIC校准数据已删除");
  } else {
    Serial.println("⚠️ 未找到要删除的校准数据");
  }
  
  mic_calibrated = false;
  mic_baseline_value = 0;
  mic_change_threshold = 50;
  
  // 重置声音检测算法状态
  recent_values_index = 0;
  consecutive_sound_count = 0;
  baseline_moving_avg = 0.0;
  for (int i = 0; i < SOUND_DETECTION_WINDOW; i++) {
    recent_analog_values[i] = 0;
  }
}

// HW181-MIC改进的声音检测算法
bool SensorManager::detectSoundImproved(int current_analog_value) {
  if (!mic_calibrated) {
    return false;  // 未校准时不检测声音
  }
  
  // 更新最近的读数缓冲区
  updateRecentValues(current_analog_value);
  
  // 计算移动平均值
  float current_moving_avg = calculateMovingAverage();
  
  // 第一次运行时初始化基线移动平均
  if (baseline_moving_avg == 0.0) {
    baseline_moving_avg = current_moving_avg;
    return false;
  }
  
  // 缓慢更新基线（适应环境噪声变化）
  baseline_moving_avg = baseline_moving_avg * 0.95 + current_moving_avg * 0.05;
  
  // 计算相对于基线的变化
  float deviation = abs(current_moving_avg - baseline_moving_avg);
  
  // 动态阈值：基于校准时设定的阈值，但考虑移动平均
  float dynamic_threshold = mic_change_threshold * 0.7;  // 降低阈值敏感度
  
  // 检测是否超过阈值 - 简化版本：一次超过就算检测到声音
  bool current_detection = (deviation > dynamic_threshold);
  
  // 直接返回当前检测结果，不需要连续检测
  return current_detection;
}

// 更新最近读数的循环缓冲区
void SensorManager::updateRecentValues(int value) {
  recent_analog_values[recent_values_index] = value;
  recent_values_index = (recent_values_index + 1) % SOUND_DETECTION_WINDOW;
}

// 计算最近几次读数的移动平均
float SensorManager::calculateMovingAverage() {
  float sum = 0;
  for (int i = 0; i < SOUND_DETECTION_WINDOW; i++) {
    sum += recent_analog_values[i];
  }
  return sum / SOUND_DETECTION_WINDOW;
}
#endif
