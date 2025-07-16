/*
 * 五合一烟感/空气质量温湿度传感器模组串口协议
 * 
 * 通信协议：
 * - 波特率：9600
 * - 数据位：8位
 * - 停止位：1位  
 * - 校验位：无
 * 
 * 数据包格式（12字节）：
 * [0]    码头：固定值0x2C
 * [1-2]  VOC空气质量（μg/m³）：Data[1]*256 + Data[2]
 * [3-4]  甲醛（μg/m³）：Data[3]*256 + Data[4]
 * [5-6]  eCO₂（ppm）：Data[5]*256 + Data[6]
 * [7-8]  温度（0.1°C）：Data[7]*256 + Data[8]
 * [9-10] 湿度（0.1%RH）：Data[9]*256 + Data[10]
 * [11]   校验和：前11个字节累加后的和，取反加1
 * 
 * 注意：
 * - 温度负数判断：若值>0x8000则为负温度，计算方式：-(0xFFFF - 原始值) * 0.1
 * - 建议使用外部标准温湿度计进行温湿度补偿
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include "nvs_flash.h"

// UART配置
#define UART_RX_PIN 1  // UART接收引脚 (GPIO1)
#define UART_TX_PIN 0  // UART发送引脚 (GPIO0)
#define UART_BAUD_RATE 9600  // 波特率，根据传感器规格调整

// 创建串口对象
HardwareSerial sensorSerial(1);  // 使用UART1

// TVOC传感器数据结构 - 五合一传感器
struct TVOCData {
  uint16_t voc_ugm3;      // VOC空气质量 (ug/m3)
  uint16_t ch2o_ugm3;     // 甲醛浓度 (ug/m3)
  uint16_t eco2_ppm;      // eCO2浓度 (ppm)
  float temperature_c;    // 温度 (°C)
  float humidity_rh;      // 湿度 (%RH)
  bool valid;             // 数据有效性
  unsigned long timestamp; // 时间戳
};

// 全局变量
TVOCData lastReading;
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 2000; // 2秒读取间隔，五合一传感器

// 存储系统
Preferences preferences;

// 统计数据 - 五合一传感器
struct TVOCStats {
  float avgVOC = 0;
  float maxVOC = 0;
  float minVOC = 99999;
  float avgCH2O = 0;
  float maxCH2O = 0;
  float minCH2O = 99999;
  float avgECO2 = 0;
  float maxECO2 = 0;
  float minECO2 = 99999;
  float avgTemp = 0;
  float maxTemp = -99;
  float minTemp = 99;
  float avgHumidity = 0;
  float maxHumidity = 0;
  float minHumidity = 100;
  uint32_t readingCount = 0;
  uint32_t validReadings = 0;
  uint32_t errorCount = 0;
};

TVOCStats stats;

// 函数声明
void initNVS();
void initUART();
bool readTVOCSensor(TVOCData &data);
void parseTVOCData(uint8_t* buffer, int length, TVOCData &data);
void updateStatistics(const TVOCData &data);
void printStatistics();
void printTVOCReading(const TVOCData &data);
bool validateTVOCData(const TVOCData &data);
void sendSensorCommand(uint8_t* command, int length);

void setup() {
  // 初始化串口
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("======================================");
  Serial.println("UART 五合一传感器数据读取程序");
  Serial.println("(VOC/甲醛/eCO2/温度/湿度)");
  Serial.println("======================================");
  Serial.println("传感器连接信息:");
  Serial.printf("- RX引脚: GPIO%d\n", UART_RX_PIN);
  Serial.printf("- TX引脚: GPIO%d\n", UART_TX_PIN);
  Serial.printf("- 波特率: %d\n", UART_BAUD_RATE);
  Serial.println("- VCC: 3.3V/5V (根据传感器规格)");
  Serial.println("- GND: GND");
  Serial.println("======================================");
  Serial.println("⚠️  温湿度补偿说明:");
  Serial.println("由于传感器安装在PCB模组上，受模块本体热影响，");
  Serial.println("建议使用外部标准温湿度计进行补偿。");
  Serial.println("如有温湿度差异，请手动进行差值修正。");
  Serial.println("======================================");

  // 初始化NVS
  initNVS();

  // 初始化UART
  initUART();

  // 初始化统计数据
  lastReading.valid = false;
  lastReading.timestamp = 0;

  Serial.println("初始化完成，开始读取TVOC传感器数据...");
  Serial.println();

  delay(2000); // 等待传感器稳定
}

void loop() {
  unsigned long currentTime = millis();

  // 定时读取传感器数据
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;

    Serial.println("========================================");
    Serial.printf("⏰ [%lu ms] 开始读取TVOC传感器数据...\n", currentTime);
    
    // 检查UART缓冲区状态
    int availableBytes = sensorSerial.available();
    Serial.printf("📡 UART缓冲区可用字节数: %d\n", availableBytes);

    TVOCData currentReading;
    
    // 读取传感器数据
    if (readTVOCSensor(currentReading)) {
      lastReading = currentReading;
      
      if (currentReading.valid) {
        updateStatistics(currentReading);
        printTVOCReading(currentReading);
        stats.validReadings++;
      } else {
        Serial.println("⚠️ 接收到数据但解析失败");
      }
    } else {
      stats.errorCount++;
      Serial.println("❌ 未接收到任何数据");
      
      // 检查连接状态
      if (!sensorSerial) {
        Serial.println("🔌 UART连接异常，尝试重新初始化...");
        initUART();
      }
    }

    stats.readingCount++;

    // 每5次读取输出统计信息
    if (stats.readingCount % 5 == 0) {
      Serial.println();
      printStatistics();
      Serial.println();
    }
    
    Serial.println("========================================");
    Serial.println();
  }

  delay(50); // 增加延迟，减少串口干扰
}

// 初始化NVS存储系统
void initNVS() {
  Serial.println("正在初始化NVS存储系统...");

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    Serial.println("NVS分区损坏，正在重新初始化...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }

  if (err == ESP_OK) {
    Serial.println("✅ NVS存储系统初始化成功");
  } else {
    Serial.printf("❌ NVS存储系统初始化失败，错误代码: %d\n", err);
  }
}

// 初始化UART通信
void initUART() {
  Serial.println("正在初始化UART通信...");
  
  // 配置UART参数: 波特率, 数据位, 停止位, 校验位
  sensorSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  // 等待串口稳定
  delay(100);
  
  if (sensorSerial) {
    Serial.println("✅ UART初始化成功");
    
    // 清空接收缓冲区
    while (sensorSerial.available()) {
      sensorSerial.read();
    }
  } else {
    Serial.println("❌ UART初始化失败");
  }
}

// 读取TVOC传感器数据
bool readTVOCSensor(TVOCData &data) {
  // 直接读取传感器连续输出的数据
  uint8_t buffer[64];
  int bytesRead = 0;
  unsigned long startTime = millis();
  
  // 等待数据到达，超时2秒
  while (bytesRead < sizeof(buffer) && (millis() - startTime) < 2000) {
    if (sensorSerial.available()) {
      buffer[bytesRead] = sensorSerial.read();
      bytesRead++;
      
      // 检查是否有换行符或特定结束符
      if (buffer[bytesRead-1] == '\n' || buffer[bytesRead-1] == '\r') {
        break;
      }
      
      // 如果连续没有新数据超过200ms，认为一帧数据接收完成
      unsigned long lastByteTime = millis();
      while (!sensorSerial.available() && (millis() - lastByteTime) < 200) {
        delay(10);
      }
      if (!sensorSerial.available()) {
        break;
      }
    }
    delay(10);
  }

  if (bytesRead > 0) {
    // 打印原始数据用于调试
    Serial.print("📊 原始数据 (");
    Serial.print(bytesRead);
    Serial.print(" 字节): ");
    
    // 以十六进制格式打印
    Serial.print("HEX: ");
    for (int i = 0; i < bytesRead; i++) {
      if (buffer[i] < 0x10) Serial.print("0");
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    
    // 以ASCII格式打印（如果可显示）
    Serial.print("| ASCII: ");
    for (int i = 0; i < bytesRead; i++) {
      if (buffer[i] >= 32 && buffer[i] <= 126) {
        Serial.print((char)buffer[i]);
      } else {
        Serial.print(".");
      }
    }
    Serial.println();
    
    // 解析接收到的数据
    parseTVOCData(buffer, bytesRead, data);
    data.timestamp = millis();
    return true;
  }

  return false;
}

// 解析五合一传感器数据 (12字节)
void parseTVOCData(uint8_t* buffer, int length, TVOCData &data) {
  // 初始化数据结构
  data.voc_ugm3 = 0;
  data.ch2o_ugm3 = 0;
  data.eco2_ppm = 0;
  data.temperature_c = 0.0;
  data.humidity_rh = 0.0;
  data.valid = false;

  Serial.print("🔍 数据分析: ");
  
  // 检查数据帧长度 (五合一传感器需要12字节)
  if (length < 12) {
    Serial.printf("数据长度不足 (%d < 12字节)\n", length);
    return;
  }
  
  // 寻找正确的数据帧起始位置 (0x2C开头)
  int frameStart = -1;
  for (int i = 0; i <= length - 12; i++) {
    if (buffer[i] == 0x2C) {
      frameStart = i;
      break;
    }
  }
  
  if (frameStart == -1) {
    Serial.println("未找到有效的数据帧头 (0x2C)");
    return;
  }
  
  // 检查是否有足够的字节
  if (frameStart + 12 > length) {
    Serial.printf("数据帧不完整，需要%d字节，实际只有%d字节\n", frameStart + 12, length);
    return;
  }
  
  Serial.printf("找到数据帧起始位置: %d\n", frameStart);
  
  // 提取数据帧
  uint8_t* frame = &buffer[frameStart];
  
  // 校验和计算 (前11个字节累加的和取反+1)
  uint8_t checksum = 0;
  for (int i = 0; i < 11; i++) {
    checksum += frame[i];
  }
  checksum = (~checksum) + 1;
  
  Serial.printf("📋 五合一传感器数据帧解析 (协议: Data[n]*256 + Data[n+1]):\n");
  Serial.printf("  [0] 码头: 0x%02X (应为0x2C)\n", frame[0]);
  Serial.printf("  [1-2] VOC: 0x%02X 0x%02X -> %d ug/m³\n", frame[1], frame[2], frame[1] * 256 + frame[2]);
  Serial.printf("  [3-4] 甲醛: 0x%02X 0x%02X -> %d ug/m³\n", frame[3], frame[4], frame[3] * 256 + frame[4]);
  Serial.printf("  [5-6] eCO2: 0x%02X 0x%02X -> %d ppm\n", frame[5], frame[6], frame[5] * 256 + frame[6]);
  Serial.printf("  [7-8] 温度: 0x%02X 0x%02X -> 原始值: %d\n", frame[7], frame[8], frame[7] * 256 + frame[8]);
  Serial.printf("  [9-10] 湿度: 0x%02X 0x%02X -> 原始值: %d\n", frame[9], frame[10], frame[9] * 256 + frame[10]);
  Serial.printf("  [11] 校验和: 0x%02X (计算值: 0x%02X)\n", frame[11], checksum);
  
  // 验证校验和
  if (frame[11] != checksum) {
    Serial.printf("⚠️ 校验和不匹配，但继续解析数据 (接收: 0x%02X, 计算: 0x%02X)\n", frame[11], checksum);
  }
  
  // 按照协议解析数据
  // VOC空气质量 (ug/m3): Data[1]*256 + Data[2]
  data.voc_ugm3 = frame[1] * 256 + frame[2];
  
  // 甲醛 (ug/m3): Data[3]*256 + Data[4]
  data.ch2o_ugm3 = frame[3] * 256 + frame[4];
  
  // eCO2 (ppm): Data[5]*256 + Data[6]
  data.eco2_ppm = frame[5] * 256 + frame[6];
  
  // 温度 (0.1°C): Data[7]*256 + Data[8]
  uint16_t temp_raw = frame[7] * 256 + frame[8];
  if (temp_raw > 0x8000) {
    // 负温度: 0xFFFF - temp_raw
    temp_raw = 0xFFFF - temp_raw;
    data.temperature_c = -(float)temp_raw * 0.1;
  } else {
    // 正温度
    data.temperature_c = (float)temp_raw * 0.1;
  }
  
  // 湿度 (0.1%RH): Data[9]*256 + Data[10]
  uint16_t humidity_raw = frame[9] * 256 + frame[10];
  data.humidity_rh = (float)humidity_raw * 0.1;
  
  data.valid = true;
  
  Serial.printf("✅ 解析成功:\n");
  Serial.printf("  VOC: %d ug/m³\n", data.voc_ugm3);
  Serial.printf("  甲醛: %d ug/m³\n", data.ch2o_ugm3);
  Serial.printf("  eCO2: %d ppm\n", data.eco2_ppm);
  Serial.printf("  温度: %.1f°C\n", data.temperature_c);
  Serial.printf("  湿度: %.1f%%RH\n", data.humidity_rh);
}

// 发送传感器命令
void sendSensorCommand(uint8_t* command, int length) {
  for (int i = 0; i < length; i++) {
    sensorSerial.write(command[i]);
  }
  sensorSerial.flush(); // 确保数据发送完成
}

// 验证五合一传感器数据的合理性
bool validateTVOCData(const TVOCData &data) {
  if (!data.valid) return false;
  
  // 检查数据范围的合理性（根据协议文档调整）
  // VOC范围: 0-65535 ug/m³ (16位数据)
  // 甲醛范围: 0-65535 ug/m³ (16位数据)
  // eCO2范围: 400-65535 ppm (16位数据)
  // 温度范围: -40~80°C
  // 湿度范围: 0-100%RH
  if (data.voc_ugm3 > 65535) return false;
  if (data.ch2o_ugm3 > 65535) return false;
  if (data.eco2_ppm > 65535) return false;
  if (data.temperature_c < -40 || data.temperature_c > 80) return false;
  if (data.humidity_rh < 0 || data.humidity_rh > 100) return false;
  
  return true;
}

// 更新五合一传感器统计数据
void updateStatistics(const TVOCData &data) {
  if (!data.valid) return;

  float voc = data.voc_ugm3;
  float ch2o = data.ch2o_ugm3;
  float eco2 = data.eco2_ppm;
  float temp = data.temperature_c;
  float humidity = data.humidity_rh;

  // 更新VOC统计
  stats.avgVOC = (stats.avgVOC * stats.validReadings + voc) / (stats.validReadings + 1);
  if (voc > stats.maxVOC) stats.maxVOC = voc;
  if (voc < stats.minVOC) stats.minVOC = voc;

  // 更新甲醛统计
  stats.avgCH2O = (stats.avgCH2O * stats.validReadings + ch2o) / (stats.validReadings + 1);
  if (ch2o > stats.maxCH2O) stats.maxCH2O = ch2o;
  if (ch2o < stats.minCH2O) stats.minCH2O = ch2o;

  // 更新eCO2统计
  stats.avgECO2 = (stats.avgECO2 * stats.validReadings + eco2) / (stats.validReadings + 1);
  if (eco2 > stats.maxECO2) stats.maxECO2 = eco2;
  if (eco2 < stats.minECO2) stats.minECO2 = eco2;

  // 更新温度统计
  stats.avgTemp = (stats.avgTemp * stats.validReadings + temp) / (stats.validReadings + 1);
  if (temp > stats.maxTemp) stats.maxTemp = temp;
  if (temp < stats.minTemp) stats.minTemp = temp;

  // 更新湿度统计
  stats.avgHumidity = (stats.avgHumidity * stats.validReadings + humidity) / (stats.validReadings + 1);
  if (humidity > stats.maxHumidity) stats.maxHumidity = humidity;
  if (humidity < stats.minHumidity) stats.minHumidity = humidity;
}

// 打印五合一传感器读取结果
void printTVOCReading(const TVOCData &data) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print("ms] ");

  if (data.valid) {
    Serial.print("✅ VOC: ");
    Serial.print(data.voc_ugm3);
    Serial.print(" ug/m³ | 甲醛: ");
    Serial.print(data.ch2o_ugm3);
    Serial.print(" ug/m³ | eCO2: ");
    Serial.print(data.eco2_ppm);
    Serial.print(" ppm | 温度: ");
    Serial.print(data.temperature_c, 1);
    Serial.print("°C | 湿度: ");
    Serial.print(data.humidity_rh, 1);
    Serial.print("%RH");

    // 空气质量评估
    String vocQuality = "优秀";
    if (data.voc_ugm3 > 1000) vocQuality = "差";
    else if (data.voc_ugm3 > 300) vocQuality = "一般";
    else if (data.voc_ugm3 > 150) vocQuality = "良好";
    
    String ch2oQuality = "优秀";
    if (data.ch2o_ugm3 > 100) ch2oQuality = "差";
    else if (data.ch2o_ugm3 > 50) ch2oQuality = "一般";
    else if (data.ch2o_ugm3 > 20) ch2oQuality = "良好";
    
    String eco2Quality = "优秀";
    if (data.eco2_ppm > 2000) eco2Quality = "差";
    else if (data.eco2_ppm > 1000) eco2Quality = "一般";
    else if (data.eco2_ppm > 600) eco2Quality = "良好";
    
    String tempQuality = "适宜";
    if (data.temperature_c > 30 || data.temperature_c < 18) tempQuality = "不适";
    else if (data.temperature_c > 26 || data.temperature_c < 20) tempQuality = "一般";
    
    String humidityQuality = "适宜";
    if (data.humidity_rh > 70 || data.humidity_rh < 30) humidityQuality = "不适";
    else if (data.humidity_rh > 60 || data.humidity_rh < 40) humidityQuality = "一般";
    
    Serial.print(" | 质量: VOC-");
    Serial.print(vocQuality);
    Serial.print(" 甲醛-");
    Serial.print(ch2oQuality);
    Serial.print(" eCO2-");
    Serial.print(eco2Quality);
    Serial.print(" 温度-");
    Serial.print(tempQuality);
    Serial.print(" 湿度-");
    Serial.print(humidityQuality);
    
  } else {
    Serial.print("❌ 无效数据");
  }
  
  Serial.println();
}

// 打印五合一传感器统计信息
void printStatistics() {
  Serial.println("=== 五合一传感器统计信息 ===");
  Serial.printf("总读取次数: %d\n", stats.readingCount);
  Serial.printf("有效读取: %d (%.1f%%)\n", 
    stats.validReadings, 
    (float)stats.validReadings / stats.readingCount * 100);
  Serial.printf("错误次数: %d\n", stats.errorCount);
  
  if (stats.validReadings > 0) {
    Serial.println("\nVOC空气质量数据 (ug/m³):");
    Serial.printf("  平均值: %.1f | 最大值: %.1f | 最小值: %.1f\n", 
      stats.avgVOC, stats.maxVOC, stats.minVOC);
    
    Serial.println("甲醛数据 (ug/m³):");
    Serial.printf("  平均值: %.1f | 最大值: %.1f | 最小值: %.1f\n", 
      stats.avgCH2O, stats.maxCH2O, stats.minCH2O);
      
    Serial.println("eCO2数据 (ppm):");
    Serial.printf("  平均值: %.1f | 最大值: %.1f | 最小值: %.1f\n", 
      stats.avgECO2, stats.maxECO2, stats.minECO2);

    Serial.println("温度数据 (°C):");
    Serial.printf("  平均值: %.1f | 最大值: %.1f | 最小值: %.1f\n", 
      stats.avgTemp, stats.maxTemp, stats.minTemp);

    Serial.println("湿度数据 (%RH):");
    Serial.printf("  平均值: %.1f | 最大值: %.1f | 最小值: %.1f\n", 
      stats.avgHumidity, stats.maxHumidity, stats.minHumidity);

    // 综合环境质量评估
    String overallQuality = "优秀";
    if (stats.avgVOC > 1000 || stats.avgCH2O > 100 || stats.avgECO2 > 2000) {
      overallQuality = "差";
    } else if (stats.avgVOC > 300 || stats.avgCH2O > 50 || stats.avgECO2 > 1000) {
      overallQuality = "一般";  
    } else if (stats.avgVOC > 150 || stats.avgCH2O > 20 || stats.avgECO2 > 600) {
      overallQuality = "良好";
    }
    
    String tempComfort = "舒适";
    if (stats.avgTemp > 30 || stats.avgTemp < 18) tempComfort = "不适";
    else if (stats.avgTemp > 26 || stats.avgTemp < 20) tempComfort = "一般";
    
    String humidityComfort = "舒适";
    if (stats.avgHumidity > 70 || stats.avgHumidity < 30) humidityComfort = "不适";
    else if (stats.avgHumidity > 60 || stats.avgHumidity < 40) humidityComfort = "一般";
    
    Serial.printf("综合空气质量: %s\n", overallQuality.c_str());
    Serial.printf("温度舒适度: %s\n", tempComfort.c_str());
    Serial.printf("湿度舒适度: %s\n", humidityComfort.c_str());
  }
  
  Serial.println("========================");
}