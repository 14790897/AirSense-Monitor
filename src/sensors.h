#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

// ===== 传感器启用配置 =====
// 用户可以通过修改这些宏定义来选择启用哪些传感器
// 设置为 1 启用，设置为 0 禁用

#ifndef ENABLE_BMP280
#define ENABLE_BMP280 1        // BMP280气压温度传感器 (I2C)
#endif

#ifndef ENABLE_21VOC
#define ENABLE_21VOC 1         // 21VOC五合一传感器 (UART)
#endif

#ifndef ENABLE_MQ135
#define ENABLE_MQ135 1         // MQ135空气质量传感器 (ADC)
#endif

#ifndef ENABLE_HW181_MIC
#define ENABLE_HW181_MIC 1     // HW181-MIC分贝检测模块 (ADC)
#endif

// ===== 条件包含头文件 =====
#if ENABLE_BMP280
#include <Adafruit_BMP280.h>
#endif

#if ENABLE_MQ135
#include <MQ135.h>
#endif

#if ENABLE_21VOC
#include <HardwareSerial.h>
#endif

// ===== 传感器数据结构定义 =====

// 21VOC传感器数据结构（即使禁用也需要定义用于接口兼容）
struct TVOCData {
  uint16_t voc_ugm3;      // VOC空气质量 (ug/m3)
  uint16_t ch2o_ugm3;     // 甲醛浓度 (ug/m3)
  uint16_t eco2_ppm;      // eCO2浓度 (ppm)
  float temperature_c;    // 温度 (°C)
  float humidity_rh;      // 湿度 (%RH)
  bool valid;             // 数据有效性
  unsigned long timestamp; // 时间戳
};

// HW181-MIC传感器数据结构（即使禁用也需要定义用于接口兼容）
struct MicData {
  float decibels;          // 分贝值 (dB)
  float sound_voltage;           // 电压值 (V)
  int analog_value;        // 模拟值 (0-4095)
  bool sound_detected;     // 是否检测到声音
  float min_db;            // 期间最小分贝
  float avg_db;            // 期间平均分贝
  bool calibrated;         // 是否已校准
  bool valid;              // 数据有效性
  unsigned long timestamp; // 时间戳
};

// 传感器数据汇总结构
struct SensorData {
#if ENABLE_BMP280
  // BMP280数据
  float temperature;
  float pressure;
  float altitude;
#endif
  
#if ENABLE_MQ135
  // MQ135数据
  float air_quality_ppm;
  float air_quality_corrected_ppm;
  bool mq135_calibrated;
#endif
  
#if ENABLE_21VOC
  // 21VOC数据
  TVOCData tvoc_data;
#endif

#if ENABLE_HW181_MIC
  // HW181-MIC数据
  MicData mic_data;
#endif
  
  // 时间戳
  unsigned long timestamp;
};

// MQ135校准状态（即使禁用也需要定义用于接口兼容）
enum MQ135CalibrationState {
  CAL_IDLE,           // 空闲状态
  CAL_PREHEATING,     // 预热中
  CAL_SAMPLING,       // 采样中
  CAL_COMPLETED       // 校准完成
};

// 传感器类
class SensorManager {
private:
#if ENABLE_BMP280
  // BMP280传感器
  Adafruit_BMP280 bmp;
#endif
  
#if ENABLE_21VOC
  // 21VOC传感器配置
  static const int UART_RX_PIN = 1;
  static const int UART_TX_PIN = 0;
  static const int UART_BAUD_RATE = 9600;
  HardwareSerial sensorSerial;
  
  // 21VOC传感器数据
  TVOCData lastTVOCReading;
  unsigned long lastTVOCReadTime;
  static const unsigned long TVOC_READ_INTERVAL = 2000;
#endif
  
#if ENABLE_MQ135
  // MQ135传感器配置
  static const int MQ135_AO_PIN = 2;
  static const bool USE_MANUAL_RZERO = false;
  static const float MANUAL_RZERO_VALUE;
  MQ135 *mq135_sensor_ptr;
  
  // 环境参数
  float ambient_temperature;
  float ambient_humidity;
  float calibratedRZero;
  bool mq135_calibrated;
  
  // MQ135校准状态管理
  MQ135CalibrationState calibrationState;
  unsigned long calibrationStartTime;
  unsigned long calibrationLastUpdate;
  static const unsigned long PREHEAT_DURATION = 30000;
  int currentSample;
  static const int totalSamples = 10;
  float rZeroSum;
  float rZeroValues[10];
  MQ135 *tempSensorPtr;
  
  // Preferences对象
  Preferences preferences;
#endif

#if ENABLE_HW181_MIC
  // HW181-MIC传感器配置
  static const int MIC_ANALOG_PIN = 3;
  static constexpr float ADC_REF_VOLTAGE = 3.3;
  static const int ADC_RESOLUTION = 4096;
  static constexpr float MIN_DB = 30.0;
  static constexpr float MAX_DB = 120.0;
  static constexpr float DB_BASELINE = 35.0;
  static constexpr float DB_SENSITIVITY = 25.0;
  static constexpr float VOLTAGE_THRESHOLD = 0.01;
  static constexpr float DB_SMOOTH_FACTOR = 0.7;
  static const int CALIBRATION_SAMPLES = 1200;
  
  // HW181-MIC传感器数据和状态
  MicData lastMicReading;
  bool mic_calibrated;
  int mic_baseline_value;
  int mic_change_threshold;
  float last_db_value;
  int last_analog_value;
  
  // 声音检测改进算法相关
  static const int SOUND_DETECTION_WINDOW = 5;  // 检测窗口大小
  int recent_analog_values[5];  // 最近几次读数（避免动态分配）
  int recent_values_index;  // 循环缓冲区索引
  int consecutive_sound_count;  // 连续检测到声音的次数
  static const int MIN_CONSECUTIVE_DETECTIONS = 3;  // 至少连续检测到3次才认为有声音
  float baseline_moving_avg;  // 基线移动平均
  
  // 分贝统计
  float min_db_in_period;
  float avg_db_sum;
  int db_sample_count;
  unsigned long sound_detected_count;
  unsigned long total_readings;
#endif
  
  // 内部方法
#if ENABLE_MQ135
  bool initPreferences();
  void saveRZeroToPreferences(float rzero);
  float loadRZeroFromPreferences();
  void deleteRZeroFromPreferences();
  
  // MQ135方法
  void startCalibration();
  void processCalibration();
#endif
  
#if ENABLE_21VOC
  // 21VOC传感器方法
  bool read21VOCSensor(TVOCData &data);
  void parse21VOCData(uint8_t* buffer, int length, TVOCData &data);
  bool validate21VOCData(const TVOCData &data);
  void print21VOCReading(const TVOCData &data);
#endif

#if ENABLE_HW181_MIC
  // HW181-MIC传感器方法
  bool readHW181MICSensor(MicData &data);
  float adcToVoltage(int adcValue);
  float calculateDecibels(int analogValue, float voltage);
  float smoothDecibels(float currentDb, float lastDb, float smoothFactor);
  void updateDecibelStatistics(float dbValue);
  bool performMicCalibration();
  bool performAutoMicCalibration();  // 自动校准方法
  void loadMicCalibrationData();
  void saveMicCalibrationData(int baseline, int threshold);
#endif
  
public:
  SensorManager();
  ~SensorManager();
  
  // 初始化方法
  bool initBMP280();
  bool init21VOCSensor();
  bool initMQ135();
  bool initHW181MIC();
  bool initAllSensors();
  
  // 数据读取方法
  SensorData readAllSensors();
  void update21VOCData(); // 定时更新21VOC数据
  
  // MQ135校准相关
  void calibrateMQ135();
  void processMQ135Calibration();
  MQ135CalibrationState getCalibrationState();
  void deleteCalibration();
  void showCalibration();
  void showCalibrationStatus();
  
  // 21VOC调试相关
  void debug21VOCSensor();
  void test21VOCConnection();
  void show21VOCData();
  
  // HW181-MIC相关
  void calibrateHW181MIC();
  void showMicData();
  void showMicCalibrationStatus();
  void deleteMicCalibration();
  
private:
  // HW181-MIC声音检测算法辅助函数
  bool detectSoundImproved(int current_analog_value);
  void updateRecentValues(int value);
  float calculateMovingAverage();
  
public:
  
  // 获取环境参数
  float getAmbientTemperature() const;
  float getAmbientHumidity() const;
  bool isMQ135Calibrated() const;
  const TVOCData& getLastTVOCReading() const;
  const MicData& getLastMicReading() const;
};

#endif // SENSORS_H
