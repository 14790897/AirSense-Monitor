# OneNET数据上传null值修复总结

## 问题描述

OneNET平台返回错误码2409，提示"required value:identifier:temperature"，原因是系统上传了null值的传感器数据。

### 错误示例
```json
{
  "id":"477136",
  "version":"1.0",
  "params":{
    "temperature":{"value":null,"time":1753199575000},
    "pressure":{"value":null,"time":1753199575000},
    "altitude":{"value":null,"time":1753199575000},
    "voc_ugm3":{"value":6,"time":1753199575000},
    "ch2o_ugm3":{"value":0,"time":1753199575000},
    "eco2_ppm":{"value":406,"time":1753199575000}
  }
}
```

### OneNET平台回执
```json
{
  "id":"477136",
  "code":2409,
  "msg":"required value:identifier:temperature"
}
```

## 解决方案

修改MQTT管理器的`publishSensorData`函数，添加数据有效性检查，只上传有效的非null数据。

## 修改内容

### 1. BMP280传感器数据检查

**修改前**:
```cpp
doc["params"]["temperature"]["value"] = data.temperature;
doc["params"]["pressure"]["value"] = data.pressure;
doc["params"]["altitude"]["value"] = data.altitude;
```

**修改后**:
```cpp
// 只添加有效的BMP280数据（非NaN和非null）
if (!isnan(data.temperature) && data.temperature != 0.0) {
  doc["params"]["temperature"]["value"] = data.temperature;
  doc["params"]["temperature"]["time"] = timestamp;
}

if (!isnan(data.pressure) && data.pressure != 0.0) {
  doc["params"]["pressure"]["value"] = data.pressure;
  doc["params"]["pressure"]["time"] = timestamp;
}

if (!isnan(data.altitude) && data.altitude != 0.0) {
  doc["params"]["altitude"]["value"] = data.altitude;
  doc["params"]["altitude"]["time"] = timestamp;
}
```

### 2. 21VOC传感器温湿度数据检查

**修改前**:
```cpp
doc["params"]["temperature"]["value"] = data.tvoc_data.temperature_c;
doc["params"]["humidity_rh"]["value"] = data.tvoc_data.humidity_rh;
```

**修改后**:
```cpp
// 使用21VOC的温湿度数据覆盖BMP280的温度（如果可用且有效）
if (!isnan(data.tvoc_data.temperature_c) && data.tvoc_data.temperature_c != 0.0) {
  doc["params"]["temperature"]["value"] = data.tvoc_data.temperature_c;
  doc["params"]["temperature"]["time"] = timestamp;
}

if (!isnan(data.tvoc_data.humidity_rh) && data.tvoc_data.humidity_rh != 0.0) {
  doc["params"]["humidity_rh"]["value"] = data.tvoc_data.humidity_rh;
  doc["params"]["humidity_rh"]["time"] = timestamp;
}
```

### 3. MQ135传感器数据检查

**修改前**:
```cpp
doc["params"]["air_quality_ppm"]["value"] = (int)round(data.air_quality_ppm);
doc["params"]["air_quality_corrected_ppm"]["value"] = (int)round(data.air_quality_corrected_ppm);
```

**修改后**:
```cpp
if (!isnan(data.air_quality_ppm) && data.air_quality_ppm > 0.0) {
  doc["params"]["air_quality_ppm"]["value"] = (int)round(data.air_quality_ppm);
  doc["params"]["air_quality_ppm"]["time"] = timestamp;
}

if (!isnan(data.air_quality_corrected_ppm) && data.air_quality_corrected_ppm > 0.0) {
  doc["params"]["air_quality_corrected_ppm"]["value"] = (int)round(data.air_quality_corrected_ppm);
  doc["params"]["air_quality_corrected_ppm"]["time"] = timestamp;
}
```

### 4. HW181-MIC传感器数据检查

**修改前**:
```cpp
doc["params"]["decibels"]["value"] = data.mic_data.decibels;
doc["params"]["sound_voltage"]["value"] = data.mic_data.sound_voltage;
```

**修改后**:
```cpp
if (!isnan(data.mic_data.decibels) && data.mic_data.decibels > 0.0) {
  doc["params"]["decibels"]["value"] = data.mic_data.decibels;
  doc["params"]["decibels"]["time"] = timestamp;
}

if (!isnan(data.mic_data.sound_voltage) && data.mic_data.sound_voltage >= 0.0) {
  doc["params"]["sound_voltage"]["value"] = data.mic_data.sound_voltage;
  doc["params"]["sound_voltage"]["time"] = timestamp;
}
```

## 数据有效性检查标准

### 浮点数数据
- 使用`!isnan(value)`检查是否为有效数字
- 根据传感器特性设置合理的最小值阈值

### 整数数据
- VOC相关数据：已通过`valid`标志检查
- 布尔数据：sound_detected始终有效

### 检查条件
1. **温度数据**: `!isnan(temperature) && temperature != 0.0`
2. **压力数据**: `!isnan(pressure) && pressure != 0.0`
3. **高度数据**: `!isnan(altitude) && altitude != 0.0`
4. **分贝数据**: `!isnan(decibels) && decibels > 0.0`
5. **电压数据**: `!isnan(voltage) && voltage >= 0.0`
6. **PPM数据**: `!isnan(ppm) && ppm > 0.0`

## 预期效果

### 修复前的JSON（包含null值）
```json
{
  "params":{
    "temperature":{"value":null,"time":1753199575000},
    "pressure":{"value":null,"time":1753199575000},
    "voc_ugm3":{"value":6,"time":1753199575000}
  }
}
```

### 修复后的JSON（只包含有效数据）
```json
{
  "params":{
    "voc_ugm3":{"value":6,"time":1753199575000},
    "ch2o_ugm3":{"value":0,"time":1753199575000},
    "eco2_ppm":{"value":406,"time":1753199575000}
  }
}
```

## 编译结果

- **RAM使用**: 12.1% (39,764 bytes)
- **Flash使用**: 63.6% (833,630 bytes)
- **编译状态**: ✅ 成功

## 技术说明

1. **数据过滤**: 只上传有效的传感器数据，避免null值导致的平台错误
2. **兼容性**: 保持与现有OneNET物模型的完全兼容
3. **性能**: 减少无效数据传输，提高上传效率
4. **可靠性**: 避免因null值导致的数据上传失败

## 测试验证

1. **BMP280未连接**: 不会上传temperature、pressure、altitude字段
2. **VOC传感器正常**: 正常上传voc_ugm3、ch2o_ugm3、eco2_ppm字段
3. **MQ135未校准**: 不会上传air_quality相关字段
4. **MIC未校准**: 不会上传decibels、sound_voltage字段

## 使用方法

1. **编译上传**:
   ```bash
   pio run --target upload
   pio run --target monitor
   ```

2. **验证效果**:
   - 查看串口输出的JSON数据
   - 确认OneNET平台不再返回2409错误
   - 验证只有有效数据被上传

现在系统只会上传有效的传感器数据，避免了null值导致的OneNET平台错误，提高了数据上传的可靠性。
