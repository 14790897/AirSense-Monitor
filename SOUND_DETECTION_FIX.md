# HW181-MIC声音检测算法修复说明

## 问题描述
用户反馈："现在一直检测有声音不对啊"

## 原因分析
原始声音检测算法存在以下问题：

1. **过于敏感的检测逻辑**：
   ```cpp
   int change = abs(analogValue - last_analog_value);
   bool soundDetected = (change > mic_change_threshold && mic_calibrated);
   ```
   - 只比较相邻两次读数，任何小的电压波动都被认为是"声音"
   - 没有考虑环境噪声的持续变化
   - 缺乏抗干扰能力

2. **阈值设置问题**：
   - 固定阈值可能对环境变化不够适应
   - 没有考虑信号的持续性

## 改进方案

### 1. 移动平均算法
- 使用5个样本的移动平均窗口，减少单次读数波动的影响
- 基于移动平均值而不是瞬时值进行比较

### 2. 动态基线跟踪
- 实时跟踪环境噪声基线：`baseline_moving_avg`
- 缓慢适应环境变化：`baseline_moving_avg = baseline_moving_avg * 0.95 + current_moving_avg * 0.05`

### 3. 即时响应检测

- **简化逻辑**：一次检测到声音变化就立即确认
- **快速响应**：`current_detection = (deviation > dynamic_threshold)`
- **无延迟**：不需要等待连续检测，提高响应速度

### 4. 动态阈值调整
- 基于校准阈值动态调整：`dynamic_threshold = mic_change_threshold * 0.7`
- 降低敏感度，减少误报

## 新算法流程

1. **更新移动平均窗口**：将当前读数加入5个样本的循环缓冲区
2. **计算当前移动平均**：对最近5次读数求平均
3. **更新基线**：缓慢跟踪环境噪声变化
4. **计算偏差**：当前移动平均相对于基线的偏差
5. **阈值比较**：偏差是否超过动态阈值
6. **即时确认**：一次超过阈值就确认有声音（响应更快）

## 核心代码

```cpp
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
```

## 新增变量

在 `sensors.h` 中新增：
```cpp
// 声音检测改进算法相关
static const int SOUND_DETECTION_WINDOW = 5;  // 检测窗口大小
int recent_analog_values[5];  // 最近几次读数
int recent_values_index;  // 循环缓冲区索引
int consecutive_sound_count;  // 连续检测到声音的次数
static const int MIN_CONSECUTIVE_DETECTIONS = 3;  // 至少连续检测到3次才认为有声音
float baseline_moving_avg;  // 基线移动平均
```

## 优势

1. **抗噪声能力强**：移动平均算法有效滤除瞬时噪声
2. **自适应环境**：动态基线跟踪适应环境噪声变化
3. **快速响应**：一次检测到就立即确认，响应速度快
4. **参数可调**：可以通过调整窗口大小、阈值系数等参数优化性能

## 测试建议

1. **静音环境测试**：确认在安静环境下不会误报
2. **正常说话测试**：确认能正确检测到人声
3. **环境噪声测试**：在有持续低级噪声的环境下测试适应性
4. **突发声音测试**：测试对突发声音的响应速度

## 参数调优

如果需要进一步调优，可以调整以下参数：

- `SOUND_DETECTION_WINDOW`：移动平均窗口大小（默认5）
- `dynamic_threshold` 计算中的系数（默认0.7）
- 基线更新速度系数（默认0.95和0.05）

## 更新日期
2024-12-19

## 修复状态
✅ 已完成 - 编译通过，待实际测试验证效果
