# 传感器配置指南

本项目支持通过编译参数自定义启用哪些传感器，实现功能裁剪和优化资源使用。

## 支持的传感器

| 传感器名称 | 宏定义 | 功能描述 | 接口类型 |
|-----------|--------|----------|----------|
| BMP280 | `ENABLE_BMP280` | 气压、温度传感器 | I2C |
| MQ135 | `ENABLE_MQ135` | 空气质量传感器 | ADC |
| 21VOC | `ENABLE_21VOC` | 五合一传感器(VOC、甲醛、eCO2、温湿度) | UART |

## 配置方法

### 方法1: 修改 platformio.ini 文件

在 `platformio.ini` 文件的 `build_flags` 中添加或修改宏定义：

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino

; 启用所有传感器（默认配置）
build_flags = 
    -DENABLE_BMP280=1
    -DENABLE_MQ135=1
    -DENABLE_21VOC=1

; 或者只启用部分传感器
; build_flags = 
;     -DENABLE_BMP280=1
;     -DENABLE_MQ135=0
;     -DENABLE_21VOC=1
```

### 方法2: 修改 sensors.h 文件

直接在 `src/sensors.h` 文件顶部修改默认值：

```cpp
// 传感器编译参数定义
#ifndef ENABLE_BMP280
#define ENABLE_BMP280 1  // 设置为0禁用BMP280
#endif

#ifndef ENABLE_MQ135
#define ENABLE_MQ135 1   // 设置为0禁用MQ135
#endif

#ifndef ENABLE_21VOC
#define ENABLE_21VOC 1   // 设置为0禁用21VOC
#endif
```

## 常用配置组合

### 最小配置（仅BMP280）
```ini
build_flags = 
    -DENABLE_BMP280=1
    -DENABLE_MQ135=0
    -DENABLE_21VOC=0
```
- 适用场景：只需要基本的温度和气压监测
- 资源占用：最小内存和Flash使用
- 硬件需求：只需要BMP280传感器

### 空气质量监测配置
```ini
build_flags = 
    -DENABLE_BMP280=1
    -DENABLE_MQ135=1
    -DENABLE_21VOC=0
```
- 适用场景：温度、气压和基本空气质量监测
- 资源占用：中等内存使用
- 硬件需求：BMP280 + MQ135传感器

### 完整环境监测配置（推荐）
```ini
build_flags = 
    -DENABLE_BMP280=1
    -DENABLE_MQ135=1
    -DENABLE_21VOC=1
```
- 适用场景：全面的环境监测
- 资源占用：最大内存和Flash使用
- 硬件需求：所有三种传感器

### 高精度VOC监测配置
```ini
build_flags = 
    -DENABLE_BMP280=0
    -DENABLE_MQ135=0
    -DENABLE_21VOC=1
```
- 适用场景：专业VOC、甲醛、eCO2监测
- 资源占用：中等内存使用
- 硬件需求：只需要21VOC传感器

## 编译和验证

1. **清理构建缓存**（推荐）
   ```bash
   pio run -t clean
   ```

2. **重新编译**
   ```bash
   pio run
   ```

3. **验证配置**
   - 查看编译日志中的传感器初始化信息
   - 通过串口监视器查看启动时的传感器状态
   - 禁用的传感器会显示"⚠️ XXX传感器已禁用"

## 代码结构说明

### 条件编译机制

项目使用条件编译技术，确保：
- 禁用的传感器相关代码不会被编译
- 相关头文件不会被包含
- 对应的类成员变量和方法不会占用内存
- 硬件初始化跳过禁用的传感器

### 影响的文件

- `src/sensors.h` - 传感器类定义
- `src/sensors.cpp` - 传感器实现
- `src/main.cpp` - 主程序（调用传感器接口）

### 自动适配

当某个传感器被禁用时：
- 相关的初始化方法会返回成功但不执行实际操作
- 数据读取方法会返回默认值或跳过相关部分
- 调试命令会显示传感器已禁用的提示

## 性能优化

### 内存优化
禁用不需要的传感器可以：
- 减少RAM使用（传感器对象、缓冲区等）
- 减少Flash使用（代码段、常量等）
- 降低CPU负载（减少数据处理）

### 建议配置

- **开发阶段**：启用所有传感器，便于测试和调试
- **生产部署**：根据实际需求禁用不必要的传感器
- **低功耗应用**：最小化传感器配置

## 故障排除

### 编译错误
1. 确保所有宏定义使用正确的语法：`-DENABLE_XXX=1`
2. 清理构建缓存后重新编译
3. 检查PlatformIO配置文件语法

### 运行时问题
1. 检查串口输出的传感器初始化信息
2. 确认硬件连接与启用的传感器匹配
3. 验证传感器初始化顺序

### 常见问题
- **Q**: 为什么禁用传感器后还能看到相关代码？
- **A**: 请确保重新编译并清理了构建缓存

- **Q**: 如何确认配置是否生效？
- **A**: 查看串口输出，禁用的传感器会显示"已禁用"信息

## 扩展开发

如需添加新的传感器：
1. 在 `sensors.h` 中添加对应的 `ENABLE_XXX` 宏定义
2. 使用条件编译包装相关代码
3. 更新本配置文档

这种设计模式使项目具有良好的可扩展性和维护性。
