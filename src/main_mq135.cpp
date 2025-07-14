#include <Arduino.h>
#include <MQ135.h>
#include <math.h>

// MQ135传感器引脚定义
#define MQ135_AO_PIN 2 // 模拟输出引脚

// 环境参数（用于温湿度补偿）
float temperature = 31.0; // 理想校准温度 31°C
float humidity = 60.0;    // 理想校准湿度 33%

// 是否使用手动RZero值（如果校准结果不理想）
#define USE_MANUAL_RZERO false
#define MANUAL_RZERO_VALUE 76.63 // 标准RZero值

// MQ135库对象 - 先用默认值初始化，校准后重新创建
MQ135 *mq135_sensor_ptr = nullptr;

// 校准模式标志
bool calibrationMode = true;
float calibratedRZero = 0.0;

// 打印PPM含义解释
void explainPPMCorrection()
{
    Serial.println("=== PPM校正说明 ===");
    Serial.println("1. PPM(标准): 基于标准条件(20°C, 33%湿度)的CO2浓度");
    Serial.println("   - 这是传感器在理想条件下的读数");
    Serial.println("   - 不考虑当前环境温湿度的影响");
    Serial.println();
    Serial.println("2. PPM(校正): 根据当前温湿度修正后的CO2浓度");
    Serial.println("   - 考虑了温度和湿度对传感器的影响");
    Serial.println("   - 更接近真实的CO2浓度值");
    Serial.println("   - 实际应用中推荐使用这个值");
    Serial.println();
    Serial.print("当前环境设置: 温度=");
    Serial.print(temperature);
    Serial.print("°C, 湿度=");
    Serial.print(humidity);
    Serial.println("%");
    Serial.println("注意: 如果环境参数不准确，校正值可能比标准值更不准确");
    Serial.println("==================");
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== MQ135 空气质量传感器 ===");
    Serial.println("根据数据手册，传感器需要：");
    Serial.println("1. 在清洁空气中放置12-24小时进行老化");
    Serial.println("2. 在20°C和33%湿度下校准");
    Serial.println("3. 多次读取平均值以减少噪声");
    Serial.println();

    Serial.println("传感器预热中，请等待30秒...");
    // 传感器预热
    for (int i = 3600; i > 0; i--)
    {
        Serial.print("剩余: ");
        Serial.print(i);
        Serial.println("s");
        delay(1000);
    }

    // 校准过程 - 先用默认参数创建传感器对象进行校准
    Serial.println();
    Serial.println("=== 开始校准过程 ===");
    Serial.println("请确保传感器在清洁空气中!");
    Serial.println("正在进行多次读取以获得准确的RZERO值...");

    // 创建临时传感器对象用于校准
    MQ135 tempSensor(MQ135_AO_PIN);

    // 多次读取求平均值
    float rZeroSum = 0;
    float rZeroSumSquared = 0; // 用于计算方差
    int samples = 30;
    float rZeroValues[30]; // 存储所有样本值

    for (int i = 0; i < samples; i++)
    {
        float rzero = tempSensor.getCorrectedRZero(temperature, humidity);
        rZeroValues[i] = rzero; // 保存样本值
        rZeroSum += rzero;
        rZeroSumSquared += rzero * rzero;

        Serial.print("样本 ");
        Serial.print(i + 1);
        Serial.print("/");
        Serial.print(samples);
        Serial.print(": RZero = ");
        Serial.println(rzero, 2);
        delay(1000);
    }

    calibratedRZero = rZeroSum / samples;

    // 计算方差和标准差
    float variance = (rZeroSumSquared / samples) - (calibratedRZero * calibratedRZero);
    float standardDeviation = sqrt(variance);
    float coefficientOfVariation = (standardDeviation / calibratedRZero) * 100; // 变异系数(%)

    Serial.println();
    Serial.print("校准完成! 平均RZero值: ");
    Serial.println(calibratedRZero, 2);
    Serial.print("标准差: ");
    Serial.print(standardDeviation, 2);
    Serial.println(" kΩ");
    Serial.print("变异系数: ");
    Serial.print(coefficientOfVariation, 1);
    Serial.println("%");

    // 方差检查 - 如果变异系数超过10%则报错
    bool calibrationValid = true;
    if (coefficientOfVariation > 10.0)
    {
        Serial.println();
        Serial.println("错误: 校准方差过大!");
        Serial.println("变异系数超过10%，测量不稳定");
        Serial.println("可能原因:");
        Serial.println("1. 环境不稳定(温度、气流变化)");
        Serial.println("2. 传感器未充分预热");
        Serial.println("3. 电路连接不稳定");
        Serial.println("4. 电源电压不稳定");
        calibrationValid = false;
    }

    // 显示最大最小值范围
    float minRZero = rZeroValues[0];
    float maxRZero = rZeroValues[0];
    for (int i = 1; i < samples; i++)
    {
        if (rZeroValues[i] < minRZero)
            minRZero = rZeroValues[i];
        if (rZeroValues[i] > maxRZero)
            maxRZero = rZeroValues[i];
    }
    Serial.print("RZero范围: ");
    Serial.print(minRZero, 2);
    Serial.print(" - ");
    Serial.print(maxRZero, 2);
    Serial.println(" kΩ");

    if (!calibrationValid)
    {
        Serial.println();
        Serial.println("建议: 重新校准或检查硬件连接");
        Serial.println("继续使用可能导致测量不准确");

        // 可选：停止程序或使用标准值
        Serial.println();
        Serial.println("选择处理方式:");
        Serial.println("1. 使用当前平均值继续 (可能不准确)");
        Serial.println("2. 使用标准RZero值 (76.63kΩ)");
        Serial.println("默认使用标准值以确保基本功能...");

        calibratedRZero = MANUAL_RZERO_VALUE;
        Serial.print("已切换到标准RZero值: ");
        Serial.println(calibratedRZero, 2);
    }

    // 校准结果诊断
    Serial.println("=== 校准诊断 ===");
    if (calibratedRZero < 50)
    {
        Serial.println("警告: RZero值偏低 (<50kΩ)");
        Serial.println("可能原因:");
        Serial.println("1. 负载电阻不是10kΩ");
        Serial.println("2. 电路连接问题");
        Serial.println("3. 传感器加热器电压不足(需要5V)");
        Serial.println("4. 环境不是清洁空气");
    }
    else if (calibratedRZero > 100)
    {
        Serial.println("警告: RZero值偏高 (>100kΩ)");
        Serial.println("可能传感器需要更长时间老化");
    }
    else
    {
        Serial.println("RZero值正常范围 (50-100kΩ)");
    }

    // 检查当前ADC值
    int currentADC = analogRead(MQ135_AO_PIN);
    float currentVoltage = (currentADC / 4095.0) * 3.3;
    Serial.print("当前ADC: ");
    Serial.print(currentADC);
    Serial.print(" (");
    Serial.print(currentVoltage, 2);
    Serial.println("V)");

    if (currentVoltage < 0.1)
    {
        Serial.println("警告: 电压过低，检查连接");
    }
    else if (currentVoltage > 3.2)
    {
        Serial.println("警告: 电压过高，检查电路");
    }

    Serial.println("注意: 请保存此值用于后续使用");
    Serial.println();

    // 重要：用校准后的RZero值重新初始化传感器
    Serial.println("用校准值重新初始化传感器...");

#if USE_MANUAL_RZERO
    Serial.println("使用手动设置的标准RZero值: " + String(MANUAL_RZERO_VALUE));
    mq135_sensor_ptr = new MQ135(MQ135_AO_PIN, MANUAL_RZERO_VALUE);
    calibratedRZero = MANUAL_RZERO_VALUE; // 更新显示值
#else
    mq135_sensor_ptr = new MQ135(MQ135_AO_PIN, calibratedRZero);
#endif

    Serial.println("传感器重新初始化完成!");
    Serial.println();

    calibrationMode = false;

    // 显示PPM校正说明
    explainPPMCorrection();

    Serial.println("开始监测...");
}

void loop()
{
    if (calibrationMode || mq135_sensor_ptr == nullptr)
    {
        // 校准模式下或传感器未初始化时不进行测量
        delay(1000);
        return;
    }

    // 获取传感器数据 - 使用指针访问

    // 1. 普通PPM：基于标准条件(20°C, 33%湿度)的CO2浓度读数
    float ppm = mq135_sensor_ptr->getPPM();

    // 2. 校正PPM：根据当前实际温湿度修正后的CO2浓度读数
    // 这个值更准确，因为它考虑了环境因素对传感器的影响
    float correctedPPM = mq135_sensor_ptr->getCorrectedPPM(temperature, humidity);

    // 3. 传感器电阻值
    float resistance = mq135_sensor_ptr->getResistance();
    float correctedResistance = mq135_sensor_ptr->getCorrectedResistance(temperature, humidity);

    // 显示详细信息
    Serial.print("时间: ");
    Serial.print(millis() / 1000);
    Serial.print("s | ");

    // 获取ADC原始值进行诊断
    int adcValue = analogRead(MQ135_AO_PIN);
    float voltage = (adcValue / 4095.0) * 5; // 这里实际上这些复杂的计算不能这么算，它里面是有电阻的

    Serial.print("ADC: ");
    Serial.print(adcValue);
    Serial.print(" (");
    Serial.print(voltage, 2);
    Serial.print("V) | ");

    Serial.print("Rs: ");
    Serial.print(resistance, 1);
    Serial.print("kΩ | ");

    Serial.print("PPM(标准): ");
    Serial.print(ppm, 1);
    Serial.print(" | PPM(校正): ");
    Serial.print(correctedPPM, 1);
    Serial.print(" | ");

    // 显示Rs/R0比值进行诊断
    float ratio = resistance / calibratedRZero;
    Serial.print("Rs/R0: ");
    Serial.print(ratio, 3);
    Serial.print(" | ");

    // 空气质量评级（基于校正后的PPM值）
    Serial.print("状态: ");
    if (correctedPPM < 50)
    {
        Serial.print("优秀");
    }
    else if (correctedPPM < 100)
    {
        Serial.print("良好");
    }
    else if (correctedPPM < 200)
    {
        Serial.print("一般");
    }
    else if (correctedPPM < 400)
    {
        Serial.print("较差");
    }
    else if (correctedPPM < 1000)
    {
        Serial.print("很差");
    }
    else
    {
        Serial.print("极差");
    }

    // 显示PPM差异分析
    float ppmDifference = correctedPPM - ppm;
    float ppmDifferencePercent = (abs(ppmDifference) / ppm) * 100;
    Serial.print(" | 差异: ");
    if (ppmDifference > 0)
    {
        Serial.print("+");
    }
    Serial.print(ppmDifference, 1);
    Serial.print("ppm (");
    Serial.print(ppmDifferencePercent, 1);
    Serial.print("%)");

    // 显示校准信息和诊断
    Serial.print(" | RZero: ");
    Serial.print(calibratedRZero, 1);
    Serial.print("kΩ");

    // 添加诊断信息
    Serial.println();
    Serial.print("诊断: ");
    if (calibratedRZero < 50)
    {
        Serial.print("RZero偏低,检查电路 ");
    }
    if (ratio < 0.3)
    {
        Serial.print("电阻比值异常 ");
    }
    if (voltage < 0.1 || voltage > 3.2)
    {
        Serial.print("电压异常 ");
    }
    Serial.println();

    delay(2000); // 2秒间隔
}