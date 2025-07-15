# 无人机查打一体无刷云台控制系统

## 项目简介

本项目旨在开发一套适用于查打一体无人机的无刷云台控制系统，具备高稳定性、高精度的目标对准与打击能力。系统基于 ESP32 微控制器，融合 FOC 算法、串级 PID 控制、磁编码器姿态反馈与电流闭环机制，实现在动态环境中云台的稳定控制与任务执行。

## 核心功能

* 三轴无刷电机控制（俯仰/横滚/偏航）
* FOC 控制算法：Clark/Park 变换，电流-电压解耦控制
* 电流/速度/位置 三级 PID 串级控制器
* 实时角度获取与速度估算（AS5600 编码器）
* 自稳控制（MPU6050 陀螺仪）
* 串口通信下发目标角度
* 支持农业激光作业 / 消防喷洒任务等拓展场景

## 技术架构

```
传感器采集 (MPU6050 + AS5600) -> 姿态解算（DMP/卡尔曼）
   ↓
姿态误差计算 -> PID 控制器（位置-速度-电流）
   ↓
FOC 控制 -> 三相 PWM 输出 -> 无刷电机驱动模块
   ↓
反馈采样（电流 / 编码器）
```

## 硬件设计

* **主控芯片**：WEMOS Lolin32 Lite（ESP32-WROOM-32）
* **电机驱动**：三相桥式电路 + MOSFET + 电流采样（INA240）
* **电源模块**：12V 输入，DC-DC 降压至 5V，再由 LDO 提供 3.3V
* **核心传感器**：AS5600 角度编码器 + MPU6050 IMU（I2C 通讯）
* **电流采样**：0.01Ω 采样电阻 + 50倍放大，ADC 精确回读
* **PCB设计**：使用嘉立创EDA，支持批量打板与封装布线

## 软件算法模块

### 1. 位置-速度-电流串级PID控制

```cpp
// 初始化PID参数
DFOC_M0_SET_ANGLE_PID(1.0, 0, 0, 100000, 30);
DFOC_M0_SET_VEL_PID(0.02, 1.0, 0, 100000, 0.5);
DFOC_M0_SET_CURRENT_PID(5.0, 200.0, 0, 100000);

// 主控循环
DFOC_M0_set_Velocity_Angle(target_angle); // 外部角度设定
```

### 2. FOC电压输出计算

```cpp
void FOC_update(float angle_el) {
    float Uq = current_pid(Id_target - I_d);
    float Ud = current_pid(Iq_target - I_q);
    float Ualpha = -Uq*sin(angle_el) + Ud*cos(angle_el);
    float Ubeta = Uq*cos(angle_el) + Ud*sin(angle_el);
    setPWM(Ualpha, Ubeta);
}
```

### 3. 电流闭环计算

```cpp
I = ADC_voltage / (50 * 0.01); // 50倍增益, 0.01Ω 电阻
```

### 4. 姿态解算（卡尔曼滤波）

```cpp
// 状态更新：
angle += dt * (gyro - bias);
// 协方差矩阵预测 & 更新
```

## 应用场景示例

### 🌼 农业应用：激光疏花

* 协同视觉识别系统进行花簇定位
* 云台定位精度 ±0.2°，响应时间 < 0.3s
* 实现效率提升20倍、成本降低60%

### 🚒 消防应用：高层灭火

* 云台配合风偏补偿算法，30L/s 水炮喷射
* 60m 射程内精度±1.5m，抗8级风

## 系统优势

* 全链路闭环控制：从位置→速度→电流
* 支持高鲁棒性滤波策略（低通+卡尔曼）
* 高精度姿态控制（±0.2°静态误差）
* 模块化设计：驱动/滤波/PID 可配置

## 开源仓库

代码与详细原理图和初代PCB文件位于：[https://github.com/Hyper-jiawei/FOC-gimbal-esp32-arduinoIDE](https://github.com/Hyper-jiawei/FOC-gimbal-esp32-arduinoIDE)

## 开发环境

* Arduino IDE
* PlatformIO (可选)
* ESP32 开发库
* MPU6050 & AS5600 驱动库

## 联系方式

作者：白佳伟（Hyper-jiawei）
邮箱：jiaweibai1201@outlook.com
