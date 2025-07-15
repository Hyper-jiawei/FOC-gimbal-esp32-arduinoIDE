#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

class MPU6050Handler {
public:
    MPU6050Handler();
    bool begin(uint8_t sda_pin = 19, uint8_t scl_pin = 22, uint8_t int_pin = 4);
    void update();
    void calibrate();
    
    // 获取姿态数据
    float getPitch() const { return pitch; }
    float getRoll() const { return roll; }
    float getYaw() const { return yaw; }
    
    // 获取原始数据
    float getGyroX() const { return gyroX; }
    float getGyroY() const { return gyroY; }
    float getGyroZ() const { return gyroZ; }
    float getAccelX() const { return accX; }
    float getAccelY() const { return accY; }
    float getAccelZ() const { return accZ; }
    
private:
    MPU6050 mpu;
    uint8_t interruptPin;
    bool dmpReady;
    
    // 姿态数据
    volatile float pitch, roll, yaw;
    volatile float gyroX, gyroY, gyroZ;
    volatile float accX, accY, accZ;
    
    // DMP变量
    Quaternion q;
    VectorFloat gravity;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    
    // 时间跟踪
    uint32_t prevUpdate;
    float dt;
    
    void updateDMP();
    void updateRawWithFilter();
};

#endif