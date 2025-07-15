#include "MPU6050.h"
#include "KalmanFilter.h"

// 卡尔曼滤波器实例
KalmanFilter kalmanX, kalmanY;

MPU6050Handler::MPU6050Handler() 
    : dmpReady(false), pitch(0), roll(0), yaw(0), 
      gyroX(0), gyroY(0), gyroZ(0),
      accX(0), accY(0), accZ(0) {}

bool MPU6050Handler::begin(uint8_t sda_pin, uint8_t scl_pin, uint8_t int_pin) {
    interruptPin = int_pin;
    Wire.begin(sda_pin, scl_pin);
    Wire.setClock(400000);
    
    mpu.initialize();
    if (!mpu.testConnection()) {
        return false;
    }
    
    // 启用DMP
    if (mpu.dmpInitialize() == 0) {
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        
        // 设置初始卡尔曼角度
        kalmanX.setAngle(0);
        kalmanY.setAngle(0);
    } else {
        dmpReady = false;
    }
    
    pinMode(interruptPin, INPUT_PULLUP);
    prevUpdate = micros();
    return true;
}

void MPU6050Handler::calibrate() {
    // 加速度计校准 (需水平放置)
    mpu.CalibrateAccel(6);
    
    // 陀螺仪校准 (需保持静止)
    mpu.CalibrateGyro(6);
    
    // 重置卡尔曼滤波器
    kalmanX.setAngle(0);
    kalmanY.setAngle(0);
    pitch = 0;
    roll = 0;
    yaw = 0;
}

void MPU6050Handler::update() {
    if (dmpReady) {
        updateDMP();   // 不再需要返回值
    } else {
        updateRawWithFilter(); // 不再需要返回值
    }
}

void MPU6050Handler::updateDMP() {
    fifoCount = mpu.getFIFOCount();
    if (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        // 获取四元数
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        
        // 获取重力矢量
        mpu.dmpGetGravity(&gravity, &q);
        
        // 获取欧拉角 (单位: 弧度)
        float ypr[3];
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // 转换为度
        yaw = ypr[0] * 180 / M_PI;
        pitch = ypr[1] * 180 / M_PI;
        roll = ypr[2] * 180 / M_PI;
        
        // 获取原始数据
        VectorInt16 aa, gg;
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGyro(&gg, fifoBuffer);
        
        accX = aa.x / 16384.0;
        accY = aa.y / 16384.0;
        accZ = aa.z / 16384.0;
        
        gyroX = gg.x / 131.0;
        gyroY = gg.y / 131.0;
        gyroZ = gg.z / 131.0;
    }
}

void MPU6050Handler::updateRawWithFilter() {
    // 读取原始数据
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // 转换为物理单位
    accX = ax / 16384.0;
    accY = ay / 16384.0;
    accZ = az / 16384.0;
    
    gyroX = gx / 131.0;
    gyroY = gy / 131.0;
    gyroZ = gz / 131.0;
    
    // 计算时间差
    uint32_t now = micros();
    dt = (now - prevUpdate) / 1000000.0;
    prevUpdate = now;
    
    // 加速度计计算姿态
    float accPitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180 / M_PI;
    float accRoll = atan2(-accX, accZ) * 180 / M_PI;
    
    // 卡尔曼滤波融合
    pitch = kalmanY.update(accPitch, gyroY, dt);
    roll = kalmanX.update(accRoll, gyroX, dt);
    yaw += gyroZ * dt;  // 陀螺仪积分
    
    // 偏航角归一化
    if (yaw > 180) yaw -= 360;
    else if (yaw < -180) yaw += 360;
}
