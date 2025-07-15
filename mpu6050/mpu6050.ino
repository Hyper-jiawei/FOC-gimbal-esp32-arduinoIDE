#include "MPU6050.h"

MPU6050Handler imu;

void setup() {
    Serial.begin(115200);
    
    // 初始化MPU6050 (SDA=21, SCL=22, INT=4)
    if (!imu.begin()) {
        Serial.println("MPU6050初始化失败!");
        while(1);
    }
    
    Serial.println("校准传感器...保持水平静止");
    delay(2000);
    imu.calibrate();
    Serial.println("校准完成!");
}

void loop() {
    static uint32_t lastPrint = 0;
    
    // 更新传感器数据 (200Hz)
    imu.update();
    
    // 100ms打印一次数据
    if (millis() - lastPrint >= 100) {
        Serial.print("Pitch: ");
        Serial.print(imu.getPitch());
        Serial.print("°\tRoll: ");
        Serial.print(imu.getRoll());
        Serial.print("°\tYaw: ");
        Serial.print(imu.getYaw());
        Serial.println("°");
        
        lastPrint = millis();
    }
    
    // 在这里添加你的FOC控制代码
    // float pitch = imu.getPitch();
    // float roll = imu.getRoll();
    // 调用你的PID控制器驱动无刷电机
}