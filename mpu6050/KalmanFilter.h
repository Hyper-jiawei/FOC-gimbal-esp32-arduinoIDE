#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
public:
    KalmanFilter() : Q_angle(0.001), Q_bias(0.003), R_measure(0.03) {}
    
    void setAngle(float angle) {
        this->angle = angle;
        bias = 0;
        P[0][0] = 0;
        P[0][1] = 0;
        P[1][0] = 0;
        P[1][1] = 0;
    }
    
    float update(float newAngle, float newRate, float dt) {
        // 预测
        rate = newRate - bias;
        angle += dt * rate;
        
        // 更新协方差矩阵
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;
        
        // 计算卡尔曼增益
        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;
        
        // 更新状态
        float y = newAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;
        
        // 更新协方差
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;
        
        return angle;
    }
    
private:
    float Q_angle;   // 过程噪声协方差 (角度)
    float Q_bias;    // 过程噪声协方差 (偏差)
    float R_measure; // 测量噪声协方差
    
    float angle;     // 计算角度
    float bias;      // 陀螺仪偏差
    float rate;      // 无偏速率
    
    float P[2][2];   // 误差协方差矩阵
};

#endif