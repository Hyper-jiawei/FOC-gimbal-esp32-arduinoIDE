#define LED_PIN 22  // 使用22号引脚驱动LED（阴极连接）

// PWM参数配置


// 电压参数
const float voltage_limit = 3.3f;    // ESP32 GPIO电压
const float voltage_power_supply = 3.3f;

void setup() {
  Serial.begin(115200);
  
  // 初始化LED引脚（新版API会自动配置）
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // 初始熄灭
  
  // 新版PWM初始化（合并ledcSetup和ledcAttachPin）
  ledcAttach(LED_PIN, 20000, 8);
  
  Serial.println("ESP32 Core 3.0+ PWM呼吸灯测试");
}

void loop() {
  // 呼吸效果：渐亮（电压3.3V→0V）
  for(float voltage = voltage_limit; voltage >= 0; voltage -= 0.05) {
    setPwm(voltage);
    delay(20);
  }
  
  // 呼吸效果：渐暗（电压0V→3.3V）
  for(float voltage = 0; voltage <= voltage_limit; voltage += 0.05) {
    setPwm(voltage);
    delay(20);
  }
  
  // 输出周期计数
  static uint32_t cycle = 0;
  Serial.printf("呼吸周期#%d\n", ++cycle);
}

// 新版PWM生成函数
void setPwm(float voltage) {
  // 电压约束
  voltage = constrain(voltage, 0, voltage_limit);
  
  // 计算占空比（注意LED连接方式）
  float duty = 1.0 - (voltage / voltage_power_supply); // 反向占空比
  
  // 写入PWM（自动管理通道）
  ledcWrite(LED_PIN, (uint32_t)(duty * 255));
}