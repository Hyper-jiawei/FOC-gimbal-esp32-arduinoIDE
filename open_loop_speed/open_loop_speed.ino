// PWM输出引脚定义
int pwmA = 32;
int pwmB = 33;
int pwmC = 25;
int enPin = 12;

//初始变量及函数定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float voltage_power_supply=12.6;
float shaft_angle=0,open_loop_timestamp=0;
float zero_electric_angle=0,Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;

// 新增变量
unsigned long last_micros = 0;
const unsigned long CONTROL_PERIOD = 10000; // 10ms控制周期(微秒)

void setup() {
  Serial.begin(115200);
  //PWM设置
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH); // 启用驱动芯片
  Serial.println("FD6287T使能已开启");

  // 用新 API 初始化 PWM
  ledcAttach(pwmA, 20000, 8);
  ledcAttach(pwmB, 20000, 8);
  ledcAttach(pwmC, 20000, 8);
  Serial.println("完成PWM初始化设置");
  
  // 初始化时间戳
  open_loop_timestamp = micros();
  last_micros = micros();
}

// 电角度求解
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}

// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);
  return a >= 0 ? a : (a + 2*PI);
}

// 设置 PWM 到控制器输出
void setPwm(float Ua, float Ub, float Uc) {
  // 计算并限制占空比到 [0, 1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  // 用「引脚」写入 PWM，占空比 0-255
  ledcWrite(pwmA, uint32_t(dc_a * 255));
  ledcWrite(pwmB, uint32_t(dc_b * 255));
  ledcWrite(pwmC, uint32_t(dc_c * 255));
}

void setPhaseVoltage(float Uq,float Ud, float angle_el) {
  angle_el = _normalizeAngle(angle_el + zero_electric_angle);
  // 帕克逆变换
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 

  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}

//开环速度函数 - 修改最小时间间隔处理
float velocityOpenloop(float target_velocity){
  unsigned long now_us = micros();
  
  // 处理时间戳溢出(约70分钟一次)
  if(now_us < open_loop_timestamp) {
    open_loop_timestamp = now_us;  // 重置时间戳
    return 0;  // 跳过本次计算
  }
  
  //计算当前时间间隔(秒)
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  
  // 确保最小时间间隔
  if(Ts < 0.001f) return 0;  // 间隔过小跳过计算
  
  // 限制最大时间间隔(防止长时间停顿导致突变)
  Ts = _constrain(Ts, 0.001f, 0.02f);
  
  // 更新角度 - 增加角度变化平滑性
  float angle_increment = target_velocity * Ts;
  shaft_angle = _normalizeAngle(shaft_angle + angle_increment);
  
  // 动态调整Uq值 - 低速时降低电压
  float Uq = voltage_power_supply/3;
  if(fabs(target_velocity) < 5.0f) {
    Uq *= 0.7f;  // 低速时降低电压
  }
  
  setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle, 7));
  
  open_loop_timestamp = now_us;
  return Uq;
}

void loop() {
  // 固定周期控制 - 确保稳定时间间隔
  unsigned long current_micros = micros();
  if(current_micros - last_micros >= CONTROL_PERIOD) {
    velocityOpenloop(10);  // 目标速度10rad/s
    last_micros = current_micros;
    
    // 调试输出
    static unsigned long last_print = 0;
    if(current_micros - last_print > 100000) {  // 每100ms打印一次
      Serial.print("Ts: ");
      Serial.print((current_micros - last_micros)*1e-6f, 4);
      Serial.print("s, Angle: ");
      Serial.println(shaft_angle, 4);
      last_print = current_micros;
    }
  }
}