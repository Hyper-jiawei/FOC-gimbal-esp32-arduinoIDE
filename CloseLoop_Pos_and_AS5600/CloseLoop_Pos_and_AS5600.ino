

#include "AS5600.h"
#include "Arduino.h"


int pwmA = 32;
int pwmB = 33;
int pwmC = 25;

//初始变量及函数定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。
float voltage_limit=12.0;
float voltage_power_supply=12.0;
float shaft_angle=0,open_loop_timestamp=0;
float zero_electric_angle=0,Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;
#define _3PI_2 4.71238898038f

int PP=7,DIR=-1;
float _electricalAngle(){
  return  _normalizeAngle((float)(DIR *  PP) * getAngle_Without_track()-zero_electric_angle);
}


// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  
  //三目运算符。格式：condition ? expr1 : expr2 
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}


void setPhaseVoltage(float Uq,float Ud, float angle_el) {
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 

  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}

void setup() {
  Serial.begin(115200);

  // 用新 API 初始化 PWM（合并 ledcSetup + ledcAttachPin）
  ledcAttach(pwmA, 30000, 8);
  ledcAttach(pwmB, 30000, 8);
  ledcAttach(pwmC, 30000, 8);
  Serial.println("完成 PWM 初始化设置");

  BeginSensor();
  setPhaseVoltage(3, 0, _3PI_2);
  delay(3000);
  zero_electric_angle = _electricalAngle();
  setPhaseVoltage(0, 0, _3PI_2);
  Serial.print("0电角度："); Serial.println(zero_electric_angle);
}


// 设置 PWM 到控制器输出
void setPwm(float Ua, float Ub, float Uc) {
  // 限制电压到 [0, voltage_limit]
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);

  // 计算并限制占空比到 [0, 1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  // 用「引脚」写入 PWM，占空比 0–255
  ledcWrite(pwmA, uint32_t(dc_a * 255));
  ledcWrite(pwmB, uint32_t(dc_b * 255));
  ledcWrite(pwmC, uint32_t(dc_c * 255));
}


//==============串口接收==============
float motor_target;
int commaPosition;
String serialReceiveUserCommand() {
  
  // a string to hold incoming data
  static String received_chars;
  
  String command = "";

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;

    // end of user input
    if (inChar == '\n') {
      
      // execute the user command
      command = received_chars;

      commaPosition = command.indexOf('\n');//检测字符串中的逗号
      if(commaPosition != -1)//如果有逗号存在就向下执行
      {
          motor_target = command.substring(0,commaPosition).toDouble();            //电机角度
          Serial.println(motor_target);
      }
      // reset the command buffer 
      received_chars = "";
    }
  }
  return command;
}
void loop() {
  // 1. 读取并打印当前电角度（弧度）
  float angle_rad = getAngle();
  Serial.println(angle_rad, 6);

  // // 2. 用同一个值做后续控制计算
  // float Kp = 0.133f;
  // float error = motor_target - DIR * angle_rad * 180.0f / PI;
  // float Uq = _constrain(Kp * error, -5.0f, 5.0f);
  // setPhaseVoltage(Uq, 0, _electricalAngle());

  // 3. 处理串口新命令
  serialReceiveUserCommand();

  delay(10);  // 根据需要调整输出频率
}

