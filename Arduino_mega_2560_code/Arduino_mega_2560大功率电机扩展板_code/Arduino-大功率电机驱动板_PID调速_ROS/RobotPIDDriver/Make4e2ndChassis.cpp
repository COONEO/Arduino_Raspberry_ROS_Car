#include "Make4e2ndChassis.h"

Make4e2ndChassis::Make4e2ndChassis()
{
/**************  第三步 测试控制电机转动方向的 引脚是否对应 ****************
 *    打开串口监视器，输入命令 “ m 10 0 ” 回车，然后看左侧电机是否正转，若否则建议更换
 * 下方 Back_Left_D1 和 Back_Left_D1_B 的引脚顺序，然后再测试。
 *    右边电机测试同上。
 *   
 ********************************************************************/
// 有 5V 电源版本 arduino_mega_2560 扩展板 大功率直流电机驱动器 + 12/24V 霍尔减速直流编码电机
  // 左侧电机控制引脚 
  Back_Left_Ena = 4;      // PWM 调速引脚 【0，255】
  Back_Left_D1 = 22;      // 方向控制引脚 IN1
  Back_Left_D1_B = 24;    // 方向控制引脚 IN2
  
  // 左侧电机控制引脚 
  Back_Right_Ena = 5;    // PWM 调速引脚 【0，255】
  Back_Right_D1 = 26;    // 方向控制引脚 IN1
  Back_Right_D1_B = 28;  // 方向控制引脚 IN2
  
  // 左右侧电机控制引脚 经过牛角插座 ---> 牛角插头排线  -----> 大功率直流电机驱动器控制接口
  /*******************************************************************************/
}

void Make4e2ndChassis::init()
{
  // Define pinMode for the pins and set the frequency for timer1.
  pinMode(Back_Left_D1, OUTPUT);  
  pinMode(Back_Left_D1_B, OUTPUT);
  pinMode(Back_Left_Ena, OUTPUT);
  pinMode(Back_Right_D1, OUTPUT);
  pinMode(Back_Right_D1_B, OUTPUT);
  pinMode(Back_Right_Ena, OUTPUT);
}

// Set speed for motor 1 and 2
void Make4e2ndChassis::setSpeeds(int m1Speed, int m2Speed)
{
  // 控制左侧电机
  if(m1Speed>0)
  {
    digitalWrite(Back_Left_D1, HIGH);
    digitalWrite(Back_Left_D1_B, LOW); 
    analogWrite(Back_Left_Ena, m1Speed );
  }
  else
  {
    digitalWrite(Back_Left_D1, LOW);
    digitalWrite(Back_Left_D1_B, HIGH);
    analogWrite(Back_Left_Ena, -m1Speed );    
  }
  
  // 控制右侧电机
  if(m2Speed>0)
  {
    digitalWrite(Back_Right_D1, HIGH);
    digitalWrite(Back_Right_D1_B, LOW); 
    analogWrite(Back_Right_Ena, m2Speed );
  }
  else
  {
    digitalWrite(Back_Right_D1, LOW);
    digitalWrite(Back_Right_D1_B, HIGH);
    analogWrite(Back_Right_Ena, -m2Speed );    
  }
}
