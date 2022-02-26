#include "Make4e2ndChassis.h"

Make4e2ndChassis::Make4e2ndChassis()
{
/**************  第三步 测试控制电机转动方向的 引脚是否对应 ****************
 *    打开串口监视器，输入命令 “ m 10 0 ” 回车，然后看电机是否正转，若否则建议更换
 * 下方 Back_Left_D1 和 Back_Left_D1_B 的引脚顺序，然后再测试。
 *    右边电机测试同上。
 *   
 ********************************************************************/
// 有 5V 电源版本 arduino_mega_2560 扩展板 tb6612FNG
// 在测试完毕 电机排线引脚线序正常之后，再测试 电机驱动器引脚是否正确
  Back_Left_Ena = 4;     // 左轮电机控制 PWM 输出引脚 （0-255）范围，
  Back_Left_D1 = 22;     // 左电机转动方向控制位 引脚 24 ，
  Back_Left_D1_B = 24;   // 左电机转动方向控制位 引脚 22 ，

  Back_Right_Ena = 5;    // 左轮电机控制 PWM 输出引脚 （0-255）范围，
  Back_Right_D1 = 28;    // 左电机转动方向控制位 引脚 26 ，
  Back_Right_D1_B = 26;  // 左电机转动方向控制位 引脚 28 ，

// 有 5V 电源版本 arduino_mega_2560 扩展板 tb6612FNG   tt-motor 1:90
//// 在测试完毕 电机排线引脚线序正常之后，再测试 电机驱动器引脚是否正确
//  Back_Left_Ena = 4;     // 左轮电机控制 PWM 输出引脚 （0-255）范围，
//  Back_Left_D1 = 22;     // 左电机转动方向控制位 引脚 24 ，
//  Back_Left_D1_B = 24;   // 左电机转动方向控制位 引脚 22 ，
//
//  Back_Right_Ena = 5;    // 左轮电机控制 PWM 输出引脚 （0-255）范围，
//  Back_Right_D1 = 28;    // 左电机转动方向控制位 引脚 26 ，
//  Back_Right_D1_B = 26;  // 左电机转动方向控制位 引脚 28 ，

  TB6612_STBY = 3;      // TB6612FNG 电机驱动模块儿专属，使能之后模块儿才能工作。

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
  pinMode(TB6612_STBY,OUTPUT);
}

// Set speed for motor 1 and 2
void Make4e2ndChassis::setSpeeds(int m1Speed, int m2Speed)
{
  if(m1Speed == 0 && m2Speed == 0)
  {
    digitalWrite(TB6612_STBY,LOW);
  }
  else
  {
    // 使能 电机驱动芯片
    digitalWrite(TB6612_STBY,HIGH);
    
  }
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
