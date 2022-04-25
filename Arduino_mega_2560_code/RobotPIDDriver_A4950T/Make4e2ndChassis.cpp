#include "Make4e2ndChassis.h"

Make4e2ndChassis::Make4e2ndChassis()
{
/**************  第三步 测试控制电机转动方向的 引脚是否对应 ****************
 *    打开串口监视器，输入命令 “ m 10 0 ” 回车，然后看电机是否正转，若否则建议更换
 * 下方 Back_Left_D1 和 Back_Left_D1_B 的引脚顺序，然后再测试。
 *    右边电机测试同上。
 *   
 ********************************************************************/
// 有 5V 电源版本 arduino_mega_2560 扩展板 A4950T
// 在测试完毕 电机排线引脚线序正常之后，再测试 电机驱动器引脚是否正确
  Back_Left_D1 = 6;     // 左电机转动方向控制位 引脚  ，
  Back_Left_D1_B = 5;   // 左电机转动方向控制位 引脚  ，

  Back_Right_D1 = 2;    // 左电机转动方向控制位 引脚  ，
  Back_Right_D1_B = 3;  // 左电机转动方向控制位 引脚  ，

}

void Make4e2ndChassis::init()
{
  // Define pinMode for the pins and set the frequency for timer1.
  pinMode(Back_Left_D1, OUTPUT);
  pinMode(Back_Left_D1_B, OUTPUT);

  pinMode(Back_Right_D1, OUTPUT);
  pinMode(Back_Right_D1_B, OUTPUT);

  // change pwm frequency ,making motors run smoothly
  TCCR3B = TCCR3B & 0b11111000| 0x01 ;
  TCCR4B = TCCR4B & 0b11111000| 0x01 ;
  
}

// Set speed for motor 1 and 2
void Make4e2ndChassis::setSpeeds(int m1Speed, int m2Speed)
{
  // 控制左侧电机
  if(m1Speed>0)
  {
    analogWrite(Back_Left_D1, m1Speed);
    analogWrite(Back_Left_D1_B, 0); 
  }
  else
  {
    analogWrite(Back_Left_D1, 0);
    analogWrite(Back_Left_D1_B, -m1Speed);
  }
  
  // 控制右侧电机
  if(m2Speed>0)
  {
    analogWrite(Back_Right_D1, m2Speed);
    analogWrite(Back_Right_D1_B, 0); 
  }
  else
  {
    analogWrite(Back_Right_D1, 0);
    analogWrite(Back_Right_D1_B, -m2Speed);   
  }
}
