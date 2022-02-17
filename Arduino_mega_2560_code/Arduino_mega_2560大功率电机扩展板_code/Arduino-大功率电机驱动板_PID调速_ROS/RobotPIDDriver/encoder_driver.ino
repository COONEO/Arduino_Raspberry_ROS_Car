#include <Encoder.h>


/*****    第二步：检查编码器接口是否和您使用的电机对应  ********************
 *   arduino mega 2560 扩展板 上有两个 2.54 mm x 6 的接线端子，直连电机接口排线
 * 先测试左侧电机，然后手动正转电机，然后用 arduino 的串口监视器 利用 命令‘e’ 查询
 * 反馈的数值是否和自己转动的方向对应；若正负相反，建议修改下方的 leftEncoder(,)中
 * 两个参数的顺序。
 *    同样的方法测试右侧电机。
 *******************************************************************/

// 有 5V 电源版本 arduino_mega_2560 扩展板 配置 大功率直流电机驱动器 12V/24V 霍尔直流电机

Encoder leftEncoder(19,18);        
Encoder rightEncoder(20,21);  
/***********************************************************/

long leftPosition;
long rightPosition;

long readEncoder(int i)
{
  if (i==LEFT)
  {
    return leftEncoder.read()-leftPosition;
  }
  else
  {
     return rightEncoder.read()-rightPosition;
  }
}

void resetEncoder(int i)
{
  if (i==LEFT)
  {
    leftPosition=leftEncoder.read();    
  }else
  {
    rightPosition=rightEncoder.read();
  }
}

void resetEncoders()
{  
  resetEncoder(LEFT);
  resetEncoder(RIGHT); 
}
