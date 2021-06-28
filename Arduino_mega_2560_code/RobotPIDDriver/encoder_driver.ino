#include <Encoder.h>

Encoder leftEncoder(18,19);  //19 18
Encoder rightEncoder(21,20);  //21 20

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
