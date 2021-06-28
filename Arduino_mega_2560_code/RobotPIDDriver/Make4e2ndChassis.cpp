#include "Make4e2ndChassis.h"

Make4e2ndChassis::Make4e2ndChassis()
{
  Back_Left_Ena = 4;
  Back_Left_D1 = 22;
  Back_Left_D1_B = 23;

  Back_Right_Ena = 5;
  Back_Right_D1 = 24;
  Back_Right_D1_B = 25;
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
  if(m1Speed<0)
  {
    digitalWrite(Back_Left_D1, HIGH);
    digitalWrite(Back_Left_D1_B, LOW); 
    analogWrite(Back_Left_Ena, -m1Speed );
  }
  else if(m1Speed>0)
  {
    digitalWrite(Back_Left_D1, LOW);
    digitalWrite(Back_Left_D1_B, HIGH);
    analogWrite(Back_Left_Ena, m1Speed );    
  }
  else {    
    digitalWrite(Back_Left_D1, LOW);
    digitalWrite(Back_Left_D1_B, LOW);
    analogWrite(Back_Left_Ena, 0 ); 
  }
  
  // 控制右侧电机
  if(m2Speed<0)
  {
    digitalWrite(Back_Right_D1, HIGH);
    digitalWrite(Back_Right_D1_B, LOW); 
    analogWrite(Back_Right_Ena, -m2Speed );
  }
  else if(m2Speed>0)
  {
    digitalWrite(Back_Right_D1, LOW);
    digitalWrite(Back_Right_D1_B, HIGH);
    analogWrite(Back_Right_Ena, m2Speed );    
  }
  else {
    digitalWrite(Back_Right_D1, LOW);
    digitalWrite(Back_Right_D1_B, LOW);
    analogWrite(Back_Right_Ena, 0 ); 
  }
}
