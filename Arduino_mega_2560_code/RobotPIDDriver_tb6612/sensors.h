
#define TrigPin 30
#define EchoPin 32

int Ultra_Range_cm;

void init_ultra_sensor() 
{
   pinMode(TrigPin, OUTPUT);
   pinMode(EchoPin, INPUT);
   Ultra_Range_cm = 0 ;
}

void Detection_Front_Range() 
{ 
  //低高低电平发一个短时间脉冲去TrigPin
  digitalWrite(TrigPin, LOW); 
  delayMicroseconds(2);
  
  //输入高电平
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  //输入低电平
  digitalWrite(TrigPin, LOW);
  
  //读取一个引脚的脉冲（HIGH或LOW）。例如，如果value是HIGH，pulseIn()会等待引脚变为HIGH，开始计时，再等待引脚变为LOW并停止计时。
  //接收到的高电平的时间(us)*340m/s/2=接收到高电平的时间(us)*17000cm/1000000us = 接收到高电平的时间*17/1000(cm) 
  Ultra_Range_cm = int( ( pulseIn(EchoPin, HIGH) / 10000.0 )* 340.0 ) / 2.0 ; 

  //单独测试 超声波的时候，才取消掉下面的串口输出注释。
//  Serial.print("ultra_sensor_dis: ");
//  Serial.println(Ultra_Range_cm);

}
