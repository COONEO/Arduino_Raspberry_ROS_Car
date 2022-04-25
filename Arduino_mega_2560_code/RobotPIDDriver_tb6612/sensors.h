#define TrigPin 30
#define EchoPin 32

int Ultra_Range_cm;
double last_ultra_time = 0.0 ;
double current_ultra_time = 0.0 ;

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
  delayMicroseconds(2);     // 1000 微秒 = 1 毫秒
  
  //输入高电平
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  //输入低电平
  digitalWrite(TrigPin, LOW);

  /************** 通过millis() 函数记录下 ，在运行超声波的时候，整个程序的系统执行一次的时长 ms 。 ****************************/
  // 测试完毕之后，记得注释掉以下四行代码
//  current_ultra_time = millis();
//  Serial.print("Ultra delta time: ");
//  Serial.println(millis() - last_ultra_time);
//  last_ultra_time = current_ultra_time;
  
  //读取一个引脚的脉冲（HIGH或LOW）。例如，如果value是HIGH，pulseIn()会等待引脚变为HIGH，开始计时，再等待引脚变为LOW并停止计时。
  //接收到的高电平的时间(us)*340m/s/2=接收到高电平的时间(us)*17000cm/1000000us = 接收到高电平的时间*17/1000(cm) ;
  //为了避免超声波 电平检测超时，影响的整个系统的执行时间，这里加上了一个 timeout = 6000 us ,超时之后将返回0 ;
  // 因此 根据声速 测算得知: 此时超声波的探测距离为【0，1.7m】，算法为： ( ( 340 (m/s) * 6 (ms) ) / 1000 (s) ) / 2.0  = 1.02 m ;
  Ultra_Range_cm = int( ( pulseIn(EchoPin, HIGH, 6000) / 10000.0 )* 340.0 ) / 2.0 ;

  //单独测试 超声波的时候，才取消掉下面的串口输出注释。
//  Serial.print("ultra_sensor_dis: ");
//  Serial.println(Ultra_Range_cm);

}
