#include "encoder_driver.h"
#include "motor_driver.h"
#include "Make4e2ndChassis.h"
#include "commands.h"
#include <PID_v1.h>
#include "sensors.h"

typedef struct
{
  double target;
  double currentEncoder;
  double lastEncoder;
  double error;
  double input;
  double output;
}
PIDInfo;
PIDInfo leftInfo, rightInfo;

//车轮配置
/*****************************  第一步修改 电机外输出轴 转动一圈 所输出的总脉冲数  ************

   由于是采用的中断方式捕获电机的霍尔脉冲，并且使用的是边沿触发方式，所以电机的编码值计算方法如下：
    encoder = （边沿触发）2 x 霍尔编码器相数量（如：2） x 霍尔编码器线束 (如 13 ) x 电机减速比 (如：30)

************************************************************************************/

//double encoderresolution = 2496.0; //编码器输出脉冲数/圈 2*2*13*90 = 2496    TT-motor encoder
//double encoderresolution = 4680.0; //编码器输出脉冲数/圈 2*2*13*90 = 4680    TT-motor encoder 1:90 金属
double encoderresolution = 1560.0;   //编码器输出脉冲数/圈 2*2*13*30 = 1560    37-motor encoder


/*********************  第四步修改 PID 参数，优化电机的调速性能  *********/

double Kp_L = 5.0, Ki_L = 30.0, Kd_L = 0.0001;   // 15.0 15.00 0.0001
double Kp_R = 5.0, Ki_R = 30.0, Kd_R = 0.0001;   // 15.0 15.00 0.0001


PID leftPID(&leftInfo.input, &leftInfo.output, &leftInfo.target, Kp_L, Ki_L, Kd_L, DIRECT);
PID rightPID(&rightInfo.input, &rightInfo.output, &rightInfo.target, Kp_R, Ki_R, Kd_R, DIRECT);
double pid_rate = 100.0;                    // default is 100 Hz
double pidinterval = 1000.0 / pid_rate;    // PID每次运算结果的执行时间 ms
long nextmotion;
int  moving;

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

#define AUTO_STOP_INTERVAL 1000                // 死机检测，若时间间隔大于1s没有收到指令，则停车
long lastMotorCommand = AUTO_STOP_INTERVAL;
long led_lasttime = millis();

#define BAUDRATE     115200


/*****************************************/

// 定义转向灯状态 枚举类型
enum LED_state
{
  left_2LED,
  right_2LED,
  blink_4LED,
  cycleblink_4LED,
};

LED_state led_state;

// 定义 LED 管脚 标号
char Front_L_LED = 25 ;
char Back_L_LED = 27 ;
char Front_R_LED = 23 ;
char Back_R_LED = 29 ;

// LED闪烁计数器变量
long blink_time = 0;

void Turn_Cycle_Blink();
void Turn_Right_Blink();
void Turn_Left_Blink();
void Blink_4LED();

/*****************************************/

/* Clear the current command parameters */
void resetCommand() {
  cmd = 0 ; //NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break;
    case READ_ENCODERS:     // 'e'
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPIDInfo();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:     // 'm'
      lastMotorCommand = millis();
      setTargetTicksPerFrame(arg1, arg2);
      Serial.println("OK");
      break;
    case UPDATE_PID:
      Serial.println("OK");
      break;
    case DISPLAY_PIDS:
      Serial.print("Kp_L:");
      Serial.print(Kp_L);
      Serial.print(" Kd_L:");
      Serial.print(Kd_L);
      Serial.print(" Ki_L:");
      Serial.print(Ki_L);
      break;
    case READ_RANGE:
      Serial.println(Ultra_Range_cm);
      break; 
    default:
      Serial.println("Invalid Command");
      break;
  }
}

// 初始化程序
void setup()
{
  Serial.begin(BAUDRATE);
  initMotorController();

  resetEncoders();
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(pidinterval);
  leftPID.SetOutputLimits(-200, 200);

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(pidinterval);
  rightPID.SetOutputLimits(-200, 200);

  led_blink_Init();
  // 初始化超声波接口 初始化
  init_ultra_sensor(); 

  setTargetTicksPerFrame(0, 0);
}

// 主循环
void loop()
{
  //从串口读取命令
  while (Serial.available() > 0)
  {
    // Read the next character
    chr = Serial.read();
    // Terminate a command with a CR
    if (chr == 13)
    {
      if (arg == 1)
        argv1[index] = 0 ;//NULL;
      else if (arg == 2)
        argv2[index] = 0; //NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ')
    {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)
      {
        argv1[index] = 0;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else
    {
      if (arg == 0)
      {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1)
      {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2)
      {
        argv2[index] = chr;
        index++;
      }
    }
  }

  //若时间间隔小于规定PID rate，而且已经执行过PID运算了，则会继续进行PID运算
  if (nextmotion <= millis() && moving == 1)
  {
    leftInfo.currentEncoder = readEncoder(LEFT);
    leftInfo.input = leftInfo.currentEncoder - leftInfo.lastEncoder;    //当前采样周期编码值和上次采样周期编码值之间的差值
    leftInfo.error = leftInfo.target - leftInfo.input;
    leftPID.Compute();
    leftInfo.lastEncoder = readEncoder(LEFT);

    rightInfo.currentEncoder = readEncoder(RIGHT);
    rightInfo.input = rightInfo.currentEncoder - rightInfo.lastEncoder;
    rightInfo.error = rightInfo.target - rightInfo.input;
    rightPID.Compute();
    rightInfo.lastEncoder = readEncoder(RIGHT);

    setMotorSpeeds(leftInfo.output, rightInfo.output);
    nextmotion = millis() + pidinterval;  // 1000/100


    /*********   PID 调试时使用 正常使用应注释掉 ***********/
    /*********   格式严格 “:” 与“,”不可以注释掉 ***********/
    
//    Serial.print("Left_Encoder_value: ");
//    Serial.print(leftInfo.input);
//    Serial.print(",");
//    Serial.print("Target_encoder: ");
//    Serial.println(leftInfo.target);
        
    /******** PID 参数调试完毕后 应注释掉上述代码*********/
    /************************************************/
  
  }

  // LED 闪烁 计数器
  if ((millis() - led_lasttime) > 300)
  {
    led_lasttime = millis();
    blink_time ++ ;
    if (blink_time > 2000 )
      blink_time = 0;
  }

  // select led state && blink leds
  switch (led_state)
  {
    case left_2LED:
      Turn_Left_Blink();
      break;
    case right_2LED:
      Turn_Right_Blink();
      break;
    case blink_4LED:
      Blink_4LED();
      break;
    case cycleblink_4LED:
      Turn_Cycle_Blink();
      break;
    default:
      break;
  }

  //自动超时停止保护
  // if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  // {
    // setTargetTicksPerFrame(0, 0);
  // }

  // 执行超声波测距程序 ,测量的值保存在 全局变量 Ultra_Range_cm(int) 中 。
  Detection_Front_Range();
  
}

void setTargetTicksPerFrame(int left, int right)
{
  if (left == 0 && right == 0)
  {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
  else
  {
    moving = 1;
  }
  leftInfo.target = left;
  rightInfo.target = right;

  if (left == -right)
  {
    if (left == 0 && right == 0)
    {
      led_state = cycleblink_4LED;    // 停车，跑马灯
    }
    else
    {
      led_state = blink_4LED;    // 旋转 双闪
    }
  }
  else
  {
    if ( left < right )
    {
      led_state = left_2LED;   // 左转 左转向灯
    }
    else if (left > right)
    {
      led_state = right_2LED;   // 右转 右转向灯
    }
  }

}

void resetPIDInfo()
{
  leftInfo.currentEncoder = 0;
  leftInfo.lastEncoder = 0;

  rightInfo.currentEncoder = 0;
  rightInfo.lastEncoder = 0;
}


void led_blink_Init()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(Front_L_LED, OUTPUT);
  pinMode(Front_R_LED, OUTPUT);
  pinMode(Back_L_LED, OUTPUT);
  pinMode(Back_R_LED, OUTPUT);
}

// 双闪
void Blink_4LED()
{
  if ( blink_time % 2 == 0 )
  {
    digitalWrite(Front_L_LED, HIGH);
    digitalWrite(Front_R_LED, HIGH);
    digitalWrite(Back_L_LED, HIGH);
    digitalWrite(Back_R_LED, HIGH);
  }
  else if ( blink_time % 2 == 1 )
  {
    digitalWrite(Front_L_LED, LOW);
    digitalWrite(Front_R_LED, LOW);
    digitalWrite(Back_L_LED, LOW);
    digitalWrite(Back_R_LED, LOW);
  }
}

// 左转闪烁
void Turn_Left_Blink()
{
  if (blink_time % 2 == 0 )
  {
    digitalWrite(Front_L_LED, HIGH);
    digitalWrite(Back_L_LED, HIGH);
    digitalWrite(Front_R_LED, HIGH);
    digitalWrite(Back_R_LED, HIGH);
  }
  else if (blink_time % 2 == 1 )
  {
    digitalWrite(Front_L_LED, LOW);
    digitalWrite(Back_L_LED, LOW);
    digitalWrite(Front_R_LED, HIGH);
    digitalWrite(Back_R_LED, HIGH);
  }
}

// 右转闪烁
void Turn_Right_Blink()
{
  if (blink_time % 2 == 0 )
  {
    digitalWrite(Front_R_LED, HIGH);
    digitalWrite(Back_R_LED, HIGH);
    digitalWrite(Front_L_LED, HIGH);
    digitalWrite(Back_L_LED, HIGH);
  }
  else if (blink_time % 2 == 1 )
  {
    digitalWrite(Front_R_LED, LOW);
    digitalWrite(Back_R_LED, LOW);
    digitalWrite(Front_L_LED, HIGH);
    digitalWrite(Back_L_LED, HIGH);
  }
}

// 四灯循环跑马灯
void Turn_Cycle_Blink()
{
  switch (blink_time % 4 )
  {
    case 0:
      digitalWrite(Front_L_LED, LOW);
      digitalWrite(Back_L_LED, HIGH);
      digitalWrite(Back_R_LED, HIGH);
      digitalWrite(Front_R_LED, HIGH);
      break;
    case 1 :
      digitalWrite(Front_L_LED, HIGH);
      digitalWrite(Back_L_LED, LOW);
      digitalWrite(Back_R_LED, HIGH);
      digitalWrite(Front_R_LED, HIGH);
      break;
    case 2 :
      digitalWrite(Front_L_LED, HIGH);
      digitalWrite(Back_L_LED, HIGH);
      digitalWrite(Back_R_LED, LOW);
      digitalWrite(Front_R_LED, HIGH);
      break;
    case 3 :
      digitalWrite(Front_L_LED, HIGH);
      digitalWrite(Back_L_LED, HIGH);
      digitalWrite(Back_R_LED, HIGH);
      digitalWrite(Front_R_LED, LOW);
      break;
    default:
      break;
  }
}
