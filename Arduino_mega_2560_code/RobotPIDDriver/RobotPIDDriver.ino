#include "encoder_driver.h"
#include "motor_driver.h"
#include "Make4e2ndChassis.h"
#include "commands.h"
#include <PID_v1.h>

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
double encoderresolution = 2496.0; //编码器输出脉冲数/圈 13*2*48*2 = 2496

//PID参数配置
double Kp_L = 2.0, Ki_L = 6.5, Kd_L = 0.0;   //2.0 5.0 0.003
double Kp_R = 2.0, Ki_R = 5.0, Kd_R = 0.003;   //2.0 5.0 0.003


PID leftPID(&leftInfo.input, &leftInfo.output, &leftInfo.target, Kp_L, Ki_L, Kd_L, DIRECT);
PID rightPID(&rightInfo.input, &rightInfo.output, &rightInfo.target, Kp_R, Ki_R, Kd_R, DIRECT);
double pid_rate = 60.0;                    // default is 60 Hz
double pidinterval = 1000.0 / pid_rate;    // PID每次运算结果的执行时间
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

#define BAUDRATE     115200

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
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
  leftPID.SetOutputLimits(-255, 255);

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(pidinterval);
  rightPID.SetOutputLimits(-255, 255);

  setTargetTicksPerFrame(0,0);
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
        argv1[index] = NULL;
      else if (arg == 2)
        argv2[index] = NULL;
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
        argv1[index] = NULL;
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
    nextmotion = millis() + pidinterval;  

  }

  //自动超时停止保护
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL)
  {
    setTargetTicksPerFrame(0, 0);
  }
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
}

void resetPIDInfo()
{
  leftInfo.currentEncoder = 0;
  leftInfo.lastEncoder = 0;

  rightInfo.currentEncoder = 0;
  rightInfo.lastEncoder = 0;
}
