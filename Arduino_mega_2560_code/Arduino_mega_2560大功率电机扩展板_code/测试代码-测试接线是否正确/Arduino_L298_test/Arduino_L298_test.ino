/**********************************************************************************************
*                                 请先看看我，请先看看我，请先看看我！！！ 
***********************************************************************************************                                 
* 本程序主要是为了测试 Arduino 扩展板和 大功率电机驱动器 之间的连线是否正确；
* 具体的连线方法，见 COONEO 微信公众号 或者 知乎 中的文章：
*   《开源！手把手教你驱动Arduino+ROS大功率直流电机》
* 主要的测试效果就是：
*   通过程序和电机转动的现象，判断 电机驱动器两输出端 和电机 正负极 接线顺序是否正确；
*  测试步骤：
*   1. 先烧录本程序，烧录之后，两个电机会 正转 ----> 暂停 ----> 正转 ----> 暂停....
*    若不是正 转，那么就得 切换一下 电机驱动器输出端 两根导线的顺序；
*   2. 然后取消掉最下面程序中的 反转代码注释； 并将正传代码加上注释，然后再编译烧录代码，
*     正常现象应该是电机会反转；
*     
*  注意事项：
*     1. 连接 Arduino 扩展板  和 驱动器 之间使用的是 牛角插头排线，请按照 推文 中的提示连接； 
*     2. 连接 驱动器电源、扩展板电源的时候，切记需要： 
*       检查电源、导线正负极！  
*       检查电源、导线正负极！  
*       检查电源、导线正负极！ 
*       
*     3. 如果需要其他的技术支持，或者交流，欢迎加入我们的 微信交流群，在 COONEO 微信公众号后台留言入群即可。
***************************************************************************************************/

// 控制左侧电机
int enA = 4;   // PWM 调速引脚
int in1 = 22;  // 电机方向控制位 1
int in2 = 24;  // 电机方向控制位 2

// 控制 右侧电机
int enB = 5;   // PWM 调速引脚
int in3 = 26;  // 电机方向控制位 1
int in4 = 28;  // 电机方向控制位 2

// 该程序片段 只会执行一次，一般用来初始化引脚和变量等 。
void setup() {
	// 设置 控制引脚均为输出状态。
  // PWM 输出引脚
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);

  // 电机方向 控制引脚
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	//默认初始化电机驱动器为 停止状态 。
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

// 该函数会一直执行，类似于 while（）函数 ；
void loop() {
	directionControl();
	delay(1000);
}

// This function lets you control spinning direction of motors
void directionControl() {
  // 电机应该会正转 ，测试反转时候，再注释掉
  analogWrite(enA, 30);    // PWM 值范围应该在[0,255] 之间。
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	analogWrite(enB, 30);  
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);

// 反转，先注释掉，测试反转时候再取消注释
//  analogWrite(enA, 30);
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, HIGH);
//  analogWrite(enB, 30);  
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, HIGH);

  delay(2000);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
