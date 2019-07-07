/**
  *************************************************************************************************
  * @file           Arm_steer345_test（重载1的例子）
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.09.18
  * @note    ´      此程序用于机械臂中ID为345的舵机控制
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>                           
    
void setup() {
  MyArm.begin(USB_SER);
  Serial.println("Arm_steer345_test");
  Serial.println("Please enter four int data:");
  Serial.println("such as: 2000 2000 1024 3000");
  MyArm.position_init(); //机械臂位置初始化
  delay(2000);
}

void loop() {
   while(Serial.available())
  {
    word st3 = Serial.parseInt(); //得到舵机3的直接位置参数
    word st4 = Serial.parseInt();
    word st5 = Serial.parseInt();
    word t   = Serial.parseInt(); //得到运行时间参数
    
    Serial.println("get st3 st4 st5 t");
   
    Serial.print("st3 = "); Serial.print(st3);
    Serial.print("  st4 = "); Serial.print(st4); 
    Serial.print("  st5 = "); Serial.print(st5); 
    Serial.print("  t = "); Serial.println(t);
    
    //机械臂运动到输入的位置
    MyArm.turn_steer_345_to_positon((word)st3,st4,st5,t);
    Serial.println();
  }
}
