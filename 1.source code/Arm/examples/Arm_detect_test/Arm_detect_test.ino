/**
  *************************************************************************************************
  * @file           Arm_detect_test
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.08.22
  * @note    ´      此程序用于机械臂中舵机的检测
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>                           
    
void setup() { 
   
  //机械臂初始化(这其中进行机械臂舵机自检)
  MyArm.begin(USB_SER); 
  
  Serial.println("Arm_detect_test"); 
  //机械臂位置初始化
  MyArm.position_init();
  delay(2000);
}

void loop() {
  //每隔一秒打印一次机械臂自检过程中得到的舵机数量
  Serial.println(MyArm.Steer_Num); 
  delay(1000);
}


