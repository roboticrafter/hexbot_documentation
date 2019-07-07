/**
  *************************************************************************************************
  * @file           Arm_learn_test
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.09.18
  * @note    ´      此程序用于机械臂位点学习演示程序（具体用法请参考视频教程或者PDF教程）
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>   
#include <Arm_learn.h>                        
    
void setup() {
  MyArm.begin(USB_SER);
  Serial.println("Arm_learn_test");
  MyArm_learn.Arm_learn_(); //学习程序初始化
  MyArm.position_init();
  delay(2000);
}

void loop() {
   MyArm_learn.start_learn(); //学习
   MyArm_learn.reappear_learn(); //重现
}


