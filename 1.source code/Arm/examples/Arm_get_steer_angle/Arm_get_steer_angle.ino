/**
  *************************************************************************************************
  * @file           Arm_get_steer_angle
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.10.24
  * @note    ´      此程序用于打印机械臂各个舵机的角度位置信息
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>                           

double angle[7] = {0};  //存储得到的角度信息
int i = 0;
    
void setup() {
  MyArm.begin(USB_SER);
  Serial.println("Arm_get_steer_angle_test:");
  MyArm.position_init(); 
  delay(2200);
}

void loop() {
  MyArm.get_Arm_angle(angle);                            //得到各个舵机角度信息
  Serial.print("----------");Serial.print(i++);Serial.println("----------");
  for(byte i = 0; i < MyArm.Steer_Num; i++){             //打印角度信息
    Serial.println(angle[i]);
  }
  delay(500);
}



