/**
  *************************************************************************************************
  * @file           Arm_get_steer_pos
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.10.24
  * @note    ´      此程序用于打印机械臂各个舵机的直接位置信息
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>                           

word pos[7] = {0};  //存储得到的直接位置信息
int i = 0;
    
void setup() {
  MyArm.begin(USB_SER);
  Serial.println("Arm_get_steer_pos_test:");
  MyArm.position_init(); 
  delay(2200);
}

void loop() {
  MyArm.get_Arm_pos(pos);                            //得到各个舵机角度信息
  
  Serial.print("----------");Serial.print(i++);Serial.println("----------");
  
  for(byte i = 0; i < MyArm.Steer_Num; i++){             //打印角度信息
    Serial.print("实际数据： "); Serial.print(pos[i]); Serial.print("  偏置： "); Serial.print(MyArm.offPos[i]);
    Serial.print("   运算后数据： "); Serial.println(pos[i] - MyArm.offPos[i]);
  }
  delay(500);
}



