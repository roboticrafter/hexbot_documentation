/**
  *************************************************************************************************
  * @file           ARM_CALIBRATION_BY_ANGLE_TEST
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.09.18
  * @note    ´      此程序用于校准机械臂的初始位置（通过设定固定偏置来达到校准的目的）
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>                           
    
void setup() {
  MyArm.begin(USB_SER);
  Serial.println("Arm_Calibration_By_Pos_Test");
  Serial.println("Please input in the following format :");
  Serial.println("id(byte)  offset(double)");
  Serial.println("such as : 5 100");
  MyArm.position_init();
  delay(2200);
}

void loop() {
  while(Serial.available())
  {
    byte id = Serial.parseInt();
    double offset = Serial.parseInt();
    MyArm.offset_by_angle(id, offset);
    MyArm.Get_Offset();
    Serial.println("get offset");
    for(int i = 0; i < MyArm.Steer_Num; i++)
    {
      Serial.print("id = "); Serial.print(i); Serial.print("  0ffset = "); Serial.println(MyArm.offPos[i]);  
    }
    MyArm.position_init();
  }
}



