/**
  *************************************************************************************************
  * @file           Arm_torque_on_off_test
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.09.18
  * @note    ´      此程序为测试舵机扭矩开关(当输入的整数等于0的时候，机械臂失去扭矩，否则，机械臂恢复扭矩)
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>                           
    
void setup() {
  MyArm.begin(USB_SER);
  Serial.println("Arm_torque_on_off_test");
  Serial.println("Please enter a integer:");
  Serial.println("Note: When this integer is equal to zero, set arm torque off.");
  Serial.println("else, set arm torque on.");
 
  MyArm.position_init(); 
  MyArm.Set_Arm_Torque_On();
  delay(3000);
}
int x = 1;

void loop() {
  
  while(Serial.available())
  {
     x = Serial.parseInt();
    if(x == 0)
    {
        Serial.println("Set Arm Torque off");
        MyArm.Set_Arm_Torque_Off();
    }
    else if(x != 0)
    {
      Serial.println("Set Arm Torue On");
      MyArm.position_init();
      MyArm.Set_Arm_Torque_On();
    }
  }
}



