/**
  *************************************************************************************************
  * @file           Arm_IK_move_Test
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.09.18
  * @note    ´      此程序用于机械臂运动学逆解，并且运动到该位置
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>                           

Serial_arm sa; //初始化一个串口接收对象
void setup() {
  MyArm.begin(USB_SER);
  Serial.println("Arm_IK_move_Test");
  Serial.println("Please enter three double and an int data:");
  Serial.println("such as : 0 176。0 210.5 2000");
  MyArm.position_init();
  delay(2000);
}

void loop() {
  while(Serial.available())
  {
    double x = sa.parsedouble(&Serial);//接收一个double型数据
    double y = sa.parsedouble(&Serial);
    double z = sa.parsedouble(&Serial);
    int t = Serial.parseInt();
    
    Serial.println("get xyzt");
   
    Serial.print("x = "); Serial.print(x);
    Serial.print("  y = "); Serial.print(y); 
    Serial.print("  z = "); Serial.print(z); 
    Serial.print("  t = "); Serial.println(t);
    
    //在t的时间内，机械臂的末端位置运行到（x,y,z）位置处
    MyArm.move_to_position((double)x,y,z,t);  
    for(int i = 0; i < 3; i++)
    {
      Serial.print(MyArm.Rad2Angle(MyArm.theta[i]));
      Serial.print("  ");  
    }
    Serial.println();
  }
}
