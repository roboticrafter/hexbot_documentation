/**
  *************************************************************************************************
  * @file           Arm_IK_Test
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.09.16
  * @note    ´      此程序测试运动学逆解（由末端位置推测关节角度）
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>                           

Serial_arm sa; //初始化一个串口接收对象

void setup() {
  MyArm.begin(USB_SER);
  Serial.println("Inverse_Movement_Test");
  Serial.println("Please enter three double data:");
}

void loop() {
  
  while(Serial.available())
  {
    double x = sa.parsedouble(&Serial);//接收一个double型数据
    double y = sa.parsedouble(&Serial);
    double z = sa.parsedouble(&Serial);
    
    Serial.println("get xyz");
    MyArm.inverse_movement(x,y,z);//姿态逆解运算
    Serial.print("x = "); Serial.print(x);
    Serial.print("  y = "); Serial.print(y);
    Serial.print("  z = "); Serial.println(z);
    for(int i = 0; i < 3; i++)
    {
      //输出逆解运算结果(注意，在这里我们将角度值转化为了弧度值)
      Serial.print(MyArm.Rad2Angle(MyArm.theta[i]));
      Serial.print("  ");  
    }
    Serial.println();
  }
}



