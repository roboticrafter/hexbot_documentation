/**
  *************************************************************************************************
  * @file           steer_get_inf_test
  * @author         
  * @qq             269426626
  * @version        V2.0
  * @date           2017.09.06
  * @note    ´      此程序用于得到舵机的全部状态信息
  * @config_steer   Serial1
  * @computer       Serial
  *************************************************************************************************
  */

#include<Steer.h>

Steer mysteer5(5,&Serial1);       //初始化一个ID为5的舵机对象             

void setup() {
  Serial.begin(115200);
  Serial.println("Steer_Get_Inf_Test");
}

void loop() {
   while(Serial.available()){    //等待输入去获得信息（任意字符）
    Serial.read();                       
    mysteer5.Get_Steer_All_Inf();//获取全部信息
    display_all_inf();           //显示全部信息
  }
}

void display_all_inf()
{
  Serial.print("Position_Current = ");  Serial.println(((word)mysteer5.Position_Current[0]<<8) + mysteer5.Position_Current[1]);
  Serial.print("Position_Target = ");  Serial.println(((word)mysteer5.Position_Target[0]<<8) + mysteer5.Position_Target[1]);
  Serial.print("RunTime = ");  Serial.println(((word)mysteer5.RunTime[0]<<8) + mysteer5.RunTime[1]);
  Serial.print("Max_Angle_Limit = ");  Serial.println(((word)mysteer5.Max_Angle_Limit[0]<<8) + mysteer5.Max_Angle_Limit[1]);
  Serial.print("Min_Angle_Limit = ");  Serial.println(((word)mysteer5.Min_Angle_Limit[0]<<8) + mysteer5.Min_Angle_Limit[1]);
  Serial.println();
}
