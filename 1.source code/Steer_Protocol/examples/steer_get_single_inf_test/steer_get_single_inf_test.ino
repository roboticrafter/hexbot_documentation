/**
  *************************************************************************************************
  * @file           steer_get_inf_test
  * @author         
  * @qq             269426626
  * @version        V2.0
  * @date           2017.09.06
  * @note    ´      此程序用于得到舵机的部分状态信息
  * @config_steer   Serial1
  * @computer       Serial
  *************************************************************************************************
  */

#include<Steer.h>

Steer mysteer5(5,&Serial1);                      

void setup() {
  Serial.begin(115200);
  Serial.println("Steer_Get_Single_Inf_Test");
}

void loop() {
   while(Serial.available()){   
    Serial.read();                       
    mysteer5.Get_Steer_Position_Current_Inf();
    Serial.print("Position_Current = ");  Serial.println(((word)mysteer5.Position_Current[0]<<8) + mysteer5.Position_Current[1]);
  }
}

