/**
  *************************************************************************************************
  * @file           steer_get_inf_usetime_test
  * @author         
  * @qq             269426626
  * @version        V2.0
  * @date           2017.09.06
  * @note    ´      此程序用于测试得到舵机全部的信息时耗费的时间（大约12ms每次读取信息）
  * @config_steer   Serial1
  * @computer       Serial
  *************************************************************************************************
  */

#include<Steer.h>

Steer mysteer5(5,&Serial1);                      

void setup() {
  Serial.begin(115200);
  Serial.println("steer_get_inf_usetime_test");
}

void loop() {
    for(int i = 0; i < 100; i++)     
    mysteer5.Get_Steer_All_Inf();
    
    Serial.print("usetime = "); Serial.println(millis());
}

