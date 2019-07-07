/**
  *************************************************************************************************
  * @file           Steer_Ping_Test
  * @author         
  * @qq             269426626
  * @version        V2.0
  * @date           2017.09.05
  * @note    ´      此程序用于舵机应答测试
  * @config_steer   Serial1
  * @computer       Serial
  *************************************************************************************************
  */

#include<Steer.h>

Steer mystter1(1,&Serial1);                      
//构造函数输入参数：第一个是ID，第二个是与舵机通信的串口

void setup() {
  Serial.begin(115200);
  Serial.println("Steer_Ping_Test");
}

void loop() {
  boolean flag = mystter1.Steer_Ping();
  Serial.print("flag = "); Serial.println(flag);
  delay(1000);
}

