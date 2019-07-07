/**
  *************************************************************************************************
  * @file           steer_change_id_test
  * @author         
  * @qq             269426626
  * @version        V2.0
  * @date           2017.09.05
  * @note    ´      此程序用于改变舵机ID
  * @config_steer   Serial1
  * @computer       Serial
  *************************************************************************************************
  */

#include<Steer.h>

Steer mystter1(1,&Serial1);     //初始化一个舵机ID为1的对象                 

void setup() {
  Serial.begin(115200);        //初始化与电脑通信串口（波特率115200）
  Serial.println("steer_change_id_test");
  Serial.println("Please enter in the following format: new_ID");
  Serial.println("such as: 3");   
}

void loop() {
   while(Serial.available()){            //等待输入一个新的ID         
    byte new_id = Serial.parseInt();     //接收新ID
    mystter1.Change_Steer_ID(new_id);     //改变舵机的ID为新ID
    delay(500);                  //等待改变，必须至少等待500ms（重要）
    if(mystter1.Steer_Ping())             //检验是否改变ID成功
    Serial.println("Change over. Please input another.") ; 
    
    else Serial.println("error: failed to change id");
  }
}

