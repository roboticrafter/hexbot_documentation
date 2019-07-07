/**
  *************************************************************************************************
  * @file           steer_reset_test
  * @author         
  * @qq             269426626
  * @version        V2.0
  * @date           2017.09.06
  * @note    ´      此程序用于舵机恢复出厂设置（舵机复位后ID值为0X01）
  * @config_steer   Serial1
  * @computer       Serial
  *************************************************************************************************
  */

#include<Steer.h>

Steer mysteer5(5,&Serial1);                      

void setup() {
  Serial.begin(115200);
  Serial.println("steer_reset_test");
}

void loop() {  
  while(Serial.available())      //按任意键进行复位操作
  {
      Serial.read();
      mysteer5.Set_Steer_Reset();
      delay(500);    //至少延时500ms，等待恢复出厂设置完成
      if(mysteer5.Steer_Ping()) //判断是否恢复出厂设置
      Serial.println("Set steer reset over");
      else Serial.println("Error: Set steer reset error");
  }
}

//暂时发现对改变舵机ID的操作，一般需要等待500ms以上的时间，否则会有概率使下面的操作失败
