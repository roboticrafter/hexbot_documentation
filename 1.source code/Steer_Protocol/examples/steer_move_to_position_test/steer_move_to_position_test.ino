/**
  *************************************************************************************************
  * @file           steer_move_to_position_test
  * @author         
  * @qq             269426626
  * @version        V2.0
  * @date           2017.09.06
  * @note    ´      此程序用于让舵机在指定时间内运行到指定位置
  * @config_steer   Serial1
  * @computer       Serial
  *************************************************************************************************
  */

#include<Steer.h>

Steer mysteer5(5,&Serial1);       //初始化一个ID为5的舵机对象               

void setup() {
  Serial.begin(115200);
  Serial.println("Please input in the following format :");
  Serial.println("position(word)  runtime(word)");
  Serial.println("such as : 2334 4301");
}
int i = 0;                                 //记录次数
void loop() {
   while(Serial.available()){             //等待接收数据                   
    word pos = word(Serial.parseInt());   //读取位置数据
    word tim = word(Serial.parseInt());   //读取时间数据
    Serial.println(i++);                  //输出次数      
    Serial.print("position = "); Serial.println(pos);
    Serial.print("time = "); Serial.println(tim);
    Serial.println();
    mysteer5.Set_Steer_position_runtime(pos, tim);//控制舵机旋转
  }
}

