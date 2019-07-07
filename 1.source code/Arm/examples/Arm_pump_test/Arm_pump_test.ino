/**
  *************************************************************************************************
  * @file           Arm_pump_on_off_test
  * @author         ZWC
  * @qq             269426626
  * @qq_group       604556405
  * @version        V2.0
  * @date           2017.10.24
  * @note    ´      此程序真空吸杯测试程序：当你输入0时，真空吸杯停止吸气；当你输入不等于0的数，真空吸杯开始运动。
  * @computer       Serial
  *************************************************************************************************
  */

#include<Arm.h>    
#include<Arm_pump.h>                       

Arm_pump *p1 = new Arm_pump(); //新建一个吸杯对象
    
void setup() {
  MyArm.begin(USB_SER);
  Serial.println("Arm_pump_on_off_test");
  Serial.println("Please enter a integer:");
  Serial.println("Note: When this integer is equal to zero, set arm pump off.");
  Serial.println("else, set arm pump on.");
 
  MyArm.position_init(); //位置初始化
  delay(3000);
}
int x = 1;

void loop() {
  
  while(Serial.available())
  {
     x = Serial.parseInt();     //作为真空杯开关的判断参数
    if(x == 0)    //判别真空吸杯关条件
    {
        Serial.println("Set Arm Pump off");
        p1->pump_off();   //关闭真空吸杯
    }
    else if(x != 0)  //判别真空吸杯开条件
    {
      Serial.println("Set Arm Pump On");
        p1->pump_on();  //开启真空吸杯
    }
  }
}



