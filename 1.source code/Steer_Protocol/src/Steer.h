/**
  *************************************************************************************************
  * @file                 Steer.h
  * @author            
  * @qq                  269426626
  * @qq_group       604556405
  * @version           V2.0
  * @date               2017.08.11
  * @note    ′          此程序为舵机通信应用层函数
  *************************************************************************************************
  */


#ifndef STEER_H
#define STEER_H

#include <Arduino.h>
#include <Steer_protocol.h>

/*class里面函数的具体说明清参照 Steer.cpp */
/*暂未开放的功能是指舵机暂时不支持该功能，里面数据读出来都市0X00*/

class Steer:public Steer_protocol
{
  private:
  byte id;
  
  public:
  
  byte Electric_Current[2] = {0};                                                               //暂未开放
  byte voltage = 0;                                                                                  //暂未开放
  byte Temperature = 0;                                                                          //暂未开放
  byte Position_Current[2] = {0};                                                              //当前位置
  byte Position_Target[2] = {0};                                                                //目标位置
  byte RunTime[2] = {0};                                                                          //运行时间（一般与目标位置一起写入，形成速度控制）
  byte Speed_Current[2] = {0};                                                                //暂未开放
  byte Max_Angle_Limit[2] = {0};                                                             //最大角度限制
  byte Min_Angle_Limit[2] = {0};                                                              //最小角度限制
  
  Steer(byte ID=0 , HardwareSerial *serial = &Serial1);                            //构造函数初始化
  Steer();                            //构造函数初始化
  
  
  boolean Steer_Ping();
  //舵机应答函数
  
  void Set_object_ID(byte ID);
     
  void Set_Steer_Max_Angle_Limit(word max_angle);                              //设置舵机的最大角度限制，默认是4096，即360°
  void Set_Steer_Min_Angle_Limit(word min_angle);                                //设置舵机的最小角度限制，默认是0，即0°   //本机械臂默认2047为90°，那么0就是-90°
  
  void Set_Steer_Torque_On();                                                               //设置舵机的扭矩开关打开
  void Set_Steer_Torque_Off();                                                                //设置舵机的扭矩开关关闭
 
  void Set_Steer_position_runtime(word pos, word runtime);                  //设置舵机的目标位置与到达该位置的运行时间
  void Change_Steer_ID( byte new_id);                                                   //改变舵机的ID
  void Set_Steer_Reset();                                                                        //舵机复位：复位后ID = 0x01
    
  void Get_Steer_All_Inf();                                                                       //得到舵机的所有信息
    
  /*获取舵机信息函数API：：：：本来是想要把这些函数放到private，只是测了一下，如果调用一次Get_Steer_All_Inf()需
    要耗时13ms附近，而只需要获取部分信息，如调用下面的函数Get_Steer_Position_Current_Inf()仅仅耗时1.3ms附近，所以
    为了给那些对响应速度要求比较高的开发者使用，我们选择开放这些函数*/
    
  void Get_Steer_Electric_Current_Inf();                                                   //获取舵机的运行电流，该功能暂未开放
  void Get_Steer_voltage_Inf();                                                                //获取舵机的电压，该功能暂未开放
  void Get_Steer_Temperature_Inf();                                                       //获取舵机的温度，该功能暂未开放
  void Get_Steer_Position_Current_Inf();                                                 //获取舵机的当前位置
  void Get_Steer_Position_Target_Inf();                                                   //获取舵机的目标位置
  void Get_Steer_RunTime_Inf();                                                             //获取舵机的运行时间
  void Get_Steer_Speed_Current_inf();                                                    //该功能暂未开放
  void Get_Steer_Angle_Limit_inf();                                                        //获取舵机的最大/最小角度限制
  
  void Set_Steer_Torque();                                                                //设置舵机的扭矩开关关闭
};

#endif






