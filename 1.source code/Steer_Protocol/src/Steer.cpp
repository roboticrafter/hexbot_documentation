/**
  *************************************************************************************************
  * @file                 Steer.cpp
  * @author            
  * @qq                  269426626
  * @qq_group       604556405
  * @version           V2.0
  * @date               2017.08.13
  * @note    ′          此程序为舵机通信应用层函数
  *************************************************************************************************
  */

#include "Steer.h"



 /**
 * @brief   		                  舵机通讯应用层构造函数：初始化ID和通讯串口
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1
 *      				                      Steer   steer1( 0, &Serial1 );
 * @endcode
 * @param[in]                 id               舵机的ID号
 * @param[in]                Serial          串口选定(需要根据自己的开发板实际确定)
 *                                   @arg           Serial1
 *                                   @arg           Serial2
 *                                   @arg           Serial3
 */
Steer::  Steer()
{
  id = 0;
  Set_Serial_init(&Serial1);
}
 /**
 * @brief   		                  舵机通讯应用层构造函数：初始化ID和通讯串口
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1
 *      				                      Steer   steer1( 0, &Serial1 );
 * @endcode
 * @param[in]                 id               舵机的ID号
 * @param[in]                Serial          串口选定(需要根据自己的开发板实际确定)
 *                                   @arg           Serial1
 *                                   @arg           Serial2
 *                                   @arg           Serial3
 */
Steer::  Steer(byte ID=0 , HardwareSerial *serial = &Serial1)
{
  id = ID;
  Set_Serial_init(serial);
}

 /**
 * @brief   		            修改舵机实例化出来的舵机的id值（不改变舵机实际的id，修改对象中私有化的舵机的值）
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1
 *      				                      Steer   steer1( 0, &Serial1 );
 *											  steer1.Set_object_ID(2);
 * @endcode
 * @param[in]                 ID             修改的舵机的ID号
 */
void Steer::Set_object_ID(byte ID){
	 id = ID;
}


/**
 * @brief   		                  得到舵机的所有信息
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1           创建一个舵机对象：Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Get_Steer_All_Inf();
 * @endcode
 * @note                        一般需要获取信息时可以使用该函数，但是使用定时中断的请注意：该函数调用一次耗时13ms左右
 */
void Steer::Get_Steer_All_Inf()
{
  Get_Steer_Electric_Current_Inf();
  Get_Steer_voltage_Inf();
  Get_Steer_Temperature_Inf();
  Get_Steer_Position_Current_Inf();
  Get_Steer_Position_Target_Inf();
  Get_Steer_RunTime_Inf();
  Get_Steer_Speed_Current_inf();
  Get_Steer_Angle_Limit_inf();
}

/**
 * @brief   		                  舵机应答函数，测试舵机的通信是否正常
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1           创建一个舵机对象：Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Steer_Ping();
 * @endcode
 * @return                      ture为应答成功， false为失败
 */
boolean Steer::Steer_Ping()
{
   byte dat;
   return (ping(id, &dat));
}
    
/**
 * @brief   		                  设置舵机的最大角限制，建议设置在4096，即360°之内
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1           创建一个舵机对象：Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_Max_Angle_Limit(3072);
 * @endcode
 * @param[in]                 max_angle               最大限制角，即舵机能转到的最大值。4094为360°
 */ 
void Steer::Set_Steer_Max_Angle_Limit(word max_angle)
{
      byte wbuf[2];
      wbuf[0] = highByte(max_angle);
      wbuf[1] = lowByte(max_angle);
      write(id , 0X0b , wbuf , 2);
}

/**
 * @brief   		                  设置舵机的最小角限制，建议设置在4096，即360°之内
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1           创建一个舵机对象：Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_Min_Angle_Limit(100);
 * @endcode
 * @param[in]                 min_angle               最小限制角，即舵机能转到的最小值。0为0°， 4094为360°
 */ 
void Steer::Set_Steer_Min_Angle_Limit(word min_angle)
{
      byte wlbuf[2];
      wlbuf[0] = highByte(min_angle);
      wlbuf[1] = lowByte(min_angle);
      write(id , 0X09 , wlbuf , 2);    
}

/**
 * @brief   		                  设置舵机舵机扭矩开关为开状态，即舵机获得扭矩
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1           创建一个舵机对象：Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_Torque_On();
 * @endcode
 */   
void Steer::Set_Steer_Torque_On()
{
      byte buf[2];
      buf[0] = 0x01;
      buf[1] = 0x01;
      write(id , 0X28 , buf , 1);    
	  
}

/**
 * @brief   		                  设置舵机舵机扭矩开关为关状态，即舵机失去扭矩
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1           创建一个舵机对象：Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_Torque_Off();
 * @endcode
 */       
void Steer::Set_Steer_Torque_Off()
{
      byte buf[2];
      buf[0] = 0x01;
      buf[1] = 0x01;
      write(id , 0X28 , buf , 1);    
	  
	   byte buf1[2];
      buf1[0] = 0x03;
      buf1[1] = 0x20;
      write(id , 0X10 , buf1 , 2);
}
  
 /**
 * @brief   		                  舵机运行函数，配置该函数使舵机产生运动
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1           创建一个舵机对象：Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_position_runtime(2047， 1000);    //一秒钟运行到2047这个目标位置
 * @endcode
 * @param[in]                 pos               舵机的位置，值范围为最大角度限制与最小角度限制之间
 * @param[in]                 runtime         舵机的运行时间，1000为1秒
 */        
void Steer::Set_Steer_position_runtime(word pos, word runtime)
{
      word tmpPos = (word)4096 - pos;
	  //word tmpPos = (word)pos;
      byte dat[5];
      dat[0] = id;
      dat[1] = highByte(tmpPos);
      dat[2] = lowByte(tmpPos);
      dat[3] = highByte(runtime);
      dat[4] = lowByte(runtime); 
      sync_write(0x2a, dat, 1, 5);      
}
 
  /**
 * @brief   		                  改变舵机的ID
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1           创建一个舵机对象：Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Change_Steer_ID(2);           //把该舵机的id设置为2
 * @endcode
 * @param[in]                 new_id           舵机的新ID
 */        
void Steer::Change_Steer_ID( byte new_id)
{
        byte new_id_ = new_id;
        write(id, 0x05, &new_id_, sizeof(new_id_));
        id = new_id;
}

  /**
 * @brief   		                  舵机恢复出厂设置
 * @code                        示例
 *      				                      // 设置id = 0，初始化串口1           创建一个舵机对象：Steer   steer1( 0, &Serial1 );
 *      				                      steer1.Set_Steer_Reset();           
 * @endcode
 * @note                         复位后舵机的ID会被设为0X01
 */        
void Steer::Set_Steer_Reset()
{
   reset(id);
   id = 1;
}

  /**
 * @brief   		                  获得舵机的当前电流
 * @note                        获得的值存储在 Electric_Current[]数组里，具体的请看类定义里面的公有成员变量
 * @note                        由于舵机开发原因，该功能暂未开放，获取数据全为0X00
 */        
void Steer::Get_Steer_Electric_Current_Inf()
{
    read(id , 0X2e , Electric_Current , 2); 
}

  /**
 * @brief   		                  获得舵机的当前电压
 * @note                        获得的值存储在 voltage里，具体的请看类定义里面的公有成员变量
 * @note                        由于舵机开发原因，该功能暂未开放，获取数据全为0X00
 */        
 void Steer::Get_Steer_voltage_Inf()
 {
    byte tmp[2] = {0};
    read(id , 0X3e , tmp , 1);  
    voltage = tmp[0];
 }
 
   /**
 * @brief   		                  获得舵机的当前温度
 * @note                        获得的值存储在 Temperature里，具体的请看类定义里面的公有成员变量
 * @note                        由于舵机开发原因，该功能暂未开放，获取数据全为0X00
 */        
  void Steer::Get_Steer_Temperature_Inf()
  {
      byte tmpe[2] = {0};
      read(id , 0X3f , tmpe , 1); 
      Temperature = tmpe[0];
  }
  
  /**
 * @brief   		                  获得舵机的当前位置
 * @note                        获得的值存储在 Position_Current[]数组里，具体的请看类定义里面的公有成员变量
 */        
  void Steer::Get_Steer_Position_Current_Inf()
  {
      read(id , 0X38 , Position_Current , 2); 
  }
  
 /**
 * @brief   		                  获得舵机的目标位置
 * @note                        获得的值存储在 Position_Target[]数组里，具体的请看类定义里面的公有成员变量
 */        
  void Steer::Get_Steer_Position_Target_Inf()
  {
      read(id , 0X2a , Position_Target , 2); 
  }
  
 /**
 * @brief   		                  获得舵机的运行时间
 * @note                        获得的值存储在 RunTime[]数组里，具体的请看类定义里面的公有成员变量
 */        
  void Steer::Get_Steer_RunTime_Inf()
  {
      read(id , 0X2c , RunTime , 2); 
  }
  
 /**
 * @brief   		                  获得舵机的当前速度
 * @note                        获得的值存储在 Speed_Current[]数组里，具体的请看类定义里面的公有成员变量
 * @note                        由于舵机开发原因，该功能暂未开放，获取数据全为0X00
 */        
  void Steer::Get_Steer_Speed_Current_inf()
  {
      read(id , 0X3a , Speed_Current , 2); 
  }
  
    /**
 * @brief   		                  获得舵机的最大/最小角度限制
 * @note                        获得的值存储在 Min_Angle_Limit[]和Max_Angle_Limit[]两个数组里，具体的请看类定义里面的公有成员变量
 */        
  void Steer::Get_Steer_Angle_Limit_inf()
  {
      read(id , 0X09 , Min_Angle_Limit , 2); 
      read(id , 0X0b , Max_Angle_Limit , 2); 
  }
  
  
  void Steer::Set_Steer_Torque()
{
      byte buf[2];
      buf[0] = 0x01;
      write(id , 0X28 , buf , 1);    
	  
	  byte buf1[2];
      buf1[0] = 0x03;
      buf1[1] = 0x20;
      write(id , 0X10 , buf1 , 2);
}
  