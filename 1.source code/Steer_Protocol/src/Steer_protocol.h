/**
  *************************************************************************************************
  * @file                 Steer_protocol.h
  * @author            
  * @qq                  269426626
  * @qq_group       604556405
  * @version           V1.0
  * @date               2017.07.28
  * @note    ′          此程序为舵机通信底层函数
  *************************************************************************************************
  */


#ifndef STEER_PROTOCOL_H
#define STEER_PROTOCOL_H

#include<arduino.h>


#include <Arduino.h>

#define BROADCAST_ADDR                        0xFE                            //广播地址

//the macros below is instructions              指令
#define INSTRUCTION_PING                      0x01                            //查询指令
#define INSTRUCTION_READ_DATA           0x02                             //读指令
#define INSTRUCTION_WRITE_DATA          0x03                             //写指令
#define INSTRUCTION_REG_WRITE             0x04                             //异步写指令
#define INSTRUCTION_ACTION                  0x05                            //触发异步写指令
#define INSTRUCTION_RESET                     0x06                             //复位指令
#define INSTRUCTION_SYNC_WRITE          0x83                             //同步写指令


//the macros below is storaged in EEPROM      读写地址（EEPROM）以下地址的存储内容都是可读写的
#define ID_REG			                                     0x05                             //ID的地址：通过修改该位置的值，可以修改舵机的地址（地址范围:: 0 ~253）
#define MIN_ANGLE_LIMIT_H	                   0x09                             //最小角度限制的高字节存储位置
#define MIN_ANGLE_LIMIT_L	                   0x0A                             //最小角度限制的低字节存储位置
#define MAX_ANGLE_LIMIT_H	                   0x0B                             //最大角度限制的高字节存储位置
#define MAX_ANGLE_LIMIT_L	                   0x0C                             //最大角度限制的低字节存储位置
#define MAX_TORQUE_H		                         0x10                             //最大扭矩高字节存储位置
#define MAX_TORQUE_L		                         0x11                             //最大扭矩低字节存储位置
#define DEFAULT_SPEED		                         0x12                             //调整速度位置                         
#define MIDDLE_POSITION_H	                   0x14                             //中位调整高字节存储位置
#define MIDDLE_POSITION_L	                   0x15                             //中位调整低字节存储位置

//the macros below is storaged in RAM       RAM地址(部分可写)
#define TORQUE_SWITCH		                       0x28                              //扭矩开关（读写）  （1开，0关）
#define TARGET_POSITION_H	                   0x2A                              //目标位置高字节存储位置(读写)
#define TARGET_POSITION_L	                     0x2B                              //目标位置低字节存储位置(读写)
#define CURRENT_POSITION_H                  0x38                              //当前位置高字节存储位置(只读)
#define CURRENT_POSITION_L                   0x39                              //当前位置低字节存储位置(只读)
#define TARGET_SPEED		                           0x41                              //速度调整(读写

//写模式

#define REG_WRITE                                     0x01                               //异步写指令
#define SYNC_WRITE                                  0x02                               //同步写指令


/*class里面函数的具体说明清参照 Steer_protocol.cpp */

class Steer_protocol
{
  private:
    HardwareSerial *svSer;

  public:
     Steer_protocol() = default;
     Steer_protocol(HardwareSerial *serial , long timeout);
    void Set_Serial_init(HardwareSerial *serial );                                                                     //主要给子类一个修改串口的接口
     
    void begin(HardwareSerial *serial , long timeout );                                                          //仅在用户单单测试舵机时使用，用来和串口监视器通信
    
    byte Check_Sum(byte *buf, byte len);
     
    boolean ping(byte id, byte *data);
    boolean read(byte id, byte regStartAddr, byte *data, byte readlen);
    void write(byte id, byte regStartAddr, byte *buf, byte bufLen);
    void sync_write(byte regStartAddr, byte *buf, byte svNum, byte perDataLen);
    //void reg_write(byte regStartAddr, byte *buf, byte svNum, byte perDataLen);                //异步写
    //void act();                                                                                                                     //触发异步写
    void reset(byte id);
};

#endif






