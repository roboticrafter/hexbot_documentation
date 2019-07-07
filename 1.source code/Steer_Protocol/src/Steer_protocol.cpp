/**
  *************************************************************************************************
  * @file                 Steer_protocol.cpp
  * @author            
  * @qq                  269426626
  * @qq_group       604556405
  * @version           V1.0
  * @date               2017.07.28
  * @note    ′          此程序为舵机通信底层函数
  *************************************************************************************************
  */

#include "Steer_protocol.h"


 /**
 * @brief   		                  舵机通讯协议构造函数：初始化通讯串口
 * @code                        示例
 *      				                      // 初始化串口1，超时10
 *      				                      Steer_protocol   Steer1( &Serial1 , 10 );
 * @endcode
 * @param[in]                Serial          串口选定
 * @param[in]                 timeout      等待超时
 */
Steer_protocol::Steer_protocol(HardwareSerial *serial, long timeout)
{
  svSer = serial;
  svSer -> begin(115200);
  svSer -> setTimeout(timeout);
}

 /**
 * @brief   		                  初始化通讯串口
 * @param[in]                Serial          串口选定
 */
void Steer_protocol::Set_Serial_init(HardwareSerial *serial )
{
   svSer = serial;
  svSer -> begin(115200);
  svSer -> setTimeout(10);
}

 /**
 * @brief   		                  用户仅在测试舵机的时候使用
 * @code                        示例      Steer_protocol   Steer1( &Serial1 , 10 );
 *      				                      Steer1.begin();
 * @endcode
 * @param[in]                Serial          串口选定
 *                                   @arg 
 *                                   @arg 
 *                                   @arg 
 * @param[in]                 timeout      等待超时
 */
void Steer_protocol::begin(HardwareSerial *serial, long timeout)
{
  serial ->begin (115200);
  serial ->setTimeout(timeout);
}

 /**
 * @brief   		                  校验和函数：校验和公式：
 *                                    Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
 * @code                        示例
 *      				                      // Steer_protocol Steer1; 输出数据buf的最后一个数字为校验和   byte buf[] = {1,2,3,4,5,6}    此时 len = 5
 *      				                      Steer1.Check_Sum(buf , 5);           
 *                                    return  ~（3+4+5）
 * @endcode
 * @param[in]                 buf          需要校验的数组地址
 * @param[in]                 len           数组长度： 如示例的长度为5
 * @return                      sum         返回值：校验和
 */
byte Steer_protocol::Check_Sum(byte *buf, byte len)
{
    int    tmp = 0;
    byte  cut = 0;
    byte  sum = 0;
    
    for(cut = 2; cut < len; cut++)
    {
        tmp += buf[cut];
    }
    sum = (byte)((~tmp) & 0xff );
    return sum;
}

 /**
 * @brief   		                  工作状态查询函数    
 * @code                        示例  
 *      				                      //Steer_protocol Steer1; byte *dat; 查询 ID 为 1 的舵机 的工作状态
 *      				                      Steer1.ping(0x01 , dat);       
 * @endcode
 * @param[in]                 id             舵机的ID
 * @param[in]                 data         输入类型为byte的指针，获得舵机工作状态
 * @return                      true:         返回值：舵机有应答，通信正常
 *                                    false:        返回值：舵机无应答，通信不正常
 */
 boolean Steer_protocol::ping(byte id, byte *data)
 {
    byte buf[6] = {0xff , 0xff, id , 0x02 , 0x01, (byte)( (~(id+3)) & (0xff) )};

    while(svSer -> availableForWrite() < 6); 
    svSer -> write(buf, 6);
    svSer -> flush();
             
     byte cnt = svSer -> readBytes(buf , 6);
    
     if((cnt == 6)  && (buf[0] == 0xff) && (buf[1] == 0xf5) && (buf[2] == id) 
          && (buf[3] == 0x06) && (buf[5] == Check_Sum( buf, 5 )) )
     {
            *data = buf[4];
            return true;
     }
     else
     {
        return false;
     }         
  }

 /**
 * @brief   		                  读取舵机状态函数    print
 * @code                        示例  
 *      				                      //舵机ID = 1；在控制表里从地址 0X38 处读取二个字节,Steer_protocol Steer1; byte dat[2];
 *      				                      Steer1.read(0x01 , 0x38 , dat , 0x02);       
 * @endcode
 * @param[in]                 id                        舵机的ID
 * @param[in]                 regStartAddr       需读取信息的内存开始地址
 * @param[in]                 data                    输入类型为byte的指针，存储读取的信息
 * @return                      true:                    返回值为真：读取成功
 *                                    false:                   返回值为假：读取失败
 */
boolean Steer_protocol::read(byte id, byte regStartAddr, byte *data, byte readlen)
{
      byte writebuf[8] = {0xff, 0xff, id, 0x04, 0x02, regStartAddr, readlen, (byte)( (~(id + 6 +regStartAddr +readlen)) & 0xff ) };
      
      while(svSer -> availableForWrite() < 8);
  
      svSer -> write(writebuf , 8);
      svSer -> flush();


      byte readbuf[readlen + 6]; 
      
      byte cnt = svSer -> readBytes( readbuf , readlen + 6);

       if( (cnt == ( readlen + 6)) && ( readbuf[0] == 0xff) && ( readbuf[1] == 0xf5) && ( readbuf[2] == id) && (readbuf[3] == readlen + 2) 
       && ( readbuf[readlen + 5] == (byte)( Check_Sum( readbuf , readlen + 5) )))
      {
          memcpy(data, &readbuf[5], readlen);
            return true;
  }
  return false;

}

 /**
 * @brief   		                  舵机复位函数    
 * @code                        示例  
 *      				                      //对ID = 1 的舵机进行复位 , Steer_protocol Steer1 ;
 *      				                      Steer1.reset(0x01) :       
 * @endcode
 * @param[in]                 id            舵机的ID
 */
void Steer_protocol::reset(byte id)
{
  byte buf[6] = { 0xff, 0xff, id, 0x2, 0x6, (byte)((~(id+8)) & 0xff ) };

  while(svSer -> availableForWrite() < 6);
  svSer -> write(buf, 6);
  svSer -> flush();
}

 /**
 * @brief   		                  写函数    
 * @code                        示例  
 *      				                      //舵机ID = 1；在控制表里从地址 0X2A 处写入二个字节，为目标位置,Steer_protocol Steer1; byte dat[2] = { 0x00 , 0xff };
 *      				                      Steer1.write( 0x01 , 0X2A , dat , 0x02 );       
 * @endcode
 * @param[in]                 id                         舵机的ID
 * @param[in]                 regStartAddr        需写入信息的内存开始地址
 * @param[in]                 buf                       输入类型为byte的数组指针，存储需要写的信息
 * @param[in]                 bufLen                 buf的数组长度
 */
void Steer_protocol::write(byte id, byte regStartAddr, byte *buf, byte bufLen)
{
  byte writeBufLen = bufLen + 7;
  byte writeBuf[writeBufLen] = {0xff, 0xff, id, (byte)(bufLen+3), INSTRUCTION_WRITE_DATA, regStartAddr};
  memcpy(&writeBuf[6], buf, bufLen);
  writeBuf[writeBufLen - 1] = Check_Sum(writeBuf, writeBufLen - 1);

  while(svSer -> availableForWrite() < writeBufLen);
  svSer -> write(writeBuf, writeBufLen);
  svSer -> flush();
  /*
  Serial.println( "writeBuf");
  for( int i = 0; i < writeBufLen; i++)
  {
    Serial.print( "   0X");
    Serial.print( writeBuf[ i ] , HEX);
  }
    Serial.println( "");
    Serial.println( "writeBuf over");*/
  }


 /**
 * @brief   		                  同步写函数（广播）:多个舵机执行相同功能（功能相同，数据可以不同，例如可以同时设定目标位置）    
 * @code                        示例  
 *      				                      //在控制表里从地址 0X2A 处写入二个字节，为目标位置,Steer_protocol Steer1; byte dat[] = { 0x01,0x00 , 0xff........0x06,0x00,0xff  };
 *      				                      Steer1.sync_write( 0X2A , buf , 6 , 3);       
 * @endcode
 * @param[in]                 regStartAddr        需写入信息的内存开始地址
 * @param[in]                 buf                       输入类型为byte的数组指针，存储需要写的信息
 * @param[in]                 svNum                 输入类型为byte的数组指针，存储需要写的信息
 * @param[in]                 perDataLen           写入数据的长度+1（为ID号）
 */
void Steer_protocol::sync_write(byte regStartAddr, byte *buf, byte svNum, byte perDataLen)
{
  byte writeBufLen = perDataLen * svNum + 8;
  
  byte writeBuf[writeBufLen] = {0xff, 0xff, BROADCAST_ADDR, (byte)((writeBufLen - 4) & 0xff),
                                INSTRUCTION_SYNC_WRITE, regStartAddr, (byte)((perDataLen - 1) & 0xff )};
                                
  memcpy(&writeBuf[7], buf, writeBufLen - 8);
  writeBuf[writeBufLen - 1] = Check_Sum(writeBuf, writeBufLen - 1);

  while(svSer -> availableForWrite() < writeBufLen);
  svSer -> write(writeBuf, writeBufLen);
  svSer -> flush();
}










