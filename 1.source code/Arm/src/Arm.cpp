#include <Arm.h>

#define DEBUGno	


Arm MyArm;                                                                   //初始化一个机械臂对象

Steer_protocol  steer_tmp(&Serial1, 10);                         //初始化一个舵机底层对象，负责舵机检测


 /**
 * @brief   		                  机械臂配置初始化
 * @code                        示例      利用USB通信
 *      				                       MyArm.begin(USB_SER);
 * @endcode
 * @param[in]                 desireSer       输入的串口号
 *          @arg                  USB_SER            使用USB通信
 *          @arg                  WIFI_SER            使用WiFi通信
 * @return                       void        
 */
 
void Arm::begin(HardwareSerial *desireSer)         
{
    comSer = desireSer;
    comSer->begin(115200);
    comSer->setTimeout(5);

    Steer_Detect();
	
	#ifdef DEBUG
    Steer_Num = 7;
	#endif
	
	Serial.println(Steer_Num);
	
    Para_Init();
    Get_Offset();
}


 /**
 * @brief   		                  机械臂位置初始化
 * @note                        负责初始化舵机的位置
 * @code                        示例      初始化舵机位置到达指定位置
 *      				                       MyArm.position_init();
 * @endcode
 * @param[in]                 void
 * @return                      void        
 */
void Arm::position_init(void)
{
  Get_Offset();
  for(int i = 0; i < Steer_Num; i++)
  {
    steer[i] ->Set_Steer_position_runtime(2047+offPos[i] , 0x06b8);
    pos_goal[i] = 2047+offPos[i];
  }
}

 /**
 * @brief   		                  机械臂坐标位置逆解函数：由（x, y, z）的坐标逆解出机械臂的舵机0，1，2的弧度值
 * @code                        示例      解算（0,188,188）的运动学逆解，得到ID为0,1,2,的舵机的角度值
 *      				                       MyArm.inverse_movement(0, 188, 188);
 * @endcode
 * @param[in]                  x_    机械臂机体x轴的坐标位置（单位是毫米）
 * @param[in]                  y_    机械臂机体y轴的坐标位置
 * @param[in]                  z_    机械臂机体z轴的坐标位置
 * @return                       void        
 */
void Arm::inverse_movement(double x_ , double y_, double z_)
{
  double x = x_, y = y_, z = z_;

    theta[0] = atan2(y, x);
	#if 1
    x -= d * cos(theta[0]);
    y -= d * sin(theta[0]);
    z -= e;
    
    double lengthA = sqrt(x * x + y * y + z * z);
    double lengthC = sqrt(h * h + c * c);
    double offsetAngle = atan(h / c);
    double angleA = acos( (a * a + lengthC * lengthC - lengthA * lengthA) / (2 * a * lengthC) );
    double angleB = atan( z / sqrt(x * x + y * y) );
    double angleC = acos( (a * a + lengthA * lengthA - lengthC * lengthC) / (2 * a * lengthA) );
    theta[1] = angleB + angleC;
    theta[2] = PI - angleA - angleB - angleC + offsetAngle;
    theta[2] += HALF_PI;
	#endif
	
	//double lengthL = sqrt(x * x + y * y + z * z);
}


 /**
 * @brief   		                  机械臂坐标位置逆解程序：与inverse_movement(double x_ , double y_, double z_)函数重载，只是把坐标值放到了点向量中了
 * @code                        示例     PVector MyDot;     MyDot.set_xyz(0, 188, 188);
 *      				                       MyArm.inverse_movement(MyDot);
 * @endcode
 * @param[in]                  pt        为储存了坐标信息的点向量
 * @return                       void        
 */
void Arm::inverse_movement(PVector pt)
{
  double x = pt.x, y = pt.y, z = pt.z;

    theta[0] = atan2(y, x);
    x -= d * cos(theta[0]);
    y -= d * sin(theta[0]);
    z -= e;
    
    double lengthA = sqrt(x * x + y * y + z * z);
    double lengthC = sqrt(h * h + c * c);
    double offsetAngle = atan(h / c);
    double angleA = acos( (a * a + lengthC * lengthC - lengthA * lengthA) / (2 * a * lengthC) );
    double angleB = atan( z / sqrt(x * x + y * y) );
    double angleC = acos( (a * a + lengthA * lengthA - lengthC * lengthC) / (2 * a * lengthA) );
    theta[1] = angleB + angleC;
    theta[2] = PI - angleA - angleB - angleC + offsetAngle;
    theta[2] += HALF_PI;
}

 /**
 * @brief   		                  机械臂运动控制函数   重要：这个函数基本上可以控制机械臂的运动
 * @note                         把机械臂的末端运行到坐标(x_, y_, z_)，运行时间为runtime
 * @code                        示例     设机械臂末端由当前位置运行到（0， 188， 188）所需要的时间为2s
 *      				                       MyArm.move_to_position(0, 188, 188, 2000);
 * @endcode
 * @param[in]                  x_             机械臂机体x轴的坐标位置（单位是毫米）
 * @param[in]                  y_             机械臂机体y轴的坐标位置
 * @param[in]                  z_             机械臂机体z轴的坐标位置
 * @param[in]                  runtime    机械臂由当前位置运行到指定位置所花的时间，单位是毫秒
 * @return                       void        
 */

void Arm::move_to_position( double x_ , double y_, double z_ , word runtime)
{    
    inverse_movement(x_ , y_, z_);
	
	for(int i=0; i<3; i++){
		Serial.print(theta[i]);
		Serial.print("	");
	}
    
    #ifdef START_PROTECT____
        double thet[3];
        thet[0] = theta[0];
        thet[1] = theta[1];
        thet[2] = theta[2];
        boolean judge = MyArm_Protect.Position_Protect(thet, sizeof(thet)/sizeof(thet[0]));
        if(judge == false){ Serial.print("out of angle limit"); return false;}
    #endif
	
	if(judgeMust2angle() && runtime > 8){
	//if( runtime > 0){
		for(byte i = 0 ; i < 3; i++)   
		{
		  pos_goal[i] = Rad2Pos(theta[i]) + offPos[i];
		  steer[i] ->Set_Steer_position_runtime(pos_goal[i] , runtime);
	      //Serial.println(pos_goal[i]);
		}
	}
}

 /**
 * @brief   		                  重要：这个函数基本上可以控制机械臂的运动
 * @note                         把机械臂的末端运行到点向量处，运行时间为runtime
 * @code                        示例     设机械臂末端由当前位置运行到（0， 188， 188）所需要的时间为2s
*                                      PVector MyDot;     MyDot.set_xyz(0, 188, 188);
 *      				                       MyArm.move_to_position(MyDot, 2000);
 * @endcode
 * @param[in]                  pt            为储存了坐标信息的点向量
 * @param[in]                  runtime    机械臂由当前位置运行到指定位置所花的时间，单位是毫秒
 * @return                       void        
 */
void Arm::move_to_position( PVector pt , word runtime)
{    
    inverse_movement(pt);
    
    #ifdef START_PROTECT___
        double thet[3];
        thet[0] = theta[0];
        thet[1] = theta[1];
        thet[2] = theta[2];
        boolean judge = MyArm_Protect.Position_Protect(thet, sizeof(thet)/sizeof(thet[0]));
        if(judge == false){ Serial.print("out of angle limit"); return false;}
    #endif

	if(judgeMust2angle() && runtime > 0){
		for(byte i = 0 ; i < 3; i++)   
		{
		  pos_goal[i] = Rad2Pos(theta[i]) + offPos[i];
		  steer[i] ->Set_Steer_position_runtime(pos_goal[i], runtime);
		}
	}
}

 /**
 * @brief   		                  重要：这个函数基本上可以控制机械臂的运动
 * @note                         直接设置机械臂中舵机0,1,2的位置，运行时间为runtime
 * @code                        示例     使三个舵机运行到中间位置
 *      				                       MyArm.move_to_position(2047, 2047, 2047, 2000);
 * @endcode
 * @param[in]                  pos0             舵机0的位置，范围（0~4095），表示（0~360°）注意：：如果开启保护，以保护中的位限为主
 * @param[in]                  pos1             舵机1的位置，范围（0~4095），表示（0~360°）
 * @param[in]                  pos2             舵机2的位置，范围（0~4095），表示（0~360°）
 * @param[in]                  runtime         机械臂由当前位置运行到指定位置所花的时间，单位是毫秒
 * @return                       void        
 */
void Arm::move_to_position( word pos0 , word pos1, word pos2 , word runtime)
{
    #ifdef START_PROTECT
        double thet[3];
        thet[0] = Pos2Rad(pos0);
        thet[1] = Pos2Rad(pos1);
        thet[2] = Pos2Rad(pos2);
        boolean judge = MyArm_Protect.Position_Protect(thet, sizeof(thet)/sizeof(thet[0]));
        if(judge == false){ Serial.print("out of angle limit"); return false;}
    #endif
    
      steer[0] ->Set_Steer_position_runtime(pos0 + offPos[0] , runtime);
      steer[1] ->Set_Steer_position_runtime(pos1 +  offPos[1], runtime);
      steer[2] ->Set_Steer_position_runtime(pos2 +  offPos[2], runtime);
}

 /**
 * @brief   		                  舵机响应检测函数
 * @note                         检测到响应舵机的数量
 * @code                        示例     
 *      				                       MyArm.Steer_Detect();
 * @endcode
 * @return                       void        
 */
byte Arm::Steer_Detect()
{
    byte state;
    byte num = 0;

    for (byte i = 0; i < 7; i++) 
    {
        if (steer_tmp.ping(i, &state)) 
        {
            num ++;
        }
    }
    Steer_Num = num;
    return num;
}


 /**
 * @brief   		                  机械臂参数初始化函数
 * @code                        示例     
 *      				                       MyArm.Para_Init();
 * @endcode
 * @return                       void        
 */
 void Arm::Para_Init()
 {
    if(Steer_Num > 0)
    {
          offPos = (short *)malloc(Steer_Num * sizeof(short));
          theta = (double *)malloc(Steer_Num * sizeof(double));
          
          pos_goal = (int *)malloc(Steer_Num * sizeof(int));
          
          for(byte i  = 0; i < Steer_Num; i++ )
          {
            steer[i] = new Steer( i, &Serial1); 
          }
    }
 }
 
  /**
 * @brief   		                  机械臂扭矩开启函数  设置机械臂的扭矩为开：使机械臂恢复扭矩
 * @code                        示例     
 *      				                       MyArm.Set_Arm_Torque_On();
 * @endcode
 * @return                       void        
 */
void Arm::Set_Arm_Torque_On(void)
{
  for( int i = 0; i < Steer_Num; i++)
  {
    steer[i] -> Set_Steer_Torque_On();
  }
}

  /**
 * @brief   		                  机械臂扭矩关闭函数  设置机械臂的扭矩为关：使机械臂处于无力状态
 * @code                        示例     
 *      				                       MyArm.Set_Arm_Torque_Off();
 * @endcode
 * @return                       void        
 */
void Arm::Set_Arm_Torque_Off(void)
{
  for( int i = 0; i < Steer_Num; i++)
  {
    steer[i] -> Set_Steer_Torque_Off();
  }
}

 /**
 * @brief   		                   第3，第4，第5号舵机旋转运动函数    重要：舵机345的状态设置函数
 * @note                         前面的机械臂的位置函数主要是用于确定舵机的末端位置，这个函数中、的作用是使舵机345运动，旋转末端机构
 * @code                        示例     使3，4，5三个舵机运行到中间位置，运动时间为两秒
 *      				                       MyArm.turn_steer_345_to_positon(2047, 2047, 2047, 2000);
 * @endcode
 * @param[in]                  pos3             舵机3的位置，范围（0~4095），表示（0~360°）注意：：如果开启保护，以保护中的位限为主
 * @param[in]                  pos4             舵机4的位置，范围（0~4095），表示（0~360°）
 * @param[in]                  pos5             舵机5的位置，范围（0~4095），表示（0~360°）
 * @param[in]                  runtime         机械臂由当前位置运行到指定位置所花的时间，单位是毫秒
 * @return                       boolean         角度超出限制返回false        
 */
boolean Arm::turn_steer_345_to_positon(word pos3 , word pos4, word pos5 , word runtime)
{
      #ifdef START_PROTECT
        double thet[3];
        thet[0] = Pos2Rad(pos3);
        thet[1] = Pos2Rad(pos4);
        thet[2] = Pos2Rad(pos5);
        boolean judge = MyArm_Protect.steer_345_angle_protect(thet, sizeof(thet)/sizeof(thet[0]));
        if(judge == false){ Serial.print("out of angle limit"); return false;}
      #endif

      steer[3] ->Set_Steer_position_runtime(pos3 + offPos[3] , runtime);
      steer[4] ->Set_Steer_position_runtime(pos4 +  offPos[4], runtime);
      steer[5] ->Set_Steer_position_runtime(pos5 +  offPos[5], runtime);
}


 /**
 * @brief   		                   重要：舵机345的状态设置重载函数
 * @note                         前面的机械臂的位置函数主要是用于确定舵机的末端位置，这个函数中、的作用是使舵机345运动，旋转末端机构
 * @code                        示例     使3，4，5三个舵机运行到中间位置，运动时间为两秒
 *      				                       MyArm.turn_steer_345_to_positon(90, 90, 90, 2000);
 * @endcode
 * @param[in]                  angle3             舵机3的角度，范围（0~360） 注意：：如果开启保护，以保护中的位限为主
 * @param[in]                  angle4             舵机4的角度，范围（0~360） 
 * @param[in]                  angle5             舵机5的角度，范围（0~360） 
 * @param[in]                  runtime           机械臂由当前位置运行到指定位置所花的时间，单位是毫秒
 * @return                       boolean           保护超出限度返回false       
 */
boolean Arm::turn_steer_345_to_positon(double angle3 , double angle4, double angle5 , word runtime)
{
      #ifdef START_PROTECT
        double thet[3];
        thet[0] = Angle2Rad(angle3);
        thet[1] = Angle2Rad(angle4);
        thet[2] = Angle2Rad(angle5);
        boolean judge = MyArm_Protect.steer_345_angle_protect(thet, sizeof(thet)/sizeof(thet[0]));
        if(judge == false){ Serial.print("out of angle limit"); return false;}
      #endif
      steer[3] ->Set_Steer_position_runtime(Angle2Pos(angle3) +  offPos[3] , runtime);
      steer[4] ->Set_Steer_position_runtime(Angle2Pos(angle4) +  offPos[4], runtime);
      steer[5] ->Set_Steer_position_runtime(Angle2Pos(angle5) +  offPos[5], runtime);
}

 /**
 * @brief   		                             重要：舵机7的状态设置重载函数
 * @note                                         前面的机械臂的位置函数主要是用于确定舵机的末端位置，这个函数中、的作用是使舵机7运动，旋转末端机构
 * @code                       示例              使舵机7运行到中间位置，运动时间为两秒
 *      				                         MyArm.turn_steer7_to_positon(2047, 2000);
 * @endcode
 * @param[in]                  pos6              舵机5的位置信息，范围（1024~3071） 
 * @param[in]                  runtime           机械臂由当前位置运行到指定位置所花的时间，单位是毫秒
 * @return                     boolean           保护超出限度返回false       
 */
boolean Arm::turn_steer6_to_positon(word pos6, word runtime) {
	steer[6] ->Set_Steer_position_runtime(pos6 + offPos[6] , runtime);
}

 /**
 * @brief   		                             重要：舵机7的状态设置重载函数
 * @note                                         前面的机械臂的位置函数主要是用于确定舵机的末端位置，这个函数中、的作用是使舵机7运动，旋转末端机构
 * @code                       示例              使舵机7运行到中间位置，运动时间为两秒
 *      				                         MyArm.turn_steer7_to_positon(2047, 2000);
 * @endcode
 * @param[in]                  angle6            舵机5的角度信息，范围（0~360） 
 * @param[in]                  runtime           机械臂由当前位置运行到指定位置所花的时间，单位是毫秒
 * @return                     boolean           保护超出限度返回false       
 */
boolean Arm::turn_steer6_to_positon(double angle6 , word runtime)
{
     steer[6] ->Set_Steer_position_runtime(Angle2Pos(angle6) +  offPos[6], runtime);
}

boolean Arm::turn_steer_to_positon(byte id, word pos, word runtime) {
	if(runtime > 10)
	steer[id] ->Set_Steer_position_runtime(pos, runtime);
}

boolean Arm::turn_steer_to_positon(byte id, double angle , word runtime)
{
	if(runtime > 0)
     steer[id] ->Set_Steer_position_runtime(Angle2Pos(angle) +  offPos[id], runtime);
}

  /**
 * @brief   		                  得到机械臂的偏置函数
 * @code                        示例     
 *      				                       MyArm.Get_Offset();
 * @endcode
 * @return                       void        
 */
void Arm::Get_Offset()
{
    for (byte i = 0; i < Steer_Num; i++) 
    {
        EEPROM.get(i * sizeof(short), offPos[i]);
    }
}

  /**
 * @brief   		                  通过机械臂的直接位置，设置机械臂偏置    通过机械臂的直接数据设置机械臂偏置函数
 * @code                        示例     设置ID为1的舵机的偏置为100
 *      				                       MyArm.offset_by_pos(1， 100);
 * @endcode
 * @param[in]                  id              舵机的ID号
 * @param[in]                  offset        设置舵机的偏置，值的范围（-2046 ~ +2046）
 * @return                       void        
 */
void Arm::offset_by_pos(byte id, short offset)
{
      if(id < Steer_Num)
      {
        EEPROM.put(id * sizeof(short), offset);
      }
      else 
      {
        comSer -> println("error: id > Steer_Num");
        comSer -> println("Please enter a correct id");
      }
}

  /**
 * @brief   		                  通过机械臂的角度，设置机械臂偏置
 * @code                        示例     设置ID为1的舵机的偏置为10°
 *      				                       MyArm.offset_by_pos(1， 10);
 * @endcode
 * @param[in]                  id             舵机的ID号
 * @param[in]                  angle        设置舵机的偏置角度，值的范围（-90 ~ +90）
 * @return                       void        
 */
void Arm::offset_by_angle(byte id, double angle)
{
  short tmp = mapFloat(angle, -90, 90, -1024, 1024);
  offset_by_pos(id, tmp);
}

 /**
 * @brief   		                  数值映射函数    
 * @param[in]                 val                     需要映射的值
 * @param[in]                 in_min                原区间的最小值            
 * @param[in]                 in_max               原区间的最大值
 * @param[in]                 out_min              需要映射到的区间的最小值
 * @param[in]                 out_max             需要映射到的区间的最大值
 * @return                                               返回值：返回映射后的值
 */
double mapFloat(double val, double in_min, double in_max, double out_min, double out_max)
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

 /**
 * @brief   		                  弧度转位置数据（位置数据为舵机的直接控制数据）    
 * @code                        示例  
 *      		                          MyArm.Rad2Pos( 2.12);		                        
 * @endcode
 * @param[in]                 rad                    弧度值
 * @return                                               返回值：舵机的位置数据
 */
short Arm::Rad2Pos(double rad)
{
    return short(mapFloat(rad, 0, PI, 1024, 3071));
}

 /**
 * @brief   		                  角度转位置数据（位置数据为舵机的直接控制数据）    
 * @code                        示例  
 *      		                          MyArm.Angle2Pos( 90 );		                        
 * @endcode
 * @param[in]                 angle                 角度值
 * @return                                               返回值：舵机的位置数据
 */
short Arm::Angle2Pos(double angle)
{
    return short(mapFloat(angle, 0, 180, 1024, 3071));
}

/**
 * @brief   		                  位置数据转角度数据   
 * @code                        示例  
 *      		                          MyArm.Pos2Angle( 90 );		                        
 * @endcode
 * @param[in]                 pos                   位置数据
 * @return                                               返回值：角度数据
 */
double Arm::Pos2Angle(double pos)
{
    return double(mapFloat(pos, 1024, 3071, 0, 180));
}

/**
 * @brief   		                  弧度值转角度值函数  
 * @code                        示例  
 *      		                          MyArm.Rad2Angle( 2.12 );		                        
 * @endcode
 * @param[in]                 rad                    弧度数据
 * @return                                               返回值：角度数据
 */
double Arm::Rad2Angle(double rad)
{
    return double(mapFloat(rad, 0, PI, 0, 180));
}

/**
 * @brief   		                  角度值转弧度值   
 * @code                        示例  
 *      		                          MyArm.Angle2Rad( 98 );		                        
 * @endcode
 * @param[in]                 angle                 角度数据
 * @return                                               返回值：弧度数据
 */
double Arm::Angle2Rad(double angle)
{
    return double(mapFloat(angle, 0, 180, 0, PI));
}

/**
 * @brief   		                  直接位置数据转换弧度值函数
 * @code                        示例  
 *      		                          MyArm.Rad2Angle( 2.12 );		                        
 * @endcode
 * @param[in]                 rad                    弧度数据
 * @return                                               返回值：角度数据
 */
double Arm::Pos2Rad(word pos)
{
   return double(mapFloat(pos, 1024, 3071, 0, PI));
}

//利用速度去做整体调整

/**
 * @brief   		                  获取机械臂所有舵机的角度信息
 * @code                        示例  ： double angle[6];
 *      		                          MyArm.Arm_angle(angle);		                        
 * @endcode
 * @param[in]                 angle                 保存的角度数组
 * @return                                               返回值：void
 */
void Arm::get_Arm_angle(double angle[])
{
  word tmp_data = 0;
  for (byte i = 0; i < Steer_Num; i++) 
    {
       steer[i]->Get_Steer_Position_Current_Inf();
       //delay(500);
       //Serial.println(steer[i]->Position_Current[0] );
       //Serial.println(steer[i]->Position_Current[1] );
       //Serial.println();
      tmp_data = (word)(steer[i]->Position_Current[0]) *256 + steer[i]->Position_Current[1];
      angle[i] = Pos2Angle(tmp_data - offPos[i]);
    }
}

/**
 * @brief   		                  获取机械臂所有舵机的直接位置信息
 * @code                        示例  ： word pos[6];
 *      		                          MyArm.Arm_pos(pos);		                        
 * @endcode
 * @param[in]                 pos                 保存的直接位置数组
 * @return                                               返回值：void
 */
void Arm::get_Arm_pos(word pos[])
{
  for (byte i = 0; i < Steer_Num; i++) 
    {
       steer[i]->Get_Steer_Position_Current_Inf();
      pos[i] = (word)(steer[i]->Position_Current[0]) *256 + steer[i]->Position_Current[1];
    }
}


//t2 - 90
//180 - t1
//270 - t1 - t2
boolean Arm::judgeMust2angle(){
  if(judgeTheta()){
      double must2angle = 4.71 - MyArm.theta[1] - MyArm.theta[2];
      #ifdef DEBUG
      Serial.println("must2angle = ");
      Serial.println(must2angle/3.14*180);
      #endif
      if(must2angle > MIN2ANGLE && must2angle < MAX2ANGLE){  //判断两个大边夹角
          if(MyArm.theta[1] > -0.8 && MyArm.theta[1] < 2.79){ //判断第一大边夹角
            //计算mainangle
            double mainangle = ( MyArm.theta[1] - asin( (176.22*sin(must2angle)) / (sqrt(67362.8 - 67157.442*cos(must2angle))) ) )/3.14*180;
            #ifdef DEBUG
            Serial.println();
            Serial.print("mainangle = ");
            Serial.println(mainangle);
            #endif
            if( mainangle < MAXMAINANGLE){   //判断主角度，就是末端点到原点的连线与地面的夹角
               return true;
            }
			Serial.println("STYLE:4 :Beyond the limit of distance");
            return false;
          }
		  Serial.println("STYLE:3 :Beyond the limit of distance");
          return false;
       }
	   Serial.println("STYLE:2 :Beyond the limit of distance");
       return false;
  }
  Serial.println("STYLE:1 :Beyond the limit of distance");
  return false;
}

boolean Arm::judgeTheta(){
	word pos_goalt[3] = {0};
     if(MyArm.Steer_Num < 3){return false;}
     else{
        for( int i=0; i < 3; i++){
		{
		  pos_goalt[i] = Rad2Pos(theta[i]) + offPos[i];
	      Serial.println(pos_goalt[i]);
		}
          if(!((MyArm.theta[i] >= -1) && (MyArm.theta[i] <= 3.14))){
            return false;
          }   
        }
      }
      return true;
 }
 
  #define OFFSET_FRONT 81.0
  #define OFFSET_ROT 11.0
  #define OFFSET_L (OFFSET_FRONT+OFFSET_ROT)
  
  #define START_X_ANGLE 90.0
  #define START_Y_ANGLE 90.0
  #define START_Z_ANGLE 90.0	

  #define START_X 107.45
  #define START_Y 0 
  #define START_Z 138.13
 
 void Arm::line_to_desxyz_zwc(){
	 float target[3];
	 
	 for(int i=0; i<3; i++){
		 target[i] = des_xyz[i];
		Serial.print("des_xyz	");
		Serial.print(des_xyz[i]);
	 }Serial.println();
	 
	 float difference[3];
	 for(int i=0; i<3; i++){
	  	difference[i] = target[i] - cur_xyz[i];
		Serial.print("cur_xyz	");
		Serial.print(cur_xyz[i]);
	 }Serial.println();
	 
	 float cartesian_mm = sqrt(sq(difference[0]) + sq(difference[1]) + sq(difference[2]));
	 if (cartesian_mm < 0.000001) return false;

	 float _feedrate_mm_s = 200.0;
	 float seconds = cartesian_mm / _feedrate_mm_s;
	 
	 int steps = max(1, int(250 * seconds));
	 Serial.print("steps: ");
	 Serial.println(steps);
	 Serial.println((int)(seconds*10));
	 
	 float inv_steps = 1.0/steps;
	 for (int s = 1; s <= steps; s++) {
  
		float fraction = float(s) * inv_steps;
		
		 for(int i=0; i<3; i++){
		  target[i] = cur_xyz[i] + difference[i] * fraction;
		  Serial.print("target	");
		  Serial.print(target[i]);
	     }Serial.println();
		 
		int time = (int)(seconds*10);
		
		if(time <= 1) time=1;
		move_to_position((double)target[0], target[1], target[2],time);
		turn_steer_to_positon(4, (theta[2]*57.3 + 5) , time);
		delay(time);
	 }
	 
	 copy_des_to_current_xyz();
 }
 
void Arm::copy_des_to_current_xyz(){
	for(int i=0; i<3; i++){
		cur_xyz[i] = des_xyz[i];
	}
}
 
void Arm::current_xyz_init(){
		cur_xyz[0] = 0.0;
		cur_xyz[1] = 176.22;
		cur_xyz[2] = 190.55;
}

 //176.22  0.00  190.55 
 
 
 
 
 
 
 
