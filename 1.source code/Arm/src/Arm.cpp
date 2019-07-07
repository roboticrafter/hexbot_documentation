#include <Arm.h>

#define DEBUGno	


Arm MyArm;                                                                   //��ʼ��һ����е�۶���

Steer_protocol  steer_tmp(&Serial1, 10);                         //��ʼ��һ������ײ���󣬸��������


 /**
 * @brief   		                  ��е�����ó�ʼ��
 * @code                        ʾ��      ����USBͨ��
 *      				                       MyArm.begin(USB_SER);
 * @endcode
 * @param[in]                 desireSer       ����Ĵ��ں�
 *          @arg                  USB_SER            ʹ��USBͨ��
 *          @arg                  WIFI_SER            ʹ��WiFiͨ��
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
 * @brief   		                  ��е��λ�ó�ʼ��
 * @note                        �����ʼ�������λ��
 * @code                        ʾ��      ��ʼ�����λ�õ���ָ��λ��
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
 * @brief   		                  ��е������λ����⺯�����ɣ�x, y, z��������������е�۵Ķ��0��1��2�Ļ���ֵ
 * @code                        ʾ��      ���㣨0,188,188�����˶�ѧ��⣬�õ�IDΪ0,1,2,�Ķ���ĽǶ�ֵ
 *      				                       MyArm.inverse_movement(0, 188, 188);
 * @endcode
 * @param[in]                  x_    ��е�ۻ���x�������λ�ã���λ�Ǻ��ף�
 * @param[in]                  y_    ��е�ۻ���y�������λ��
 * @param[in]                  z_    ��е�ۻ���z�������λ��
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
 * @brief   		                  ��е������λ����������inverse_movement(double x_ , double y_, double z_)�������أ�ֻ�ǰ�����ֵ�ŵ��˵���������
 * @code                        ʾ��     PVector MyDot;     MyDot.set_xyz(0, 188, 188);
 *      				                       MyArm.inverse_movement(MyDot);
 * @endcode
 * @param[in]                  pt        Ϊ������������Ϣ�ĵ�����
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
 * @brief   		                  ��е���˶����ƺ���   ��Ҫ��������������Ͽ��Կ��ƻ�е�۵��˶�
 * @note                         �ѻ�е�۵�ĩ�����е�����(x_, y_, z_)������ʱ��Ϊruntime
 * @code                        ʾ��     ���е��ĩ���ɵ�ǰλ�����е���0�� 188�� 188������Ҫ��ʱ��Ϊ2s
 *      				                       MyArm.move_to_position(0, 188, 188, 2000);
 * @endcode
 * @param[in]                  x_             ��е�ۻ���x�������λ�ã���λ�Ǻ��ף�
 * @param[in]                  y_             ��е�ۻ���y�������λ��
 * @param[in]                  z_             ��е�ۻ���z�������λ��
 * @param[in]                  runtime    ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
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
 * @brief   		                  ��Ҫ��������������Ͽ��Կ��ƻ�е�۵��˶�
 * @note                         �ѻ�е�۵�ĩ�����е���������������ʱ��Ϊruntime
 * @code                        ʾ��     ���е��ĩ���ɵ�ǰλ�����е���0�� 188�� 188������Ҫ��ʱ��Ϊ2s
*                                      PVector MyDot;     MyDot.set_xyz(0, 188, 188);
 *      				                       MyArm.move_to_position(MyDot, 2000);
 * @endcode
 * @param[in]                  pt            Ϊ������������Ϣ�ĵ�����
 * @param[in]                  runtime    ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
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
 * @brief   		                  ��Ҫ��������������Ͽ��Կ��ƻ�е�۵��˶�
 * @note                         ֱ�����û�е���ж��0,1,2��λ�ã�����ʱ��Ϊruntime
 * @code                        ʾ��     ʹ����������е��м�λ��
 *      				                       MyArm.move_to_position(2047, 2047, 2047, 2000);
 * @endcode
 * @param[in]                  pos0             ���0��λ�ã���Χ��0~4095������ʾ��0~360�㣩ע�⣺����������������Ա����е�λ��Ϊ��
 * @param[in]                  pos1             ���1��λ�ã���Χ��0~4095������ʾ��0~360�㣩
 * @param[in]                  pos2             ���2��λ�ã���Χ��0~4095������ʾ��0~360�㣩
 * @param[in]                  runtime         ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
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
 * @brief   		                  �����Ӧ��⺯��
 * @note                         ��⵽��Ӧ���������
 * @code                        ʾ��     
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
 * @brief   		                  ��е�۲�����ʼ������
 * @code                        ʾ��     
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
 * @brief   		                  ��е��Ť�ؿ�������  ���û�е�۵�Ť��Ϊ����ʹ��е�ۻָ�Ť��
 * @code                        ʾ��     
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
 * @brief   		                  ��е��Ť�عرպ���  ���û�е�۵�Ť��Ϊ�أ�ʹ��е�۴�������״̬
 * @code                        ʾ��     
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
 * @brief   		                   ��3����4����5�Ŷ����ת�˶�����    ��Ҫ�����345��״̬���ú���
 * @note                         ǰ��Ļ�е�۵�λ�ú�����Ҫ������ȷ�������ĩ��λ�ã���������С���������ʹ���345�˶�����תĩ�˻���
 * @code                        ʾ��     ʹ3��4��5����������е��м�λ�ã��˶�ʱ��Ϊ����
 *      				                       MyArm.turn_steer_345_to_positon(2047, 2047, 2047, 2000);
 * @endcode
 * @param[in]                  pos3             ���3��λ�ã���Χ��0~4095������ʾ��0~360�㣩ע�⣺����������������Ա����е�λ��Ϊ��
 * @param[in]                  pos4             ���4��λ�ã���Χ��0~4095������ʾ��0~360�㣩
 * @param[in]                  pos5             ���5��λ�ã���Χ��0~4095������ʾ��0~360�㣩
 * @param[in]                  runtime         ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
 * @return                       boolean         �Ƕȳ������Ʒ���false        
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
 * @brief   		                   ��Ҫ�����345��״̬�������غ���
 * @note                         ǰ��Ļ�е�۵�λ�ú�����Ҫ������ȷ�������ĩ��λ�ã���������С���������ʹ���345�˶�����תĩ�˻���
 * @code                        ʾ��     ʹ3��4��5����������е��м�λ�ã��˶�ʱ��Ϊ����
 *      				                       MyArm.turn_steer_345_to_positon(90, 90, 90, 2000);
 * @endcode
 * @param[in]                  angle3             ���3�ĽǶȣ���Χ��0~360�� ע�⣺����������������Ա����е�λ��Ϊ��
 * @param[in]                  angle4             ���4�ĽǶȣ���Χ��0~360�� 
 * @param[in]                  angle5             ���5�ĽǶȣ���Χ��0~360�� 
 * @param[in]                  runtime           ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
 * @return                       boolean           ���������޶ȷ���false       
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
 * @brief   		                             ��Ҫ�����7��״̬�������غ���
 * @note                                         ǰ��Ļ�е�۵�λ�ú�����Ҫ������ȷ�������ĩ��λ�ã���������С���������ʹ���7�˶�����תĩ�˻���
 * @code                       ʾ��              ʹ���7���е��м�λ�ã��˶�ʱ��Ϊ����
 *      				                         MyArm.turn_steer7_to_positon(2047, 2000);
 * @endcode
 * @param[in]                  pos6              ���5��λ����Ϣ����Χ��1024~3071�� 
 * @param[in]                  runtime           ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
 * @return                     boolean           ���������޶ȷ���false       
 */
boolean Arm::turn_steer6_to_positon(word pos6, word runtime) {
	steer[6] ->Set_Steer_position_runtime(pos6 + offPos[6] , runtime);
}

 /**
 * @brief   		                             ��Ҫ�����7��״̬�������غ���
 * @note                                         ǰ��Ļ�е�۵�λ�ú�����Ҫ������ȷ�������ĩ��λ�ã���������С���������ʹ���7�˶�����תĩ�˻���
 * @code                       ʾ��              ʹ���7���е��м�λ�ã��˶�ʱ��Ϊ����
 *      				                         MyArm.turn_steer7_to_positon(2047, 2000);
 * @endcode
 * @param[in]                  angle6            ���5�ĽǶ���Ϣ����Χ��0~360�� 
 * @param[in]                  runtime           ��е���ɵ�ǰλ�����е�ָ��λ��������ʱ�䣬��λ�Ǻ���
 * @return                     boolean           ���������޶ȷ���false       
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
 * @brief   		                  �õ���е�۵�ƫ�ú���
 * @code                        ʾ��     
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
 * @brief   		                  ͨ����е�۵�ֱ��λ�ã����û�е��ƫ��    ͨ����е�۵�ֱ���������û�е��ƫ�ú���
 * @code                        ʾ��     ����IDΪ1�Ķ����ƫ��Ϊ100
 *      				                       MyArm.offset_by_pos(1�� 100);
 * @endcode
 * @param[in]                  id              �����ID��
 * @param[in]                  offset        ���ö����ƫ�ã�ֵ�ķ�Χ��-2046 ~ +2046��
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
 * @brief   		                  ͨ����е�۵ĽǶȣ����û�е��ƫ��
 * @code                        ʾ��     ����IDΪ1�Ķ����ƫ��Ϊ10��
 *      				                       MyArm.offset_by_pos(1�� 10);
 * @endcode
 * @param[in]                  id             �����ID��
 * @param[in]                  angle        ���ö����ƫ�ýǶȣ�ֵ�ķ�Χ��-90 ~ +90��
 * @return                       void        
 */
void Arm::offset_by_angle(byte id, double angle)
{
  short tmp = mapFloat(angle, -90, 90, -1024, 1024);
  offset_by_pos(id, tmp);
}

 /**
 * @brief   		                  ��ֵӳ�亯��    
 * @param[in]                 val                     ��Ҫӳ���ֵ
 * @param[in]                 in_min                ԭ�������Сֵ            
 * @param[in]                 in_max               ԭ��������ֵ
 * @param[in]                 out_min              ��Ҫӳ�䵽���������Сֵ
 * @param[in]                 out_max             ��Ҫӳ�䵽����������ֵ
 * @return                                               ����ֵ������ӳ����ֵ
 */
double mapFloat(double val, double in_min, double in_max, double out_min, double out_max)
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

 /**
 * @brief   		                  ����תλ�����ݣ�λ������Ϊ�����ֱ�ӿ������ݣ�    
 * @code                        ʾ��  
 *      		                          MyArm.Rad2Pos( 2.12);		                        
 * @endcode
 * @param[in]                 rad                    ����ֵ
 * @return                                               ����ֵ�������λ������
 */
short Arm::Rad2Pos(double rad)
{
    return short(mapFloat(rad, 0, PI, 1024, 3071));
}

 /**
 * @brief   		                  �Ƕ�תλ�����ݣ�λ������Ϊ�����ֱ�ӿ������ݣ�    
 * @code                        ʾ��  
 *      		                          MyArm.Angle2Pos( 90 );		                        
 * @endcode
 * @param[in]                 angle                 �Ƕ�ֵ
 * @return                                               ����ֵ�������λ������
 */
short Arm::Angle2Pos(double angle)
{
    return short(mapFloat(angle, 0, 180, 1024, 3071));
}

/**
 * @brief   		                  λ������ת�Ƕ�����   
 * @code                        ʾ��  
 *      		                          MyArm.Pos2Angle( 90 );		                        
 * @endcode
 * @param[in]                 pos                   λ������
 * @return                                               ����ֵ���Ƕ�����
 */
double Arm::Pos2Angle(double pos)
{
    return double(mapFloat(pos, 1024, 3071, 0, 180));
}

/**
 * @brief   		                  ����ֵת�Ƕ�ֵ����  
 * @code                        ʾ��  
 *      		                          MyArm.Rad2Angle( 2.12 );		                        
 * @endcode
 * @param[in]                 rad                    ��������
 * @return                                               ����ֵ���Ƕ�����
 */
double Arm::Rad2Angle(double rad)
{
    return double(mapFloat(rad, 0, PI, 0, 180));
}

/**
 * @brief   		                  �Ƕ�ֵת����ֵ   
 * @code                        ʾ��  
 *      		                          MyArm.Angle2Rad( 98 );		                        
 * @endcode
 * @param[in]                 angle                 �Ƕ�����
 * @return                                               ����ֵ����������
 */
double Arm::Angle2Rad(double angle)
{
    return double(mapFloat(angle, 0, 180, 0, PI));
}

/**
 * @brief   		                  ֱ��λ������ת������ֵ����
 * @code                        ʾ��  
 *      		                          MyArm.Rad2Angle( 2.12 );		                        
 * @endcode
 * @param[in]                 rad                    ��������
 * @return                                               ����ֵ���Ƕ�����
 */
double Arm::Pos2Rad(word pos)
{
   return double(mapFloat(pos, 1024, 3071, 0, PI));
}

//�����ٶ�ȥ���������

/**
 * @brief   		                  ��ȡ��е�����ж���ĽǶ���Ϣ
 * @code                        ʾ��  �� double angle[6];
 *      		                          MyArm.Arm_angle(angle);		                        
 * @endcode
 * @param[in]                 angle                 ����ĽǶ�����
 * @return                                               ����ֵ��void
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
 * @brief   		                  ��ȡ��е�����ж����ֱ��λ����Ϣ
 * @code                        ʾ��  �� word pos[6];
 *      		                          MyArm.Arm_pos(pos);		                        
 * @endcode
 * @param[in]                 pos                 �����ֱ��λ������
 * @return                                               ����ֵ��void
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
      if(must2angle > MIN2ANGLE && must2angle < MAX2ANGLE){  //�ж�������߼н�
          if(MyArm.theta[1] > -0.8 && MyArm.theta[1] < 2.79){ //�жϵ�һ��߼н�
            //����mainangle
            double mainangle = ( MyArm.theta[1] - asin( (176.22*sin(must2angle)) / (sqrt(67362.8 - 67157.442*cos(must2angle))) ) )/3.14*180;
            #ifdef DEBUG
            Serial.println();
            Serial.print("mainangle = ");
            Serial.println(mainangle);
            #endif
            if( mainangle < MAXMAINANGLE){   //�ж����Ƕȣ�����ĩ�˵㵽ԭ������������ļн�
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
 
 
 
 
 
 
 
