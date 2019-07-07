#include <Arm_learn.h>

Arm_learn MyArm_learn;                                                //初始化一个机械臂学习对象

bool flag_first_sheqi = true;


 /**
 * @brief   		                  机械臂构造函数
 * @note                         用来初始化机械臂的学习功能    
 * @code                        示例  
 *      				                      Arm_learn MyArm_learn;  
 * @endcode
 */
void Arm_learn::Arm_learn_()
{
    button0_time_cnt = 0;
    button1_time_cnt = 0; 
    rom_offset = 8* sizeof(short);
    EEPROM.get(rom_offset + 1*sizeof(word), record_point_num);
    start_learn_flag = false;
    record_flag = false;
    
    pinMode(BTN0_PIN, INPUT);
    pinMode(BTN1_PIN, INPUT);
    
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(BUZZER_PIN, LOW);                   
}

 /**
 * @brief   		                  按键检测函数
 * @code                        示例  
 *      				                      MyArm_learn.button_learn_detect();    
 * @endcode
 */
void Arm_learn::button_learn_detect(void)
{
  if(digitalRead(BTN0_PIN) == LOW)
  {
    button0_time_cnt = 0;
    for(int i = 0; (i < 3000) && (digitalRead(BTN0_PIN) == LOW) ; i++)
    {
      button0_time_cnt = i;
      delay(10);
    }
  }
  
  if(digitalRead(BTN1_PIN) == LOW)
  {
    button1_time_cnt = 0;
    for(int i = 0; (i < 3000) && (digitalRead(BTN1_PIN) == LOW) ; i++)
    {
      button1_time_cnt = i;
      delay(10);
    }
  }
  
  if(button0_time_cnt > 101)  { start_learn_flag = !start_learn_flag;  record_flag = false; button0_time_cnt=0;
                                                if(start_learn_flag == true){record_point_num = 0;MyArm.Set_Arm_Torque_Off();} tone(37, 3000, 2000);}
  if( button1_time_cnt > 18) { record_flag = true; button1_time_cnt=0;}
}

 /**
 * @brief   		                  机械臂学习程序
 * @code                        示例  
 *      				                      MyArm_learn.start_learn();    
 * @endcode
 */
void Arm_learn::start_learn(void)
{
  button_learn_detect();

  while(start_learn_flag == true)
  {
    //Serial.println("test222!!!");
    button_learn_detect();
    if( record_flag == true)
    {
		 record_point_num++;
		 record_one_point();
		 record_flag = false;
		 EEPROM.put(rom_offset + 1*sizeof(word), record_point_num);
		 tone(37, 3000, 500);
	//      Serial.println("test!!!");
		  delay(10);
	   if(flag_first_sheqi){
		 record_point_num--;
	   }
	   flag_first_sheqi = false;
    }
  }
}

 /**
 * @brief   		                  位点记录函数
 * @code                        示例  
 *      				                      MyArm_learn.start_learn();    
 * @endcode
 */
void Arm_learn::record_one_point(void)
{
  for(int i = 0 ; i < MyArm.Steer_Num; i ++)
  {
      word pos_tmpe = 4096 - get_steer_positon(i);
      Serial.print("get from steer     ");Serial.println(pos_tmpe);
      EEPROM.put(rom_offset + 2*sizeof(word) + record_point_num*MyArm.Steer_Num*sizeof(word) + i*sizeof(word), pos_tmpe);
  }
  Serial.println(record_point_num);
  for(int i = 0; i < MyArm.Steer_Num; i++)
  {
      word pos_tmpe = 0;
      EEPROM.get(rom_offset + 2*sizeof(word) + record_point_num*MyArm.Steer_Num*sizeof(word) + i*sizeof(word), pos_tmpe);
      Serial.print("get for eeprom    "); Serial.println(pos_tmpe);
  }
}

 /**
 * @brief   		                  机械臂学习姿态重现函数
 * @code                        示例  
 *      				                      MyArm_learn.reappear_learn();    
 * @endcode
 */
void Arm_learn::reappear_learn()
{
  button_learn_detect();
  if((start_learn_flag == false) && (record_flag == true))
  {  
  Serial.println("start reappear_learn");

    word tmpe_pos[MyArm.Steer_Num] = {0};

   // MyArm.Set_Arm_Torque_On();
    EEPROM.get(rom_offset + 1*sizeof(word), record_point_num);
    Serial.print("record_point_num:   ");Serial.println(record_point_num);
    for(int i = 1; i <= record_point_num; i++)
    {
      for(int j = 0; j < MyArm.Steer_Num; j++)
      {
          EEPROM.get(rom_offset + 2*sizeof(word) + i*MyArm.Steer_Num*sizeof(word) + j*sizeof(word), tmpe_pos[j]);
          Serial.println(tmpe_pos[j]);
      }
          MyArm.move_to_position(  tmpe_pos[0] ,  tmpe_pos[1],  tmpe_pos[2] ,  RUN_TIME);
          MyArm.turn_steer_345_to_positon(tmpe_pos[3] ,  tmpe_pos[4],  tmpe_pos[5] ,  RUN_TIME);
          if(MyArm.Steer_Num == 7){MyArm.steer[6] ->Set_Steer_position_runtime(tmpe_pos[6] , RUN_TIME);}
          delay(RUN_TIME);
    }
  }
}

 /**
 * @brief   		                  得到机械臂位姿（采用了三次均值滤波）
 * @code                        示例  得到一号舵机的位置数据
 *      				                      MyArm_learn.get_steer_positon(1);    
 * @endcode
 * @param[in]                 i            舵机的ID号
 */
word Arm_learn::get_steer_positon(byte i)
{
  word pos_tmp[3] = {0};
  for(int j = 0; j < 1; j++)
  {
     MyArm.steer[i]->Get_Steer_Position_Current_Inf();
     pos_tmp[j] = MyArm.steer[i] ->Position_Current[0];
     pos_tmp[j] = (pos_tmp[j] * 256) + MyArm.steer[i] ->Position_Current[1];
  }
  return (pos_tmp[0] + pos_tmp[0] + pos_tmp[0])/3;
}

//利用速度去做整体调整