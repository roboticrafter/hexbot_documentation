#include "Arm_protect.h"

Arm_protect MyArm_Protect;
 /**
 * @brief   		                   角度限制保护函数::适用范围舵机012
 * @note                         约束机械臂的角度在一定范围内，超出则报警，这个是约束0
 * @code                        示例     double theta[3] = {1 ,2 ,2};
 *      				                       MyArm_Protect.angle_limt_protect(theta, sizeof(theta)/sizeof(theta[0]));
 * @endcode
 * @param[in]                  theta   存储舵机012的弧度信息
 * @param[in]                  num    theta数组的大小
 * @return                       true：代表角度都在约束范围内   false: 代表角度超出限制        
 */
boolean Arm_protect::angle_limt_protect(double theta[], int num = 0)
{      
      if(num > 2)
      {
        if( (theta[0] >= STEER_0_MIN_LIMT)  && (theta[0] <= STEER_0_MAX_LIMT)  \
            && (theta[1] >= STEER_1_MIN_LIMT) && (theta[1] <= STEER_1_MAX_LIMT) \
            && (theta[2] >= STEER_2_MIN_LIMT) && (theta[2] <= STEER_2_MAX_LIMT) )
            {
              Serial.println("in limit");
              return true;
            }
          else
          {
              Serial.println("Eeror : out of position limit0");
              return false;
          }
      }
      else 
      {
        Serial.println("Eeror : there is not enough data");
        return false;
      }
}

 /**
 * @brief   		                   机械臂直线位置超限判别函数::适用范围舵机012
 * @note                         约束机械臂的位置符合一定的运动规律，theta1和theta2有一个动态的约束过程，这是约束1
 * @code                        示例     double theta[3] = {1 ,2 ,2};
 *      				                       MyArm_Protect.position_line_calculate(theta[1],  theta[2]);
 * @endcode
 * @param[in]                  theta1    舵机1的弧度值
 * @param[in]                  theta2    舵机2的弧度值
 * @return                       true：代表角度都在约束范围内   false: 代表角度超出限制        
 */
boolean Arm_protect::position_line_calculate(double theta1, double theta2)
{
   double jud = -58*tan(theta1) + 40*tan(theta1)*sin(theta2) - 40*cos(theta2) + 18.3;
   #ifdef PRINTF_DATA
   Serial.print("-58*");
   Serial.print(tan(theta1)); 
   Serial.print(" + 40*");
   Serial.print(tan(theta1)*sin(theta2)); 
   Serial.print(" -40*"); 
   Serial.print(cos(theta2));
   Serial.print(" + 18  = ");
   Serial.println(jud);
   
   Serial.print("theta1 =  "); Serial.print(theta1); Serial.print("   tan(theta1) = "); Serial.println(tan(theta1));
   Serial.print("theta2 =  "); Serial.print(theta2); Serial.print("   sin(theta2) = "); Serial.println(sin(theta2));
   Serial.print("  cos(theta2)= "); Serial.println(sin(cos(theta2)));
   #endif
   
   if(jud > 0)
   {
      return true;
   }
    else 
   {
        Serial.println("Eeror : out of position limit1");
        return false;
   }
}

 /**
 * @brief   		                   机械臂直线位置超限判别函数::适用范围舵机012
 * @note                         约束机械臂的位置符合一定的运动规律，theta1和theta2有一个动态的约束过程，这个是约束2
 * @code                        示例     double theta[3] = {1 ,2 ,2};
 *      				                       MyArm_Protect.position_line_calculate(theta[1],  theta[2]);
 * @endcode
 * @param[in]                  theta1    舵机1的弧度值
 * @param[in]                  theta2    舵机2的弧度值
 * @return                       true：代表角度都在约束范围内   false: 代表角度超出限制        
 */
  boolean Arm_protect::Positon_line2_limit(double theta1, double theta2)
  {
  #ifdef PRINTF_DATA
  Serial.print(theta1); Serial.print(" + "); Serial.print(theta2); Serial.print(" - "); Serial.print( 13*PI/9) ; Serial.print(" = "); Serial.println(theta1 + theta2 - 13*PI/9);
  #endif
    if(theta1 + theta2 < 13*PI/9)
    {
      return true;
    }
    else 
    {
      Serial.println("Eeror : out of position limit2");
      return false;
    }
  }

 /**
 * @brief   		                   机械臂位置保护函数::适用范围舵机012
 * @note                         是约束012的一个总约束
 * @code                        示例     double theta[3] = {1 ,2 ,2};
 *      				                       MyArm_Protect.Position_Protect(theta,  sizeof(theta)/sizeof(theta[0]));
 * @endcode
 * @param[in]                  theta   存储舵机012的弧度信息
 * @param[in]                  num    theta数组的大小
 * @return                       true：代表角度都在约束范围内   false: 代表角度超出限制        
 */
boolean Arm_protect::Position_Protect( double *theta, int num = 0)
{
        if(num > 2)
        {
          boolean judge1 = angle_limt_protect( theta, num);
          boolean judge2 = position_line_calculate(theta[1], theta[2]);
          boolean judge3 = Positon_line2_limit(theta[1], theta[2]);
          if( judge1 && judge2&&judge3){return true;}
          else{return false;}
        }
        else
        {
          return false;
        }
}

 /**
 * @brief   		                   角度限制保护函数::适用范围舵机345
 * @note                         约束机械臂的角度在一定范围内，超出则报警，这个是舵机345的约束0
 * @code                        示例     double theta[3] = {1 ,2 ,2};
 *      				                       MyArm_Protect.steer_345_angle_protect(theta, sizeof(theta)/sizeof(theta[0]));
 * @endcode
 * @param[in]                  theta   存储舵机012的弧度信息
 * @param[in]                  num    theta数组的大小
 * @return                       true：代表角度都在约束范围内   false: 代表角度超出限制        
 */
boolean Arm_protect::steer_345_angle_protect(double theta[], int num = 0)
{
      if(num > 2)
      {
        if( (theta[3] >= STEER_3_MIN_LIMT)  && (theta[3] <= STEER_3_MAX_LIMT)  \
            && (theta[4] >= STEER_4_MIN_LIMT) && (theta[4] <= STEER_4_MAX_LIMT) \
            && (theta[5] >= STEER_5_MIN_LIMT) && (theta[5] <= STEER_5_MAX_LIMT) )
            {
              Serial.println("angle of 345 in limit");
              return true;
            }
          else
          {
              Serial.println("Eeror : out of position limit3");
              return false;
          }
      }
      else 
      {
        Serial.println("Eeror : there is not enough data");
        return false;
      }

}

 /**
 * @brief   		                   机械臂位置保护超限报警函数::适用范围舵机012345
 * @code                        示例     
 *      				                       MyArm_Protect.warning();
 * @endcode
 * @param[in]                  void
 * @return                       void     
 */
void Arm_protect::warning()
{
  for(int i = 0; i < 3; i++)
  {
    tone(BUZZER_PIN, 3500, 200);
    delay(200);
    noTone(BUZZER_PIN);
    delay(300);
  }
}

