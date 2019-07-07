#ifndef  ARM_PROTECT_H
#define ARM_PROTECT_H

#define PRINTF_DATA

#include "Arduino.h"
#include "Arm.h"
#include "Serial_arm.h"

#define BUZZER_PIN                37                                                           //蜂鸣器引脚

//注意：：所有角度的输入输出全都是以弧度制为准

const double STEER_0_MIN_LIMT   = PI/18;                                          //机械臂舵机0的弧度最小值
const double STEER_0_MAX_LIMT   = 17*PI/18;                                    //机械臂舵机0的弧度最大值，以下类似
const double STEER_1_MIN_LIMT   = PI/18;
const double STEER_1_MAX_LIMT   = PI;
const double STEER_2_MIN_LIMT   = PI/18;
const double STEER_2_MAX_LIMT   = 17*PI/18;

const double STEER_3_MIN_LIMT   = 0;                                          
const double STEER_3_MAX_LIMT   = PI;                                     
const double STEER_4_MIN_LIMT   = 0;                                          
const double STEER_4_MAX_LIMT   = PI;                                     
const double STEER_5_MIN_LIMT   = 0;                                          
const double STEER_5_MAX_LIMT   = 2*PI;                                     

/*************函数详细说明请参照CPP文件*************/

class Arm_protect{
  private:
  
  public:     
  boolean angle_limt_protect(double theta[], int num = 0);
  
  boolean position_line_calculate(double theta1, double theta2);
  boolean Positon_line2_limit(double theta1, double theta2);
  
  boolean Position_Protect( double *theta, int num = 0);
  
  /****前边的都是舵机012的位置保护，这个是舵机345的旋转角度的保护*****/
  boolean steer_345_angle_protect(double theta[], int num = 0);
  
  void warning();
};

extern Arm_protect MyArm_Protect;

#endif
