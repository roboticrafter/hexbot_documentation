#ifndef ARM_SPEED_H
#define ARM_SPEED_H

#include "TimerOne.h"
#include <Arm.h>


class Arm_speed{

  //总常量执行函数
  //各个常量执行函数
  
  private:
  
	double speed;  //每秒转过的直接数据
	word timerTime = 10;  //设定定时时间为10ms
	word detx,dety,detz;
    
  public:
	void begin(HardwareSerial *desireSer);
	void allJugdeExecute();
  
};


#endif
