#ifndef ARM_SPEED_H
#define ARM_SPEED_H

#include "TimerOne.h"
#include <Arm.h>


class Arm_speed{

  //�ܳ���ִ�к���
  //��������ִ�к���
  
  private:
  
	double speed;  //ÿ��ת����ֱ������
	word timerTime = 10;  //�趨��ʱʱ��Ϊ10ms
	word detx,dety,detz;
    
  public:
	void begin(HardwareSerial *desireSer);
	void allJugdeExecute();
  
};


#endif
