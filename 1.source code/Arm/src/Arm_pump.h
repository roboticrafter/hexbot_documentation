#ifndef  ARM_PUMP_H
#define ARM_PUMP_H

#include "Arduino.h"

#define PUMP_PIN  40

/*************������ϸ˵�������CPP�ļ�*************/

class Arm_pump {

  public:
    Arm_pump();
    boolean pump_on();
    boolean pump_off();
 };

#endif
