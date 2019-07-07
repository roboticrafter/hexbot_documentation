#ifndef  ARM_PUMP_H
#define ARM_PUMP_H

#include "Arduino.h"

#define PUMP_PIN  40

/*************函数详细说明请参照CPP文件*************/

class Arm_pump {

  public:
    Arm_pump();
    boolean pump_on();
    boolean pump_off();
 };

#endif
