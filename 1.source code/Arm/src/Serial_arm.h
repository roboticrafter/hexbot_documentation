#ifndef  SERIAL_ARM_H
#define SERIAL_ARM_H

#include "Arduino.h"


/*************函数详细说明请参照CPP文件*************/

class Serial_arm {

  public:
  
    int parseint(HardwareSerial *desireSer);
    double parsedouble(HardwareSerial *desireSer);
};

#endif
