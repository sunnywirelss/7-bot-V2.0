#ifndef  SERIAL_ARM_H
#define SERIAL_ARM_H

#include "Arduino.h"


/*************������ϸ˵�������CPP�ļ�*************/

class Serial_arm {

  public:
  
    int parseint(HardwareSerial *desireSer);
    double parsedouble(HardwareSerial *desireSer);
};

#endif
