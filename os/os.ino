#include <DS1307.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include "configdefs.h"
#include "cmddefs.h"
#include "initdevices.h"

SFE_BMP180 pressure;
DS1307 rtc(A14, A15);
datetimevar *dtime;
Time t;
void setup() {
  // put your setup code here, to run once:
  init_rtc();
  init_bmp180();
  init_variables();

 
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (command) {
    case WRITE_CONFIG:
      WriteConfig();
      break;
    case READ_CONFIG:
      ReadConfig();
      break;
    default:
      t = rtc.getTime();
      break;
  }

}
