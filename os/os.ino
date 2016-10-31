#include <DHT.h>


#include <DS1307.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include "cmddefs.h"
#include "configdefs.h"
#include "initdevices.h"


SFE_BMP180 pressure;
DS1307 rtc(A14, A15);
DHT dht(DHTPIN, DHTTYPE);
Time t;
void setup() {
  // put your setup code here, to run once:
  init_rtc();
  init_bmp180();
  init_variables();
  init_dht();
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
    case READ_DHT:
      dhtresult = gettemphum();
      break;      
    case READ_BMP180:
      pressao = getPressure();
      break;
    case ADD_COMMAND:
      addcommandtobuffer(command);
      break;
    default:
      command = getcommandfrombuffer();
      t = rtc.getTime();
      break;
  }

}
