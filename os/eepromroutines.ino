
#include <EEPROM.h>
void ReadConfig()
{
  int value;
  int address = 0;
  value = EEPROM.read(address);
  if (value!=0)
  {
    cfg->OS_READ_INTERVAL = value;
    Serial.print("Value: ");
    Serial.print(value);
    Serial.print("\n\r");    
    Serial.print("READ_INTERVAL: ");
    Serial.print(cfg->OS_READ_INTERVAL);    
  }
}

void WriteConfig()
{

  int address = 0;
  EEPROM.update(address, cfg->OS_READ_INTERVAL);
  address++;
  
  
  }
