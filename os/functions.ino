#include <stdlib.h>

datetimevar* get_time()
{
  datetimevar *regtime = new datetimevar;

  regtime->hora = rtc.getTimeStr();
  regtime->data = rtc.getDateStr(FORMAT_SHORT);
  regtime->dia = rtc.getDOWStr(FORMAT_SHORT);

  return regtime;

}

double getPressure()
{
  char status;
  double T, P, p0, a;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          Serial.print("Pressão Atmosferica: ");
          Serial.print(P);
          return (P);
          //pressão absoluta
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}

temp_press * gettemphum()
{
  // A leitura da temperatura e umidade pode levar 250ms!
  // O atraso do sensor pode chegar a 2 segundos.
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // testa se retorno é valido, caso contrário algo está errado.
  if (isnan(t) || isnan(h))
  {
    Serial.println("Failed to read from DHT");
  }
  else
  {
   /*
    Serial.print("Umidade: ");
    Serial.print(h);
    Serial.print(" %t");
    Serial.print("Temperatura: ");
    Serial.print(t);
    Serial.println(" *C");
    */
    char *temp = new char[10];               //temporarily holds data from vals
    char *hum = new char[10];               //temporarily holds data from vals
    dtostrf(t, 4, 6, temp);  //4 is mininum width, 4 is precision; float value is copied onto buff
    dtostrf(h, 4, 6, hum);  //4 is mininum width, 4 is precision; float value is copied onto buff
    temp_press *dhtresult = new temp_press;
    dhtresult->temperatura = temp;
    dhtresult->humidade = hum;
    return dhtresult;
  }
}
int getcommandfrombuffer()
{

  int cmd = commandbuffer[0]; //busca o primeiro da pilha de baixo para cima
                //rotina para movimentar a pilha uma posição para baixo
  int position = 0;
  while (position < 15)
  {
    commandbuffer[position] = commandbuffer[position + 1];
    if (position == 14)
      commandbuffer[position+1] = 0;
    position++;
  }
  return cmd;
}
 
int addcommandtobuffer(int command)
{
  int position = 0;
  while (position <= 15)
  {
    if (commandbuffer[position] == 0)
    {
      commandbuffer[position] = command;
      return command;
    }
    position++;
  }
  return 0;
}
void scheduler()
{
  unsigned long  tabs = millis();

 unsigned long cmdtime = elaptime + OS_READ_INTERVAL;
 unsigned long nexttime=tabs;
 
 Serial.print( " \ncmdtime ");
 Serial.print( cmdtime );
 Serial.print( "\ntabs ");
 Serial.print( tabs);
 Serial.print( "\n ");
 
 
 
  if (tabs > cmdtime)
  {
    Serial.print("\nScheduler ok ");
    addcommandtobuffer(READ_DHT);
    Serial.print("\nread_dht");
    addcommandtobuffer(READ_BMP180);
    Serial.print("\nread_BMP180");
    Serial.print("\n"); 
    elaptime++;   
    elaptime = nexttime;
    
     Serial.print(elaptime);
     while(true)
     {; }
  }
}


