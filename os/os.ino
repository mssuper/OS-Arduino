#include <Time.h>
#include <TimeLib.h>

#include <TimeLib.h>
#include <TimeAlarms.h>
#include <DHT.h>
#include <DS1307.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include "cmddefs.h"
#include "configdefs.h"




/******************************************Constantes***********************************************/
#define DHTPIN A1 // pino que estamos conectado
#define DHTTYPE DHT11 // DHT 11

/******************************************Constantes***********************************************/

SFE_BMP180 pressure;
DS1307 rtc(A14, A15);
DHT dht(DHTPIN, DHTTYPE);
AlarmId Aid;



void setup() {
  
  /******************************************inicializa a Serial***********************************************/
  Serial.begin(9600);
  /******************************************inicializa a Serial***********************************************/
  
  /******************************************Aciona o relogio***********************************************/
  rtc.halt(false);
  //Definicoes do pino SQW/Out
  rtc.setSQWRate(SQW_RATE_1);
  rtc.enableSQW(true);
  
  Time timeadjust = rtc.getTime();
  setTime(timeadjust.hour,timeadjust.min,timeadjust.sec,timeadjust.mon,timeadjust.date,timeadjust.year); // set time to Saturday 8:29:00am Jan 1 2011

  /******************************************Aciona o relogio***********************************************/

  
  /******************************************inicializa o BMP180***********************************************/
  pressure.begin();
  /******************************************inicializa o BMP180***********************************************/
  /******************************************inicializa o DHT11***********************************************/
  dht.begin();
  /******************************************inicializa o DHT11***********************************************/

  /******************************************Demais Variaveis***********************************************/
  memset(commandbuffer,0,sizeof(commandbuffer));
  datetimevar *dtime = get_time();
  temp_press *dhtresult;
  int pressao = 0;
  /******************************************Demais Variaveis***********************************************/
  Alarm.alarmRepeat(8,30,0, MorningAlarm);
  Aid = Alarm.timerRepeat(OS_READ_INTERVAL, scheduler);
  Serial.print("Timer iniciado ID: "); 
  Serial.println(Aid); 
  Serial.print("Setup iniciado em: ");
  Serial.println(rtc.getTimeStr());
}

void loop() {
  // put your main code here, to run repeatedly:
  int command = 0;
   command = getcommandfrombuffer();  
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
      break;
  }
    Alarm.delay(0);
}
void MorningAlarm()
{

  }

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
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          Serial.print("Pressao Atmosferica: ");
          Serial.println(P);
          return (P);  //pressão absoluta
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
    
      Serial.print("Umidade: ");
      Serial.print(h);
      Serial.println(" %t");
      Serial.print("Temperatura: ");
      Serial.print(t);
      Serial.println(" *C");

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

  const int cmd = commandbuffer[0]; //busca o primeiro da pilha de baixo para cima
  //rotina para movimentar a pilha uma posição para baixo
  int position = 0;
  while (position < 14)
  {
    commandbuffer[position] = commandbuffer[position + 1];
    if (position == 13)
      commandbuffer[position + 1] = 0;
    position++;
  }
  if (cmd!=0)
  {
    Serial.print("Comando: "); 
    Serial.print(cmd);
    Serial.println(" encaminhado para execucao.");
  }
  return cmd;
}

void addcommandtobuffer(const int command)
{
  int position = 0;
  while (position < 14)
  {
    if (commandbuffer[position] == 0)
    {
      commandbuffer[position] = command;
      Serial.print("Comando: "); 
      Serial.print(commandbuffer[position]);
      Serial.println(" adicionado.");
      break;
    }
    position++;
  }
  
}

void scheduler()
{
    Serial.println("Scheduler executado com sucesso ");
    addcommandtobuffer(READ_DHT);
    addcommandtobuffer(READ_BMP180);
}
