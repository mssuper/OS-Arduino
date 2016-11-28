#include <Arduino.h>

#include <stdio.h>
#include <DS1307.h>
#include <Time.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <DHT.h>
#include <SFE_BMP180.h>
#include <SPI.h>
#include <SD.h>
#include <MemoryFree.h>
#include "cmddefs.h"
#include "configdefs.h"
#include "functions.h"


/******************************************Constantes***********************************************/
#define DHTPIN A1 // pino que estamos conectado
#define DHTTYPE DHT11 // DHT 11
#define CSPIN 53
#define RTC_SDAPIN A14
#define RTC_SCLPIN A15
#define DATALOGGERFILEPATH "/DATA/DATALOG.LOG"
#define S1_4_20 A4
#define S2_4_20 A5
#define PLUV_PIN 3
/******************************************Constantes***********************************************/
//! Declaração de variáveis de bibliotecas
/*! As Declarações que estão apresentadas abaixo são das bibliotecas adicionadas para cada periférico.
  \author Marcelo Silveira.
  \since 21/09/2016
  \version 1.0.0
*/
SFE_BMP180 pressure;
DS1307 rtc(RTC_SDAPIN, RTC_SCLPIN);
DHT dht(DHTPIN, DHTTYPE);

double ElapsedTime = 0;
double pluvcounter = 0;
float interval;
AlarmId TimerID0;
AlarmId TimerID15;
AlarmId TimerID60;
AlarmId TMID;

/*! Inicializa a Classe SoftwareSerial para comunicação com o XBee
  ! Inicialisa a classe XBee para operação do rádio
*/
void setup() {

  configmodule();



  /******************************************inicializa a Serial***********************************************/
  //! Inicialização da Serial
  /*! Para que a serial funcione ela deve ser iniciada na função setup().
    \author Marcelo Silveira.
    \since 21/09/2016
    \version 1.0.0
  */
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  /******************************************inicializa a Serial***********************************************/


  /******************************************inicializa a MicroSD***********************************************/
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.

  Serial.println("Initializing SD card...");
  File logfile;
  if (!SD.begin(CSPIN)) {
    Serial.println("initialization failed!");
    return;
  }
  else
  {
    if (!SD.exists("\\DATA"))
    {
      if ( SD.mkdir("/DATA") )
      {
        if (!SD.exists(DATALOGGERFILEPATH))
        {
          Serial.print("Diretorio DATA criado com sucesso: ");
          logfile = SD.open(DATALOGGERFILEPATH, FILE_WRITE); // only open a new file if it doesn't exist
          logfile.println("Local, Data, Hora, PBar, TempPBar, Pluv, Profund, UmdEquip, TempEquip");
          logfile.close();
          Serial.print("Arquivo ");
          Serial.print(DATALOGGERFILEPATH);
          Serial.println(" criado com sucesso : " );
        }
      }
      else {
        Serial.print("Erro na criação do directorio");
      }
    }
    else
    {
      if (!SD.exists(DATALOGGERFILEPATH))
      {
        File logfile = SD.open(DATALOGGERFILEPATH, FILE_WRITE); // only open a new file if it doesn't exist
        if (logfile)
        {
          logfile.println("Local, Data, Hora, PBar, TempPBar, UmdEquip, TempEquip");
          logfile.close();
          Serial.print("Arquivo ");
          Serial.print(DATALOGGERFILEPATH);
          Serial.println(" criado com sucesso : " );
        }
        else
          Serial.println("Erro na abertura do arquivo : " );

      }
    }
    Serial.println("SD Card inicializado com sucesso.");
  }

  /******************************************inicializa a MicroSD***********************************************/

  /******************************************inicializa o XBee Serial***********************************************/
  //! Inicialização da Serial
  /*! Para que a serial funcione ela deve ser iniciada na função setup().
    \author Marcelo Silveira.
    \since 21/09/2016
    \version 1.0.0
  */
  // start soft serial
  //  Serial2.begin(9600);
  //  xbee.setSerial(Serial2);
  // I think this is the only line actually left over
  // from Andrew's original example
  //  Serial.println("Xbee Iniciado!");


  /******************************************inicializa o XBee Serial***********************************************/



  /******************************************Registra variaveis do pluviometro***********************************************/
  /*PLUV_PIN é o pino 3  para o Ard

    Placas possuem pinos reservados para interrupções

    Uno, Nano, Mini, other 328-based  2, 3
    ---> Mega, Mega2560, MegaADK 2, 3, 18, 19, 20, 21
    Micro, Leonardo, other 32u4-based 0, 1, 2, 3, 7
    Zero  all digital pins, except 4
    MKR1000 Rev.1 0, 1, 4, 5, 6, 7, 8, 9, A1, A2
    Due todos os pinos digitais
    101 todos os pinos digitais
  */
  pinMode(PLUV_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PLUV_PIN), CallPluvcouter, CHANGE);


  /******************************************Registra variaveis do pluviometro***********************************************/




  /******************************************Aciona o relogio***********************************************/
  //! Inicialização do Real Time Clock
  /*! Nesta seção inicializa o RTC e sincroniza o relogio interno do arduino.
    \author Marcelo Silveira.
    \since 22/09/2016
    \version 1.0.0
  */
  Time  StartTime;
  rtc.halt(false);
  //Definicoes do pino SQW/Out
  rtc.setSQWRate(SQW_RATE_1);



  //rtc.setDOW(DOMINGO);        // Set Day-of-Week to SUNDAY
  //rtc.setTime(19, 56, 0);     // Set the time to 12:00:00 (24hr format)
  //rtc.setDate(23, 11, 2016);   // Set the date to October 3th, 2010

  rtc.enableSQW(true);
  StartTime = rtc.getTime(); //busca hora do RTC
  //Ajusta a hora do arduino
  setTime(StartTime.hour, StartTime.min, StartTime.sec, StartTime.mon, StartTime.date, StartTime.year); // set time to Saturday 8:29:00am Jan 1 2011
  /******************************************Aciona o relogio***********************************************/

  /******************************************inicializa o BMP180***********************************************/
  //! Inicialização do BMP180
  /*! Inicializa a classe que consulta o BMP180
    \author Marcelo Silveira.
    \since 25/09/2016
    \version 1.0.0
  */
  pressure.begin(); //inicia a classe utilizada pelo BMP180
  /******************************************inicializa o BMP180***********************************************/
  /******************************************inicializa o DHT11***********************************************/
  //! Inicialização do DHT11
  /*! Inicializa a classe que consulta o DHT11
    \author Marcelo Silveira.
    \since 24/09/2016
    \version 1.0.0
  */
  dht.begin(); //inicia a classe utilizada pelo DHT11
  /******************************************inicializa o DHT11***********************************************/

  /******************************************Demais Variaveis***********************************************/
  //! Inicialização de variaveis  do ambiente
  /*! Inicializa a ou resseta variaveis de inicialização do sistema
    \author Marcelo Silveira.
    \since 25/09/2016
    \version 1.0.0
  */
  //! Memset da variavel commandbuffer.
  /*! Esta função é utilizada para  evitar o aparecimento de comandos inválidos na inicialização.
  */
  memset(commandbuffer, 0, sizeof(commandbuffer)); //zera todos os elementos da matriz de comandos



  /******************************************Demais Variaveis***********************************************/

  //! Classe Alarm.
  /*! para a execução de rotinas internas foram criados dois timers um Alarm e uma trigger, um para a execução a cada 24 hs e outro para cada OS_READ_INTERVAL.
  */
  Alarm.alarmRepeat(8, 30, 0, MorningAlarm); //executa função MorningAlarm() todos os dias 8:30
  TimerID0 = Alarm.timerRepeat(OS_READ_INTERVAL, scheduler);// // timer interno para execução do Scheduler a cada  5 min
  Alarm.disable(TimerID0);
  TimerID15 = Alarm.timerRepeat(900, scheduler);// timer interno para execução do Scheduler a cada  15 min
  Alarm.disable(TimerID15);
  TimerID60 = Alarm.timerRepeat(3600, scheduler);// timer interno para execução do Scheduler a cada  a cada hora cheia
  Alarm.disable(TimerID60);
  Alarm.timerRepeat(60, ElapsedCounter);
  TMID = Alarm.timerRepeat(1, TimerManager);


  if (DEBUG_ON)
  {
    Serial.print("Setup iniciado em: ");
    Serial.println(rtc.getTimeStr());
    Serial.print("Timer iniciado ID: ");

  }
}

void loop() {
  // put your main code here, to run repeatedly:

  int command = 0;
  command = getcommandfrombuffer();
  switch (command) {
    case READ_DHT:
      gettemphum();
      break;
    case READ_BMP180:
      getPressure();
    case READ_OTT:
      ReadOTT();
      break;
    case ADD_COMMAND:
      addcommandtobuffer(command);
      break;
    case WRITE_DATA:
      writedata();
      break;
    default:
      timerminuto = minute();
      break;
  }
  Alarm.delay(0);
}

void TimerManager()
{
  //Serial.print("Timer Manager");
  int min = minute();
  //Serial.println(min);
  if (OS_READ_INTERVAL_TYPE == 0)
  {
    Serial.println("OS_READ_INTERVAL_TYPE =0");
    Alarm.enable(TimerID0);
    Alarm.disable(TMID);
  }

  if (OS_READ_INTERVAL_TYPE == 1)
    if (min == 15 || min == 30 || min == 45)
    {
      Serial.println("OS_READ_INTERVAL_TYPE = 1");
      Alarm.enable(TimerID15);
      Alarm.disable(TMID);
    }
  if (OS_READ_INTERVAL_TYPE == 2)
    if (min == 0)
    {
      Serial.println("OS_READ_INTERVAL_TYPE = 2");
      Alarm.enable(TimerID60);
      Alarm.disable(TMID);
    }


}

//! Função utilizada para executar rotinas a cada 24hs
/*! Esta função esta na chamada da classe Alarm
  \author Marcelo Silveira.
  \since 01/10/2016
  \version 1.0.0
*/
void MorningAlarm()
{

}
//! Função utilizada para contar o tempo de utilização do sistema
/*! Esta função apenas incrementa uma variavel minuto a minuto para futura conversão em horas
  \author Marcelo Silveira.
  \since 24/11/2016
  \version 1.0.0
*/
void ElapsedCounter()
{
  ElapsedTime++;//Adiciona 1 minuto ao tempo de operação do sistema
}
void CallPluvcouter()
{
  if (millis() > interval)
  {
    PluvControl(PLUVINC);
    Serial.println("Evento Pluviometro incremento");
    Serial.println(millis());

    interval = millis() + OS_PLUVIOMETER_READ_INTERVAL;
  }
}
void PluvControl(unsigned int CMD)
{
  switch (CMD) {
    case PLUVRESET:
      pluvcounter = 0;
      break;
    case PLUVINC:
      pluvcounter++;
      Serial.println(pluvcounter);

      break;
    default:
      break;
  }

}



//! Função utilizada para buscar a pressão e temperatura do BMP180
/*! Esta função se utiliza da classe SFE_BMP180 para buscar dados no sensor e traduzi-lo para hPa
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
void getPressure()
{
  char status;
  double T, P;
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Aguarda a leitura:
    delay(status);
    // Retorna a temperatura :
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Inicia a medição da presasão:
      // O parametro define a melhor amostragem, de 0 a 3 (alta resolução, maior tempo de espera).
      // Em caso de erro será retornado 0.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          if (DEBUG_ON)
          {
            Serial.print("\nPressao Atmosferica: ");
            Serial.println(P);
            Serial.print("Temperatura Atmosferica: ");
            Serial.println(T);
          }
          char str_temp[10];
          dtostrf(P, 4, 4, str_temp);
          sprintf(resultPBar->PBar, "%s", str_temp);
          dtostrf(T, 4, 4, str_temp);
          sprintf(resultPBar->Temp, "%s", str_temp);

        }
        else Serial.println("Erro na leitura da medição de pressão\n");
      }
      else Serial.println("erro ao iniciar a medição de pressão\n");
    }
    else Serial.println("erro ao iniciar a buscar a medida da temperatura\n");
  }
  else Serial.println("erro ao iniciar a medição de temperatura\n");
}

//! Função gettemphum()
/*! Esta função se utiliza da classe DHT para buscar dados no sensor e traduzí-lo para ÂºC e % de URA
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
void gettemphum()
{
  // A leitura da temperatura e umidade pode levar 250ms!
  // O atraso do sensor pode chegar a 2 segundos.
  double h = dht.readHumidity();
  double t = dht.readTemperature();
  // testa se retorno Ã© valido, caso contrÃ¡rio algo estÃ¡ errado.
  if (isnan(t) || isnan(h))
  {
    Serial.println("Failed to read from DHT");
  }
  else
  {
    if (DEBUG_ON)
    {
      Serial.print("\nUmidade: ");
      Serial.print(h);
      Serial.println(" %t");
      Serial.print("Temperatura: ");
      Serial.print(t);
      Serial.println(" *C");
    }
    char str_temp[10];
    dtostrf(t, 4, 4, str_temp);
    sprintf(dhtresult->temperatura, "%s", str_temp);
    dtostrf(h, 4, 4, str_temp);
    sprintf(dhtresult->humidade, "%s", str_temp);
  }
}
//! Função getcommandfrombuffer()
/*! Função utiliza a variavel global commandbuffer para o empilhamento de comandos este são retornados e uma cascata é feita
  ! para colocar o proximo comando para execução
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
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
  if (cmd != 0)
  {
    if (DEBUG_ON)
    {
      Serial.print("\nComando: ");
      Serial.print(cmd);
      Serial.println(" encaminhado para execucao.");
    }
  }
  return cmd;
}
//! Função addcommandtobuffer()
/*! Função utiliza a variavel global commandbuffer para o empilhamento de comandos, o comando é recebido e empilhado na
  !posição mais alta disponivel
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
void addcommandtobuffer(const int command) //Função para adição de de comando na pilha "commandbuffer[16]"
{
  int position = 0;
  while (position < 14)
  {
    if (commandbuffer[position] == 0)
    {
      commandbuffer[position] = command;
      if (DEBUG_ON)
      {
        Serial.print("\nComando: ");
        Serial.print(commandbuffer[position]);
        Serial.println(" adicionado.");
      }
      break;
    }
    position++;
  }
}

//! Função scheduler()
/*! Função utilizada para executar rotinas internas e outras se necessário for
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
void scheduler()
{
  if (DEBUG_ON)
    Serial.println("\nScheduler executado com sucesso ");
  LastExecutionMinute = minute();
  addcommandtobuffer(READ_TIME);
  addcommandtobuffer(READ_DHT);
  addcommandtobuffer(READ_BMP180);
  addcommandtobuffer(READ_OTT);
  addcommandtobuffer(WRITE_DATA);//Ultimo comando sempre
}

//! Função writedata()
/*! Função utilizada para executar rotinas internas e outras se necessário for
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/

void writedata() {

  if (SD.exists(DATALOGGERFILEPATH))
  {
    Time timeread = rtc.getTime();
    char hora[13];
    char data[13];

    memset(data, 0, sizeof(data));
    sprintf(hora, "%02d:%02d:%02d", timeread.hour, timeread.min, timeread.sec);
    sprintf(data, "%d-%0d-%d", timeread.date, timeread.mon,  timeread.year);
    File datalogfile = SD.open(DATALOGGERFILEPATH, FILE_WRITE);
    if (datalogfile)
    {
      Serial.println("Arquivo aberto");

      // if the file is available, write to it:
      // print to the serial port too:

      //                    Local, Data, Hora, PBar, TempPBar, Pluv,UmdEquip TempEquip
      Serial.print("Tempo total de uso: ");
      Serial.print(ElapsedTime / 60, 2);
      Serial.println(" Horas de operacao");
      Serial.print("freeMemory()=");
      Serial.println(freeMemory());
      Serial.print("\nHora de execucao: ");
      Serial.println(hora);
      char dataString[256];
      memset(dataString, 0, sizeof(dataString));
      char str_tmp_pluv[10];
      char str_tmp_prof[10];
      dtostrf(pluvcounter * OS_PLUVIOMETER_VOLUME, 3, 2, str_tmp_pluv);
      dtostrf(Profundidade, 2, 2, str_tmp_prof);



      //                            Local, Data, Hora, PBar, TempPBar, Pluv, Profund, UmdEquip TempEquip

      sprintf (dataString, "%s, %s, %s, %s, %s, %s, %s, %s, %s", OS_LOCATION, data, hora, resultPBar->PBar, resultPBar->Temp, str_tmp_pluv, str_tmp_prof, dhtresult->humidade, dhtresult->temperatura);
      Serial.println(dataString);
      datalogfile.println(dataString); //adiciona a linha no arquivo

      datalogfile.close();      //fecha o arquivo de dados
      PluvControl(PLUVRESET); //reseta a variavel do pluviometro
    }
    else
      Serial.print("Erro ao abrir o arquivo: ");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.print("Arquivo não existe: ");
    Serial.println(DATALOGGERFILEPATH);
  }
}

void configmodule() {
  OS_READ_INTERVAL = 30; //tempo padrão de leitura de todos o equipamentos.
  strcpy(OS_LOCATION, "Default Location");
  OS_READ_INTERVAL_TYPE  = 0;
  /*                                      0 Executa as leituras no momento que ativado
                                          1 Executa as leituras a cada quarto cheio 15 em 15 min
                                          2 Executa as leitura na hora cheia
  */
  OS_PLUVIOMETER_VOLUME = 0.2;
  OS_PLUVIOMETER_READ_INTERVAL = 1000;
  OS_OTT_FACTOR = 0.534;
  /*OS_GPRS_ISP;
    OS_GPRS_APN;
    OS_GPRS_USERNAME;
    OS_GPRS_PASSWORD;
    OS_FTP_SERVER;
    OS_FTP_USERNAME;
    OS_FTP_PASSWORD;
    OS_SMPT_SERVER;
    OS_SMTP_USER;
    OS_SMTP_PASSWORD;
  */
}

void ReadConfigModule()
{

}
void ReadOTT()
{

  int S2Value = analogRead(S2_4_20);
  Serial.print("Valor 4-20: ");
  Serial.println(S2Value);
  Serial.print("Calculado: ");
if (S2Value >= 4)
    Profundidade = (S2Value - 4) / OS_OTT_FACTOR;
else
  Profundidade = 0;
  Serial.println(Profundidade);
}
