const char __SERIAL__ [ ] = "AJMA2C6D";

/* Arquivo  */



#include <Arduino.h>

#include <stdio.h>
#include <Time.h>
#include <TimeLib.h>
#include <DHT.h>
#include <TimeAlarms.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <MemoryFree.h>
#include "cmddefs.h"
#include "configdefs.h"
#include <IniFile.h>
#include "defs.h"
#include <Adafruit_GPS.h>
#define getName(var)  #var


/******************************************Constantes***********************************************/
#define DHTPIN A1 // pino que estamos conectado
#define DHTTYPE DHT11 // DHT 11
#define CSPIN 53
#define BATPIN A4
/* Macro define o local padrÃ£o do arquivo que manterÃ¡ os dados
   coletados dos sensores.                                     */
#define DATALOGGERFILEPATH "/DATA/DATALOG.LOG"
#define BATTERYLOGFILEPATH "/DATA/BATTERY.LOG"
#define CONFIGFILEPATH "/CONFIG/CONFIG.INI"
#define PROTOCOL_HEADER   '#'
#define S2_4_20 A2

// what's the name of the hardware serial port?
#define GPSSerial Serial3
bool GPSECHO = false;
uint32_t timer = millis();



// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
/* <title PLUV__PIN>

  PLUV_PIN Ã© o pino definido para a interrupÃ§Ã£o do pluviometro.



  Placas possuem pinos reservados para interrupÃ§Ãµes



  Uno, Nano, Mini, other 328-based 2, 3

  \---\> Mega, Mega2560, MegaADK 2, 3, 18, 19, 20, 21

  Micro, Leonardo, other 32u4-based 0, 1, 2, 3, 7

  Zero all digital pins, except 4

  MKR1000 Rev.1 0, 1, 4, 5, 6, 7, 8, 9, A1, A2

  Due todos os pinos digitais

  101 todos os pinos digitais                                   */
#define PLUV_PIN 3





/* <title pessure>

  Classe que faz a leitura do BMP180 * */
SFE_BMP180 pressure;
/* <title RTC>
   <toctitle Dispositivo relÃ³gio para ajuste de horario do CLP>

   Classe que farÃ¡ a leitura do real time clock                 */
//DS1307 rtc(RTC_SDAPIN, RTC_SCLPIN);
/* <title DHT>

   DeclaraÃ§Ã£o da classe de leitura do do DHT,esta classe fara a
   leitura da temperatura e umidade interna do equipamento.     */
DHT dht(DHTPIN, DHTTYPE);


/* <title ElapsedTime>

   variÃ¡vel que registra o tempo decorrido desde o inicio do
   sistema                                                   */
double ElapsedTime = 0;
/* <title pluvcounter>

   Variavel utilizada para contar as interrupÃ§Ãµes causadas pelo
   pluviometro.                                                 */
double pluvcounter = 0;
float interval;
/* <title TimerID>

   ID do timer de execuÃ§Ã£o imediata */
AlarmId TimerID0;
/* <title TimerID15>

   ID do timer de execuÃ§Ã£o de quarto de hora */
AlarmId TimerID15;
/* <title TimerID60>

   ID Timer do timer de execuÃ§Ã£o de hora cheia. */
AlarmId TimerID60;
/* <title TMID>

   ID da variÃ¡vel utilizada pelo TimerManager */
AlarmId TMID;

AlarmId TimeAdjust;

AlarmId GPSAdjust;


//Time  StartTime;

/* Neste CLP a Função Setup Ã© utilizada pra fazer as setagens
   dos parametros iniciais do sistema                         */
void setup() {




  /******************************************inicializa a Serial***********************************************/
  //! InicializaÃ§Ã£o da Serial
  /*! Para que a serial funcione ela deve ser iniciada na Função setup().
    \author Marcelo Silveira.
    \since 21/09/2016
    \version 1.0.0
  */
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  /******************************************inicializa a Serial***********************************************/
  Serial.print("Inicializando parametros de configuração....");
  configmodule();
  Serial.println("[Ok]");

  /******************************************inicializa a MicroSD***********************************************/
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.

  Serial.print("GPS data start..............................");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  Serial.println("[Ok]");
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  Serial.print("SDCARD Inicializando........................");
  pinMode(53, OUTPUT);
  if (!SD.exists("\\DATA"))
  {
    if (SD.mkdir("/DATA"))
    {
      if (!SD.exists(DATALOGGERFILEPATH))
      {
        File logfile = SD.open(DATALOGGERFILEPATH, FILE_WRITE); // only open a new file if it doesn't exist
        File batlogfile = SD.open(BATTERYLOGFILEPATH, FILE_WRITE);

        logfile.println("Local, Data, Hora, PBar, TempPBar, Pluv, Profund, UmdEquip, TempEquip");
        logfile.close();

        batlogfile.println("SN, Data, Hora, Volts, %");
        batlogfile.close();

        if (OS_DEBUG)
        {
          Serial.print("Pasta DATA criado com sucesso: ");
          Serial.print("Arquivo ");
          Serial.print(DATALOGGERFILEPATH);
          Serial.println(" criado com sucesso : ");
        }
      }
    }
    else {
      if (OS_DEBUG)
        Serial.print("Erro na criaçãoo da Pasta");
    }
  }
  else
  {
    if (!SD.exists(DATALOGGERFILEPATH))
    {
      File logfile = SD.open(DATALOGGERFILEPATH, FILE_WRITE); // only open a new file if it doesn't exist
      if (logfile)
      {
        logfile.println("Local, Lat, Lon Data, Hora, PBar, TempPBar, UmdEquip, TempEquip");
        logfile.close();
        if (OS_DEBUG)
        {
          Serial.print("Arquivo ");
          Serial.print(DATALOGGERFILEPATH);
          Serial.println(" criado com sucesso : ");
        }
      }
      else if (OS_DEBUG)
        Serial.println("Erro na abertura do arquivo : ");
    }
    if (!SD.exists(BATTERYLOGFILEPATH))
    {
      File batlogfile = SD.open(BATTERYLOGFILEPATH, FILE_WRITE); // only open a new file if it doesn't exist
      if (batlogfile)
      {
        batlogfile.println("SN, Data, Hora, Volts, %");
        batlogfile.close();
        if (OS_DEBUG)
        {
          Serial.print("Arquivo ");
          Serial.print(BATTERYLOGFILEPATH);
          Serial.println(" criado com sucesso : ");
        }
      }
      else if (OS_DEBUG)
        Serial.println("Erro na abertura do arquivo : ");
    }

  }

  if (OS_DEBUG)
    Serial.println("[Ok]");


  /******************************************fim da inicializaÃ§Ã£o a MicroSD***********************************************/

  /******************************************inicializa o XBee Serial***********************************************/
  //! InicializaÃ§Ã£o da Serial
  /*! Para que a serial funcione ela deve ser iniciada na Função setup().
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
  /*PLUV_PIN Ã© o pino 3  para o Ard

    Placas possuem pinos reservados para interrupÃ§Ãµes

    Uno, Nano, Mini, other 328-based  2, 3
    ---> Mega, Mega2560, MegaADK 2, 3, 18, 19, 20, 21
    Micro, Leonardo, other 32u4-based 0, 1, 2, 3, 7
    Zero  all digital pins, except 4
    MKR1000 Rev.1 0, 1, 4, 5, 6, 7, 8, 9, A1, A2
    Due todos os pinos digitais
    101 todos os pinos digitais
  */
  Serial.print("Attach interrupt start......................");
  pinMode(PLUV_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PLUV_PIN), CallPluvcouter, CHANGE);
  Serial.println("[Ok]");

  /******************************************Registra variaveis do pluviometro***********************************************/



  /******************************************inicializa o BMP180***********************************************/
  //! InicializaÃ§Ã£o do BMP180
  /*! Inicializa a classe que consulta o BMP180
    \author Marcelo Silveira.
    \since 25/09/2016
    \version 1.0.0
  */
  Serial.print("Incializando Barômetro......................");
  pressure.begin(); //inicia a classe utilizada pelo BMP180
  Serial.println("[Ok]");
  /******************************************inicializa o BMP180***********************************************/
  /******************************************inicializa o DHT11***********************************************/
  //! InicializaÃ§Ã£o do DHT11
  /*! Inicializa a classe que consulta o DHT11
    \author Marcelo Silveira.
    \since 24/09/2016
    \version 1.0.0
  */
  Serial.print("Incializando monitor de Temperatura.........");
  dht.begin(); //inicia a classe utilizada pelo DHT11
  Serial.println("[Ok]");
  /******************************************inicializa o DHT11***********************************************/

  /******************************************Demais Variaveis***********************************************/
  //! InicializaÃ§Ã£o de variaveis  do ambiente
  /*! Inicializa a ou resseta variaveis de inicializaÃ§Ã£o do sistema
    \author Marcelo Silveira.
    \since 25/09/2016
    \version 1.0.0
  */
  //! Memset da variavel commandbuffer.
  /*! Esta Função Ã© utilizada para  evitar o aparecimento de comandos invÃ¡lidos na inicializaÃ§Ã£o.
  */
  memset(commandbuffer, 0, sizeof(commandbuffer)); //zera todos os elementos da matriz de comandos
  memset(SerialCommands, 0, sizeof(SerialCommands));

  if (!SetCompilerDateTime(__DATE__, __TIME__))
    while (true)
    {
      ;
    }



  /******************************************Demais Variaveis***********************************************/

  //! Classe Alarm.
  /*! para a execuÃ§Ã£o de rotinas internas foram criados dois timers um Alarm e uma trigger, um para a execuÃ§Ã£o a cada 24 hs e outro para cada OS_READ_INTERVAL.
  */
  Serial.print("Inicializando Timers........................");
  TimerID0 = Alarm.timerRepeat(OS_READ_INTERVAL, scheduler);// // timer interno para execuÃ§Ã£o do Scheduler a cada  5 min
  Alarm.disable(TimerID0);
  TimerID15 = Alarm.timerRepeat(900, scheduler);// timer interno para execuÃ§Ã£o do Scheduler a cada  15 min
  Alarm.disable(TimerID15);
  TimerID60 = Alarm.timerRepeat(3600, scheduler);// timer interno para execuÃ§Ã£o do Scheduler a cada  a cada hora cheia
  Alarm.disable(TimerID60);
  Alarm.timerRepeat(60, ElapsedCounter);
  TMID = Alarm.timerRepeat(1, TimerManager);


  Serial.println("[Ok]");

  if (OS_DEBUG)
  {
    time_t t = now();
    char hora[13];
    char data[13];

    memset(data, 0, sizeof(data));
    sprintf(hora, "%02d:%02d:%02d", hour(t), minute(t), second(t));
    sprintf(data, "%02d/%02d/%02d", day(t), month(t), year(t));
    Serial.print("Data Padrão do Sistema: ");
    Serial.print(hora);
    Serial.print(" ");
    Serial.println(data);

    Serial.print("Timer ID: ");

  }




}

/* <title Loop>

   Loop()

   Laço principal do sistema  */
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
      break;
    case READ_OTT:
      ReadOTTRadar();
      break;
    case ADD_COMMAND:
      addcommandtobuffer(command);
      break;
    case WRITE_DATA:
      writedata();
      break;
    case LISTCONFIG:
      ListConfig();
      break;
    case LISTFILE:
      sendfiletotx();
      break;
    case SYNCTIME:
      SetInternalTime();
      break;
    case GPSTRACE:
      if (GPSECHO)
        GPSECHO = false;
      else
        GPSECHO = true;
      break;
    case BATLOAD:
      Batteryload();
      break;
    case READVCC:
      Serial.println(readVcc());
      break;
    default:
      if (Serial.available() > 0)
      {
        readBuffer();
      }
      else
      {
        if (strlen(SerialCommands) > 0)
        {
          parse_data(SerialCommands);
          memset(SerialCommands, 0, sizeof(SerialCommands));
        }
      }
      ReadGPS();
      break;
  }
  Alarm.delay(0);
}

/* <title TimerManager>

   <b>void</b> TimerManager()

   Função de ajuste do horÃ¡rio de consulta dos equipamentos */
void TimerManager()
{
  //Serial.print("Timer Manager");
  int min = minute();
  //Serial.println(min);
  if (OS_READ_INTERVAL_TYPE == 0)
  {
    if (OS_DEBUG)
      Serial.println(" OS_READ_INTERVAL_TYPE =0");
    Alarm.enable(TimerID0);
    Alarm.disable(TMID);
  }

  if (OS_READ_INTERVAL_TYPE == 1)
    if (min == 15 || min == 30 || min == 45)
    {
      if (OS_DEBUG)
        Serial.println(" OS_READ_INTERVAL_TYPE = 1");
      Alarm.enable(TimerID15);
      Alarm.disable(TMID);
    }
  if (OS_READ_INTERVAL_TYPE == 2)
    if (min == 0)
    {
      if (OS_DEBUG)
        Serial.println(" OS_READ_INTERVAL_TYPE = 2");
      Alarm.enable(TimerID60);
      Alarm.disable(TMID);
    }


}



//! Função utilizada para contar o tempo de utilizaÃ§Ã£o do sistema
/*! Esta Função apenas incrementa uma variavel minuto a minuto para futura conversÃ£o em horas
  \author Marcelo Silveira.
  \since 24/11/2016
  \version 1.0.0
*/
void ElapsedCounter()
{
  ElapsedTime++;//Adiciona 1 minuto ao tempo de operaÃ§Ã£o do sistema
}
/* <title CallPluvcouter>

   <b>void</b> CallPluvcouter()

  Função chamada pela interrupÃ§Ã£o de hardware para a contagem
   do pluviÃ´metro                                              */
void CallPluvcouter()
{
  if (millis() > interval)
  {
    PluvControl(PLUVINC);
    if (OS_DEBUG)
    {
      Serial.println("Evento Pluviometro incremento");
      Serial.println(millis());
    }

    interval = millis() + OS_PLUVIOMETER_READ_INTERVAL;
  }
}
/* <title PluvControl>

   <b>void</b> PluvControl(<b>unsigned</b> <b>int</b> CMD)

  Função utilizada para controlar o pluviometro quando chamada
   como o argumento CMD a <link PLUVRESET, Macro PLUVRESET>,
   zera os valores da variavel <link pluvcounter>, quan passado
   como argumento a <link PLUVINC, Macro PLUVINC> incrementa em
   1 a variÃ¡vel <link pluvcounter>.                             */
void PluvControl(unsigned int CMD)
{
  switch (CMD) {
    case PLUVRESET:
      pluvcounter = 0;
      break;
    case PLUVINC:
      pluvcounter++;
      if (OS_DEBUG)
        Serial.println(pluvcounter);
      break;
    default:
      break;
  }

}



/* <title getPressure>

   <b>void</b> getPressure()

  Função utilizada para obter os valores de temperatur e
   pressÃ£o atmosfï¿½rica, buscando a classe <link pressure, pessure>
   para buscar dados no sensor e traduzi-lo para hPa e ï¿½C



  Função criada em 30/09/2016                                     */
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
      // Inicia a mediÃ§Ã£o da pressÃ£o:
      // O parametro define a melhor amostragem, de 0 a 3 (alta resoluÃ§Ã£o, maior tempo de espera).
      // Em caso de erro serÃ¡ retornado 0.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          if (OS_DEBUG)
          {
            Serial.print("\nPressao Atmosferica: ");
            Serial.println(P);
            Serial.print("Temperatura: ");
            Serial.println(T);
          }
          
          
          PBar = String(P,2);
          
          Temp = String(T,2);

        }
        else if (OS_DEBUG)
          Serial.println("Erro na leitura da mediÃ§Ã£o de pressÃ£o");
      }
      else if (OS_DEBUG)
        Serial.println("erro ao iniciar a mediÃ§Ã£o de pressÃ£o");
    }
    else if (OS_DEBUG)Serial.println("erro ao iniciar a buscar a medida da temperatura");
  }
  else if (OS_DEBUG)
    Serial.println("erro ao iniciar a mediÃ§Ã£o de temperatura");
}

/* <title gettemphum>

   void gettemphum()

   AFunção gettemphum() busca pela classe <link dht@DHTPIN@DHTTYPE, DHT>
   para buscar dados no sensor de temperatura e umidade

   Criada em 30/09/2016                                                   */
void gettemphum()
{
  // A leitura da temperatura e umidade pode levar 250ms!
  // O atraso do sensor pode chegar a 2 segundos.
  double h = dht.readHumidity();
  double t = dht.readTemperature();
  // testa se retorno ÃƒÂ© valido, caso contrÃƒÂ¡rio algo estÃƒÂ¡ errado.
  if (isnan(t) || isnan(h))
  {
    Serial.println("Failed to read from DHT");
  }
  else
  {
    if (OS_DEBUG)
    {
      Serial.print("\nUmidade: ");
      Serial.print(h);
      Serial.println(" %t");
      Serial.print("Temperatura: ");
      Serial.print(t);
      Serial.println(" *C");
    }
    temperatura = String(t,2);
    umidade = String(h,2);
  }
}
//! Função getcommandfrombuffer()
/*! Função utiliza a variavel global commandbuffer para o empilhamento de comandos este sÃ£o retornados e uma cascata Ã© feita
  ! para colocar o proximo comando para execuÃ§Ã£o
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
int getcommandfrombuffer()
{
  const int cmd = commandbuffer[0]; //busca o primeiro da pilha de baixo para cima
  //rotina para movimentar a pilha uma posiÃ§Ã£o para baixo
  int position = 0;
  while (position < 14)
  {
    commandbuffer[position] = commandbuffer[position + 1];
    if (position == 13)
      commandbuffer[position + 1] = 0;
    position++;
  }
  return cmd;
}
/* <title addcommandtobuffer>

   <b>void</b> addcommandtobuffer(<b>const</b> <b>int</b>
   command);

  Função utilizada para adicionar comandos na variavel <link commandbuffer, CommandBuffer>

   EstaFunção coloca o comando na posiï¿½ï¿½o mais alta da pilha
   disponivel.

   Data de CriaÃ§Ã£o 30/09/2016                                                               */
void addcommandtobuffer(const int command) //Função para adiÃ§Ã£o de de comando na pilha "commandbuffer[16]"
{
  int position = 0;
  while (position < 14)
  {
    if (commandbuffer[position] == 0)
    {
      commandbuffer[position] = command;
      break;
    }
    position++;
  }
}

//! Função scheduler()
/*! Função utilizada para executar rotinas internas e outras se necessÃ¡rio for
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
void scheduler()
{
  SetInternalTime();
  LastExecutionMinute = minute();
  addcommandtobuffer(READ_DHT);
  addcommandtobuffer(READ_BMP180);
  addcommandtobuffer(READ_OTT);
  addcommandtobuffer(BATLOAD);
  addcommandtobuffer(SYNCTIME);
  addcommandtobuffer(WRITE_DATA);//Ultimo comando sempre

}

//! Função writedata()
/*! Função utilizada para executar rotinas internas e outras se necessÃ¡rio for
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/

void writedata() {
  char dataString[128];
  if (SD.exists(DATALOGGERFILEPATH))
  {

    File datalogfile;
    time_t t = now();
    char hora[16];
    char data[16];

    memset(data, 0, sizeof(data));
    memset(hora, 0, sizeof(data));
    sprintf(hora, "%02d:%02d:%02d", hour(t), minute(t), second(t));
    sprintf(data, "%02d-%02d-%4d", day(t), month(t), year(t));
    datalogfile = SD.open(DATALOGGERFILEPATH, FILE_WRITE);
    
    if (datalogfile)
    {


      // if the file is available, write to it:
      // print to the serial port too:

      //                    Local,lat, long, Data, Hora, PBar, TempPBar, Pluv,UmdEquip TempEquip

      String lat;
      String lon;
      memset(dataString, 0, sizeof(dataString));
      char str_tmp_pluv[15];
      char str_tmp_prof[15];
      dtostrf(pluvcounter * OS_PLUVIOMETER_VOLUME, 3, 2, str_tmp_pluv);
      dtostrf(Profundidade, 2, 2, str_tmp_prof);
      if (OS_DEBUG)
      {
        Serial.print("Tempo total de uso: ");
        Serial.print(ElapsedTime / 60, 2);
        Serial.println(" Horas de operacao");
        Serial.print("freeMemory()=");
        Serial.println(freeMemory());
        Serial.print("\nHora de execucao: ");
        Serial.println(hora);
        Serial.print("Tensao: ");
        Serial.print(readVcc());
        Serial.println("mV");
      }

      if (GPS.fix)
      {
        String dirlat = String(GPS.lat);
        String dirlon = String(GPS.lon);
        
        if (dirlat == "S")
          lat  = String((GPS.latitude / 100)*-1, 6);
        else
          lat  = String(GPS.latitude / 100, 6);

        if (dirlon == "W")
          lon =  String((GPS.longitude / 100)*-1, 6);
        else
          lon =  String(GPS.longitude / 100, 6);

      } else
      {
        lat  = String("0");
        lon =  String("0");

      }




      //                            Local, Lat, Long, Data, Hora, PBar, TempPBar, Pluv, Profund, UmdEquip TempEquip,Battery

      sprintf(dataString, "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s", OS_LOCATION, lat.c_str(), lon.c_str(), data, hora, PBar.c_str(), Temp.c_str(), str_tmp_pluv, str_tmp_prof, umidade.c_str(), temperatura.c_str(), BatteryVoltage.c_str());
      datalogfile.println(dataString); //adiciona a linha no arquivo
      if (OS_DEBUG)
        Serial.println(dataString);

      datalogfile.close();      //fecha o arquivo de dados
      PluvControl(PLUVRESET); //reseta a variavel do pluviometro
    }
    else if (OS_DEBUG)
      Serial.print("Erro ao abrir o arquivo: ");
  }
  // if the file isn't open, pop up an error:
  else {
    if (OS_DEBUG)
    {
      Serial.print("Arquivo não existe: ");
      Serial.println(DATALOGGERFILEPATH);
    }

  }
}

void configmodule() {

  File logfile;
  if (!SD.begin(CSPIN)) {
    Serial.println("Erro 0xFFFFFFF Falha na initialização!");
    while (1)
      ;
  }
  const size_t bufferLen = 80;
  char buffer[bufferLen];
  IniFile ini(CONFIGFILEPATH, FILE_READ, true);
  if (!ini.open()) {
    Serial.print(CONFIGFILEPATH);
    // Cannot do anything else
    while (1);
  }
  ini.getValue("sistema", "OS_DEBUG", buffer, bufferLen, OS_DEBUG);
  ini.getValue("sistema", "OS_READ_INTERVAL", buffer, bufferLen, OS_READ_INTERVAL);
  ini.getValue("sistema", "OS_LOCATION", OS_LOCATION, sizeof(OS_LOCATION));
  ini.getValue("sistema", "OS_READ_INTERVAL_TYPE", buffer, bufferLen, OS_READ_INTERVAL_TYPE);
  ini.getValue("devices", "OS_PLUVIOMETER_VOLUME", buffer, bufferLen, OS_PLUVIOMETER_VOLUME);
  ini.getValue("devices", "OS_PLUVIOMETER_READ_INTERVAL", buffer, bufferLen, OS_PLUVIOMETER_READ_INTERVAL);
  ini.getValue("devices", "OS_OTT_FACTOR", buffer, bufferLen, OS_OTT_FACTOR);
  ini.getValue("modem", ">OS_GPRS_ISP", buffer, bufferLen, OS_GPRS_ISP, sizeof(OS_GPRS_ISP));
  ini.getValue("modem", ">OS_GPRS_APN", buffer, bufferLen, OS_GPRS_APN, sizeof(OS_GPRS_APN));
  ini.getValue("modem", ">OS_GPRS_USERNAME", buffer, bufferLen, OS_GPRS_USERNAME, sizeof(OS_GPRS_USERNAME));
  ini.getValue("modem", ">OS_GPRS_PASSWORD", buffer, bufferLen, OS_GPRS_PASSWORD, sizeof(OS_GPRS_PASSWORD));
  ini.getValue("modem", ">OS_FTP_SERVER", buffer, bufferLen, OS_FTP_SERVER, sizeof(OS_FTP_SERVER));
  ini.getValue("modem", ">OS_FTP_USERNAME", buffer, bufferLen, OS_FTP_USERNAME, sizeof(OS_FTP_USERNAME));
  ini.getValue("modem", ">OS_FTP_PASSWORD", buffer, bufferLen, OS_FTP_PASSWORD, sizeof(OS_FTP_PASSWORD));
  ini.getValue("modem", ">OS_SMPT_SERVER", buffer, bufferLen, OS_SMPT_SERVER, sizeof(OS_SMPT_SERVER));
  ini.getValue("modem", ">OS_SMTP_USER", buffer, bufferLen, OS_SMTP_USER, sizeof(OS_SMTP_USER));
  ini.getValue("modem", ">OS_SMTP_PASSWORD", buffer, bufferLen, OS_SMTP_PASSWORD, sizeof(OS_SMTP_PASSWORD));
  ini.close();

}


/* <title ReadOTTRadar>

   <b>void</b> ReadOTTRadar()

  Função utilizada para fazer a leitura do modulo 4-20 do radar
   OTT.
*/
void ReadOTTRadar()
{

  float S1Value = analogRead(S2_4_20);

  if (S1Value >= 4)
    Profundidade = S1Value * 1023 * 0.30 / 117;
  else
    Profundidade = 0;
  if (OS_DEBUG)
  {
    Serial.print("Valor 4-20: ");
    Serial.println(S1Value);
    Serial.print("Calculado: ");
    Serial.println(Profundidade);
  }
}


/* <title readVcc>

   <b>long</b> readVcc()

  Função utilizada para consultar a voltagem de entrada do
   hardware, poderÃ¡ ser utilizada para alarmar em caso de queda
   permanente de energia                                        */
long readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}


/*ex.: #0x1|param1|param2|string|bool|double|int*/
void parse_data(char *data) {

  _params params[1];
  int lastpos;
  char localbuffer[128];
  char headercmd;
  headercmd = data[0];
  Serial.println(headercmd);
  if (headercmd == 35) {
    int counter = 0;
    for (int i = 0; counter >= 7; i++)
    {
      if (data[i] == '|') {
        counter++;
        if (counter == 1) {
          char *p;
          lastpos = i;
          strncpy(localbuffer, data + 1, i - 1);
          params->_command = strtoul(localbuffer, &p, 16);
          Serial.println(params->_command);
        }
        if (counter == 2) {
          strncpy(params->_param1, data + lastpos + 1, i - lastpos - 1);
          lastpos = i;
          Serial.println(params->_param1);
        }
        if (counter == 3) {
          strncpy(params->_param2, data + lastpos + 1, i - lastpos - 1);
          lastpos = i;
        }
        if (counter == 4) {
          strncpy(params->_strvalue, data + lastpos + 1, i - lastpos - 1);
          lastpos = i;
        }
        if (counter == 5) {
          strncpy(localbuffer, data + lastpos + 1, i - lastpos - 1);
          lastpos = i;
          if (atoi(localbuffer) == 1)
            params->_boolvalue = true;
          if (atoi(localbuffer) == 0)
            params->_boolvalue = false;
        }
        if (counter == 6) {
          strncpy(localbuffer, data + lastpos + 1, i - lastpos - 1);
          lastpos = i;
          params->_doublevalue = strtod(localbuffer, 0);
        }
        if (counter == 7) {
          strncpy(localbuffer, data + lastpos + 1, i - lastpos - 1);
          lastpos = i;
          params->_intvalue = atoi(localbuffer);
        }
      }
    }
    switch (params->_command) {
      case SET_CONFIG:
        if (strcmp(params->_param1, "OS_DEBUG"))
          OS_DEBUG = params->_boolvalue;
        else if (strcmp(params->_param1, "OS_READ_INTERVAL"))
          OS_READ_INTERVAL = params->_intvalue;
        else if (strcmp(params->_param1, "OS_LOCATION"))
          strcpy(OS_LOCATION, params->_strvalue);
        else if (strcmp(params->_param1, "OS_READ_INTERVAL_TYPE"))
          OS_READ_INTERVAL_TYPE = params->_intvalue;
        else if (strcmp(params->_param1, "OS_PLUVIOMETER_VOLUME"))
          OS_PLUVIOMETER_VOLUME = params->_doublevalue;
        else if (strcmp(params->_param1, "OS_PLUVIOMETER_READ_INTERVAL"))
          OS_PLUVIOMETER_READ_INTERVAL = params->_intvalue;
        else if (strcmp(params->_param1, "OS_OTT_FACTOR"))
          OS_OTT_FACTOR = params->_doublevalue;
        else if (strcmp(params->_param1, "OS_GPRS_ISP"))
          strcpy(OS_GPRS_ISP, params->_strvalue);
        else if (strcmp(params->_param1, "OS_GPRS_APN"))
          strcpy(OS_GPRS_APN, params->_strvalue);
        else if (strcmp(params->_param1, "OS_GPRS_USERNAME"))
          strcpy(OS_GPRS_USERNAME, params->_strvalue);
        else if (strcmp(params->_param1, "OS_GPRS_PASSWORD"))
          strcpy(OS_GPRS_PASSWORD, params->_strvalue);
        else if (strcmp(params->_param1, "OS_FTP_SERVER"))
          strcpy(OS_FTP_SERVER, params->_strvalue);
        else if (strcmp(params->_param1, "OS_FTP_SERVER"))
          strcpy(OS_FTP_SERVER, params->_strvalue);
        else if (strcmp(params->_param1, "OS_FTP_USERNAME"))
          strcpy(OS_FTP_USERNAME, params->_strvalue);
        else if (strcmp(params->_param1, "OS_FTP_PASSWORD"))
          strcpy(OS_FTP_PASSWORD, params->_strvalue);
        else if (strcmp(params->_param1, "OS_SMPT_SERVER"))
          strcpy(OS_SMPT_SERVER, params->_strvalue);
        else if (strcmp(params->_param1, "OS_SMTP_USER"))
          strcpy(OS_SMTP_USER, params->_strvalue);
        else if (strcmp(params->_param1, "OS_SMTP_PASSWORD"))
          strcpy(OS_SMTP_PASSWORD, params->_strvalue);
        break;
      default:
        break;
    }

  }
  else
  {
    int cmd = atoi(data);
    if (cmd != 0)
      addcommandtobuffer(cmd);
  }
  //free(params);
}

void sendfiletotx() {
  File dataFile = SD.open(DATALOGGERFILEPATH, FILE_READ);
  if (dataFile) {

    Serial.write("--Start--\n");
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    Serial.write("\n--end--");
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.print("error opening ");
    Serial.println(DATALOGGERFILEPATH);
  }

}


void printErrorMessage(uint8_t e, bool eol = true)
{
  switch (e) {
    case IniFile::errorNoError:
      Serial.print("no error");
      break;
    case IniFile::errorFileNotFound:
      Serial.print("file not found");
      break;
    case IniFile::errorFileNotOpen:
      Serial.print("file not open");
      break;
    case IniFile::errorBufferTooSmall:
      Serial.print("buffer too small");
      break;
    case IniFile::errorSeekError:
      Serial.print("seek error");
      break;
    case IniFile::errorSectionNotFound:
      Serial.print("section not found");
      break;
    case IniFile::errorKeyNotFound:
      Serial.print("key not found");
      break;
    case IniFile::errorEndOfFile:
      Serial.print("end of file");
      break;
    case IniFile::errorUnknownError:
      Serial.print("unknown error");
      break;
    default:
      Serial.print("unknown error value");
      break;
  }
  if (eol)
    Serial.println();
}

void ListConfig() {

  Serial.print("sistema: OS_DEBUG: ");
  Serial.println(OS_DEBUG);

  Serial.print("sistema: OS_READ_INTERVAL: ");
  Serial.println(OS_READ_INTERVAL);



  Serial.print("sistema: OS_LOCATION: ");
  Serial.println(OS_LOCATION);

  Serial.print("sistema: OS_READ_INTERVAL_TYPE: ");
  Serial.println(OS_READ_INTERVAL_TYPE);
  Serial.print("pluviometro: OS_PLUVIOMETER_VOLUME: ");
  Serial.println(OS_PLUVIOMETER_VOLUME);
  Serial.print("pluviometro: OS_PLUVIOMETER_READ_INTERVAL: ");
  Serial.println(OS_PLUVIOMETER_READ_INTERVAL);
  Serial.print("radar: OS_OTT_FACTOR: ");
  Serial.println(OS_OTT_FACTOR);
  Serial.print("modem: OS_GPRS_ISP: ");
  Serial.println(OS_GPRS_ISP);
  Serial.print("modem: OS_GPRS_APN: ");
  Serial.println(OS_GPRS_APN);
  Serial.print("modem: OS_GPRS_USERNAME: ");
  Serial.println(OS_GPRS_USERNAME);
  Serial.print("modem: OS_GPRS_PASSWORD: ");
  Serial.println(OS_GPRS_PASSWORD);
  Serial.print("modem: OS_FTP_SERVER: ");
  Serial.println(OS_FTP_SERVER);
  Serial.print("modem: OS_FTP_USERNAME: ");
  Serial.println(OS_FTP_USERNAME);
  Serial.print("modem: OS_FTP_PASSWORD: ");
  Serial.println(OS_FTP_PASSWORD);
  Serial.print("modem: OS_SMPT_SERVER: ");
  Serial.println(OS_SMPT_SERVER);
  Serial.print("modem: OS_SMTP_USER: ");
  Serial.println(OS_SMTP_USER);
  Serial.print("modem: OS_SMTP_PASSWORD: ");
  Serial.println(OS_SMTP_PASSWORD);
}
void SetInternalTime()
{
  if (GPS.fix) {
    tmElements_t tmSet;
    tmSet.Year = GPS.year;
    tmSet.Month = GPS.month;
    tmSet.Day = GPS.day;
    tmSet.Hour = GPS.hour - 3;
    tmSet.Minute = GPS.minute;
    tmSet.Second = GPS.seconds;
    const unsigned long DEFAULT_TIME = 1357041600;
    time_t t_of_day;
    t_of_day = makeTime(tmSet);
    if (t_of_day >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
      setTime(t_of_day); // Sync Arduino clock to the time received on the serial port
    }
  } else
    Serial.println("Sinal GPS Não Encontrado");

}
void writeconfig() {
  SD.remove(CONFIGFILEPATH);
  File ConfigFile = SD.open(CONFIGFILEPATH, FILE_WRITE);
  /*Cria a seção Sistema*/
  ConfigFile.println("[sistema]");
  ConfigFile.print("OS_DEBUG=");
  ConfigFile.println(OS_DEBUG);

  ConfigFile.print("OS_READ_INTERVAL=");
  ConfigFile.println(OS_READ_INTERVAL);

  ConfigFile.print("OS_LOCATION=");
  ConfigFile.println(OS_LOCATION);

  ConfigFile.print("OS_READ_INTERVAL=");
  ConfigFile.println(OS_READ_INTERVAL);



  /*Cria a seção Devices*/

  ConfigFile.println("[devices]");
  ConfigFile.print("OS_PLUVIOMETER_VOLUME=");
  ConfigFile.println(OS_PLUVIOMETER_VOLUME);

  ConfigFile.print("OS_PLUVIOMETER_READ_INTERVAL=");
  ConfigFile.println(OS_PLUVIOMETER_READ_INTERVAL);

  ConfigFile.print("OS_OTT_FACTOR=");
  ConfigFile.println(OS_OTT_FACTOR);

  /*Cria a seção Modem*/
  ConfigFile.println("[modem]");

  ConfigFile.print("OS_GPRS_ISP=");
  ConfigFile.println(OS_GPRS_ISP);
  ConfigFile.print("OS_GPRS_APN=");
  ConfigFile.println(OS_GPRS_APN);
  ConfigFile.print("OS_GPRS_USERNAME=");
  ConfigFile.println(OS_GPRS_USERNAME);
  ConfigFile.print("OS_GPRS_PASSWORD=");
  ConfigFile.println(OS_GPRS_PASSWORD);
  ConfigFile.print("OS_FTP_SERVER=");
  ConfigFile.println(OS_FTP_SERVER);

  ConfigFile.print("OS_FTP_USERNAME=");
  ConfigFile.println(OS_FTP_USERNAME);
  ConfigFile.print("OS_FTP_PASSWORD=");
  ConfigFile.println(OS_FTP_PASSWORD);
  ConfigFile.print("OS_SMPT_SERVER=");
  ConfigFile.println(OS_SMPT_SERVER);
  ConfigFile.print("OS_SMTP_USER=");
  ConfigFile.println(OS_SMTP_USER);
  ConfigFile.print("OS_SMTP_PASSWORD=");
  ConfigFile.println(OS_SMTP_PASSWORD);
  ConfigFile.close();



}


/*void SetParameter()
  {
   sscanf(sentence,"%s %*s %d",str,&i);
  }*/



void readBuffer()
{
  int n = strlen(SerialCommands);
  int c;
  if (n != 0)
    n + 1;
  char inByte;
  if (Serial.available() > 0)
  {
    inByte = Serial.read();
    // add to our read buffer
    SerialCommands[n] = inByte;
  }
}

void ReadGPS()
{
  // read data from the GPS in the 'main loop'
  //char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  //  if (GPSECHO)
  //   if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  GPS.read();
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    if (false)//GPSECHO)
      Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  if (GPSECHO) {
    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis()) timer = millis();

    // approximately every 2 seconds or so, print out the current stats
    if (millis() - timer > 2000) {
      timer = millis(); // reset the timer
      if (OS_DEBUG) {
        Serial.print("\nTime: ");
        Serial.print(GPS.hour, DEC); Serial.print(':');
        Serial.print(GPS.minute, DEC); Serial.print(':');
        Serial.print(GPS.seconds, DEC); Serial.print('.');
        Serial.println(GPS.milliseconds);
        Serial.print("Date: ");
        Serial.print(GPS.day, DEC); Serial.print('/');
        Serial.print(GPS.month, DEC); Serial.print("/20");
        Serial.println(GPS.year, DEC);
        Serial.print("Fix: "); Serial.print((int)GPS.fix);
        Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      }
      if (GPS.fix) {
        if (OS_DEBUG) {
          Serial.print("Location: ");
          Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
          Serial.print(", ");
          Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
          Serial.print("Speed (knots): "); Serial.println(GPS.speed);
          Serial.print("Angle: "); Serial.println(GPS.angle);
          Serial.print("Altitude: "); Serial.println(GPS.altitude);
          Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
        }
      }
    }
  }
}

bool SetCompilerDateTime(const char *datein, const char *timein )
{
  tmElements_t tm, tmSet;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 - paul, perhaps we define in time.h?
  const char *monthName[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  int Hour, Min, Sec;
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;
  if (sscanf(datein, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  if (sscanf(timein, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  Serial.print("Datecode: ");
  tmSet.Year = tm.Year;
  tmSet.Month = tm.Month;
  tmSet.Day = tm.Day;
  tmSet.Hour = tm.Hour;
  tmSet.Minute = tm.Minute;
  tmSet.Second = tm.Second;

  time_t t_of_day;
  t_of_day = makeTime(tmSet);
  if (t_of_day >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
    setTime(t_of_day); // Sync Arduino clock to the time received on the serial port
  }
  Serial.println(t_of_day);
  return true;
}

void Batteryload()
{
  double batVal, pinVoltage, ratio, D_BatteryVoltage;
  batVal = analogRead(BATPIN);    // read the voltage on the divider
  //Serial.println(batVal);
  pinVoltage = batVal * 0.00488;       //  Calculate the voltage on the A/D pin
  ratio = 3.2792;                                  //  A reading of 1 for the A/D = 0.0048mV
  //  if we multiply the A/D reading by 0.00488 then
  //  we get the voltage on the pin.
  //R1 = ((R2*Vin)/Vout)-R2

  //with a R2 of 5k ohms, I get the following values of R1 for 4 battery voltages:
  //nominal     max    R1  R2   Ratio
  //12           17    12  5    2.40
  //24           34    12  2    6.00
  //36           51    12  1.3  9.23
  //48           68    12  0.9 13.33
  //All values are in k ohms.
  //Solve for Vout to make sure Vout never exceeds 5v
  //Vout = (R2/(R1+R2))*Vin

  D_BatteryVoltage = pinVoltage * ratio;    //  Use the ratio calculated for the voltage divider
  //  to calculate the battery voltage
  BatteryVoltage= String(D_BatteryVoltage, 2);
  Serial.println(D_BatteryVoltage);
}

