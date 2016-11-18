#include <stdio.h>
#include <DS1307.h>
#include <Time.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <DHT.h>
#include <SFE_BMP180.h>
#include <SPI.h>
#include <SD.h>



#include "cmddefs.h"
#include "configdefs.h"

//! Defines de identificaÃ§Ã£o de pinos e auxiliares
/*! As DeclaraÃ§Ãµes que estÃ£o apresentadas abaixo identificam o equipamento e seu pino associado.
  \author Marcelo Silveira.
  \since 21/09/2016
  \version 1.0.0

*/
/******************************************Constantes***********************************************/
#define DHTPIN A1 // pino que estamos conectado
#define DHTTYPE DHT11 // DHT 11
#define CSPIN 53
#define RTC_SDAPIN A14
#define RTC_SCLPIN A15
#define DATALOGGERFILEPATH "/DATA/DATALOG.LOG"
#define FILEFORMAT "%s, %s, %s, %s, %s, %s, %s"
/******************************************Constantes***********************************************/
//! DeclaraÃ§Ã£o de variÃ¡veis de bibliotecas
/*! As DeclaraÃ§Ãµes que estÃ£o apresentadas abaixo sÃ£o das bibliotecas adicionadas para cada perifÃ©rico.
  \author Marcelo Silveira.
  \since 21/09/2016
  \version 1.0.0
*/
SFE_BMP180 pressure;
DS1307 rtc(RTC_SDAPIN, RTC_SCLPIN);
DHT dht(DHTPIN, DHTTYPE);
AlarmId Aid;
File logfile;



/*! Inicialisa a Classe SoftwareSerial para comunicaÃ§Ã£o com o XBee
  /*! Inicialisa a classe XBee para operaÃ§Ã£o do rÃ¡dio
*/
void setup() {

  /******************************************inicializa a Serial***********************************************/
  //! InicializaÃ§Ã£o da Serial
  /*! Para que a serial funcione ela deve ser iniciada na funÃ§Ã£o setup().
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

  if (!SD.begin(CSPIN)) {
    Serial.println("initialization failed!");
    return;
  }
  else
  {
    if (!SD.exists("/DATA"))
    {
      if ( SD.mkdir("/DATA") )
      {
        if (!SD.exists(DATALOGGERFILEPATH))
        {
          Serial.print("Diretorio DATA criado com sucesso: ");
          logfile = SD.open(DATALOGGERFILEPATH, FILE_WRITE); // only open a new file if it doesn't exist
          logfile.println("Local, Data, Hora, PBar, TempPBar, UmdEquip, TempEquip");
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
  //! InicializaÃ§Ã£o da Serial
  /*! Para que a serial funcione ela deve ser iniciada na funÃ§Ã£o setup().
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








  /******************************************Aciona o relogio***********************************************/
  //! InicializaÃ§Ã£o do Real Time Clock
  /*! Nesta seÃ§Ã£o inicializa o RTC e sincroniza o relogio interno do arduino.
    \author Marcelo Silveira.
    \since 22/09/2016
    \version 1.0.0
  */
  rtc.halt(false);
  //Definicoes do pino SQW/Out
  rtc.setSQWRate(SQW_RATE_1);

  
  
  //rtc.setDOW(SUNDAY);        // Set Day-of-Week to SUNDAY
  //rtc.setTime(13, 56, 0);     // Set the time to 12:00:00 (24hr format)
  //rtc.setDate(18, 11, 2016);   // Set the date to October 3th, 2010

  rtc.enableSQW(true);
  Time timeadjust = rtc.getTime(); //busca hora do RTC
  //Ajusta a hora do arduino
  setTime(timeadjust.hour, timeadjust.min, timeadjust.sec, timeadjust.mon, timeadjust.date, timeadjust.year); // set time to Saturday 8:29:00am Jan 1 2011
  /******************************************Aciona o relogio***********************************************/

  /******************************************inicializa o BMP180***********************************************/
  //! InicializaÃ§Ã£o do BMP180
  /*! Inicializa a classe que consulta o BMP180
    \author Marcelo Silveira.
    \since 25/09/2016
    \version 1.0.0
  */
  pressure.begin(); //inicia a classe utilizada pelo BMP180
  /******************************************inicializa o BMP180***********************************************/
  /******************************************inicializa o DHT11***********************************************/
  //! InicializaÃ§Ã£o do DHT11
  /*! Inicializa a classe que consulta o DHT11
    \author Marcelo Silveira.
    \since 24/09/2016
    \version 1.0.0
  */
  dht.begin(); //inicia a classe utilizada pelo DHT11
  /******************************************inicializa o DHT11***********************************************/

  /******************************************Demais Variaveis***********************************************/
  //! InicializaÃ§Ã£o de variaveis  do ambiente
  /*! Inicializa a ou resseta variaveis de inicializaÃ§Ã£o do sistema
    \author Marcelo Silveira.
    \since 25/09/2016
    \version 1.0.0
  */
  //! Memset da variavel commandbuffer.
  /*! Esta funÃ§Ã£o Ã© utilizada para  evitar o aparecimento de comandos invÃ¡lidos na inicializaÃ§Ã£o.
  */
  memset(commandbuffer, 0, sizeof(commandbuffer)); //zera todos os elementos da matriz de comandos
  datetimevar *dtime = get_time();
  temp_press *dhtresult;

  /******************************************Demais Variaveis***********************************************/

  //! Classe Alarm.
  /*! para a execuÃ§Ã£o de rotinas internas foram criados dois timers um Alarm e uma trigger, um para a execuÃ§Ã£o a cada 24 hs e outro para cada OS_READ_INTERVAL.
  */
  Alarm.alarmRepeat(8, 30, 0, MorningAlarm); //executa funÃ§Ã£o MorningAlarm() todos os dias 8:30
  Aid = Alarm.timerRepeat(OS_READ_INTERVAL, scheduler);// timer interno para execuÃ§Ã£o do Scheduler
  if (DEBUG_ON)
  {
    Serial.print("Setup iniciado em: ");
    Serial.println(rtc.getTimeStr());
    Serial.print("Timer iniciado ID: ");
    Serial.println(Aid);
  }
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
    case READ_TIME:
      dtime = get_time();
      break;
    case READ_DHT:
      dhtresult = gettemphum();
      break;
    case READ_BMP180:
      resultPBar = getPressure();
      break;
    case ADD_COMMAND:
      addcommandtobuffer(command);
      break;
    case WRITE_DATA:
      writedata();
      break;
    default:
      break;
  }
  Alarm.delay(0);
}
//! Função utilizada para executar rotinas a cada 24hs
/*! Esta funÃ§Ã£o esta na chamada da classe Alarm
  \author Marcelo Silveira.
  \since 01/10/2016
  \version 1.0.0
*/
void MorningAlarm()
{

}

//! FunÃ§Ã£o utilizada para buscar o horÃ¡rio do RTC
/*! Esta funÃ§Ã£o serÃ¡ utilizada para a recuperaÃ§Ã£o de horÃ¡rio jÃ¡ que o RTC nÃ£o altera seu horÃ¡rio independente de estar ligado ou nÃ£o
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/

datetimevar* get_time()
{
  datetimevar *regtime = new datetimevar;
  regtime->hora = rtc.getTimeStr();
  regtime->data = rtc.getDateStr(FORMAT_SHORT);
  regtime->dia = rtc.getDOWStr(FORMAT_SHORT);
  return regtime;
}
//! FunÃ§Ã£o utilizada para buscar a pressÃ£o e temperatura do BMP180
/*! Esta funÃ§Ã£o se utiliza da classe SFE_BMP180 para buscar dados no sensor e traduzÃ­-lo para hPa
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
TempPBar* getPressure()
{
  char status;
  double T, P, p0, a;


  status = pressure.startTemperature();
  if (status != 0)
  {
    // Aguarda a leitura:
    delay(status);
    // Retorna a temperatura :
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Inicia a mediÃ§Ã£o da presasÃ£o:
      // O parametro define a melhor amostragem, de 0 a 3 (alta resoluÃ§Ã£o, maior tempo de espera).
      // Em caso de erro serÃ¡ retornado 0.

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
          char *temp = new char[15];               //temporarily holds data from vals
          char *pressb = new char[15];               //temporarily holds data from vals
          dtostrf(T, 5, 6, temp);  //4 is mininum width, 4 is precision; float value is copied onto buff
          dtostrf(P, 5, 6, pressb);  //4 is mininum width, 4 is precision; float value is copied onto buff
          TempPBar *result = new TempPBar;
          result->PBar = pressb;
          result->Temp = temp;
          return result;
        }
        else Serial.println("Erro na leitura da mediÃ§Ã£o de pressÃ£o\n");
      }
      else Serial.println("erro ao iniciar a mediÃ§Ã£o de pressÃ£o\n");
    }
    else Serial.println("erro ao iniciar a buscar a medida da temperatura\n");
  }
  else Serial.println("erro ao iniciar a mediÃ§Ã£o de temperatura\n");
}

//! FunÃ§Ã£o gettemphum()
/*! Esta funÃ§Ã£o se utiliza da classe DHT para buscar dados no sensor e traduzÃ­-lo para ÂºC e % de URA
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
temp_press * gettemphum()
{
  // A leitura da temperatura e umidade pode levar 250ms!
  // O atraso do sensor pode chegar a 2 segundos.
  float h = dht.readHumidity();
  float t = dht.readTemperature();

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
//! FunÃ§Ã£o getcommandfrombuffer()
/*! FunÃ§Ã£o utiliza a variavel global commandbuffer para o empilhamento de comandos este sÃ£o retornados e uma cascata Ã© feita
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
//! FunÃ§Ã£o addcommandtobuffer()
/*! FunÃ§Ã£o utiliza a variavel global commandbuffer para o empilhamento de comandos, o comando Ã© recebido e empilhado na
  !posiÃ§Ã£o mais alta disponivel
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
void addcommandtobuffer(const int command) //FunÃ§Ã£o para adiÃ§Ã£o de de comando na pilha "commandbuffer[16]"
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

//! FunÃ§Ã£o scheduler()
/*! FunÃ§Ã£o utilizada para executar rotinas internas e outras se necessÃ¡rio for
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
void scheduler()
{
  if (DEBUG_ON)
  {
    Serial.println("\nScheduler executado com sucesso ");
    Serial.print("\nHorario de execucao: ");
    Serial.println(rtc.getTimeStr());
  }
  addcommandtobuffer(READ_TIME);
  addcommandtobuffer(READ_DHT);
  addcommandtobuffer(READ_BMP180);
  addcommandtobuffer(WRITE_DATA);

}

//! FunÃ§Ã£o writedata()
/*! FunÃ§Ã£o utilizada para executar rotinas internas e outras se necessÃ¡rio for
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/

void writedata() {

  if (SD.exists(DATALOGGERFILEPATH))
  {
    logfile = SD.open(DATALOGGERFILEPATH,FILE_WRITE);
    if (logfile)
    {
      Serial.println("Arquivo aberto");

      // if the file is available, write to it:
      // print to the serial port too:

      //                    Local, Data, Hora, PBar, TempPBar, UmdEquip TempEquip
      Serial.println(OS_LOCATION);
      Serial.println(dtime->data);
      Serial.println(dtime->hora);
      Serial.println(resultPBar->PBar);
      Serial.println(resultPBar->Temp);
      Serial.println(dhtresult->temperatura);
      Serial.println(dhtresult->humidade);
      char dataString[256];



      sprintf (dataString, FILEFORMAT, OS_LOCATION, dtime->data, dtime->hora, resultPBar->PBar, resultPBar->Temp, dhtresult->temperatura, dhtresult->humidade);
      Serial.println(dataString);
      logfile.println(dataString);

      logfile.close();
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

