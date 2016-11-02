#include <Time.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <DHT.h>
#include <DS1307.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include "cmddefs.h"
#include "configdefs.h"

//! Defines de identificação de pinos e auxiliares
/*! As Declarações que estão apresentadas abaixo identificam o equipamento e seu pino associado.
  \author Marcelo Silveira.
  \since 21/09/2016
  \version 1.0.0

*/
/******************************************Constantes***********************************************/
#define DHTPIN A1 // pino que estamos conectado
#define DHTTYPE DHT11 // DHT 11

#define RTC_SDAPIN A14
#define RTC_SCLPIN A15


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
AlarmId Aid;

void setup() {

  /******************************************inicializa a Serial***********************************************/
  //! Inicialização da Serial
  /*! Para que a serial funcione ela deve ser iniciada na função setup().
    \author Marcelo Silveira.
    \since 21/09/2016
    \version 1.0.0
  */
  Serial.begin(9600);
  /******************************************inicializa a Serial***********************************************/

  /******************************************Aciona o relogio***********************************************/
  //! Inicialização do Real Time Clock
  /*! Nesta seção inicializa o RTC e sincroniza o relogio interno do arduino.
    \author Marcelo Silveira.
    \since 22/09/2016
    \version 1.0.0
  */
  rtc.halt(false);
  //Definicoes do pino SQW/Out
  rtc.setSQWRate(SQW_RATE_1);
  rtc.enableSQW(true);
  Time timeadjust = rtc.getTime(); //busca hora do RTC
  //Ajusta a hora do arduino
  setTime(timeadjust.hour, timeadjust.min, timeadjust.sec, timeadjust.mon, timeadjust.date, timeadjust.year); // set time to Saturday 8:29:00am Jan 1 2011
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
  datetimevar *dtime = get_time();
  temp_press *dhtresult;
  int pressao = 0;
  /******************************************Demais Variaveis***********************************************/

  //! Classe Alarm.
  /*! para a execução de rotinas internas foram criados dois timers um Alarm e uma trigger, um para a execução a cada 24 hs e outro para cada OS_READ_INTERVAL.
  */
  Alarm.alarmRepeat(8, 30, 0, MorningAlarm); //executa função MorningAlarm() todos os dias 8:30
  Aid = Alarm.timerRepeat(OS_READ_INTERVAL, scheduler);// timer interno para execução do Scheduler
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
//! Função utilizada para executar rotinas a cada 24hs
/*! Esta função esta na chamada da classe Alarm
  \author Marcelo Silveira.
  \since 01/10/2016
  \version 1.0.0
*/
void MorningAlarm()
{

}

//! Função utilizada para buscar o horário do RTC
/*! Esta função será utilizada para a recuperação de horário já que o RTC não altera seu horário independente de estar ligado ou não
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
//! Função utilizada para buscar a pressão e temperatura do BMP180
/*! Esta função se utiliza da classe SFE_BMP180 para buscar dados no sensor e traduzí-lo para hPa
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
double getPressure()
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
          return (P);  //pressão absoluta
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
/*! Esta função se utiliza da classe DHT para buscar dados no sensor e traduzí-lo para ºC e % de URA
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

  // testa se retorno é valido, caso contrário algo está errado.
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
//! Função getcommandfrombuffer()
/*! Função utiliza a variavel global commandbuffer para o empilhamento de comandos este são retornados e uma cascata é feita 
 *! para colocar o proximo comando para execução
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
 *!posição mais alta disponivel
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
  {
    Serial.println("\nScheduler executado com sucesso ");
    Serial.print("\nHorario de execucao: ");
    Serial.println(rtc.getTimeStr());
  }
  addcommandtobuffer(READ_DHT);
  addcommandtobuffer(READ_BMP180);
}

void XBee_ReadPacket()
{
  
}

