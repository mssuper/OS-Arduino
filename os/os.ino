#include <Printers.h>
#include <XBee.h>
#include <SD.h>
#include <SPI.h>
#include <Time.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <DHT.h>
#include <DS1307.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <SoftwareSerial.h>
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
#define SD_PINCS 53
#define FILENAME "datalogger.log"

// Define NewSoftSerial TX/RX pins
// Connect Arduino pin 8 to TX of usb-serial device
#define  SSRX 8
// Connect Arduino pin 9 to RX of usb-serial device
#define  SSTX 9
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
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
ZBRxResponse rx = ZBRxResponse();
ZBRxIoSampleResponse ioSample = ZBRxIoSampleResponse();


/*! Inicialisa a Classe SoftwareSerial para comunicação com o XBee
  /*! Inicialisa a classe XBee para operação do rádio
*/
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


  /******************************************inicializa a MicroSD***********************************************/
  pinMode(SD_PINCS, OUTPUT);
  // SD Card Initialization
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    while (1);
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
  Serial2.begin(9600);
  xbee.setSerial(Serial2);
  // I think this is the only line actually left over
  // from Andrew's original example
  Serial.println("Xbee Iniciado!");


  /******************************************inicializa o XBee Serial***********************************************/








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
  XBee_ReadPacket();
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
  char buf[100];
  memset(buf, 0, sizeof(buf));
  int buf_index = 0;
  if (Serial2.available()) {


    // Read single char
    char single_character = Serial2.read();
    if (single_character > 0 )
    {
      if (single_character == 13)
      {
        // Ignore
      }
      else if (single_character == 10)
      {
        // We're done. At a 0 to the array to signal the end of string.

        buf[buf_index] = 0;
        buf_index = 0; // Reset index (so we can call it twice)
        Serial.println(single_character); // We're done

      }
      else
      {
        // Collect the characters together
        buf[buf_index] = single_character;
        buf_index++;
        Serial.println((int)single_character); // We're done

      }
    }
  }
}
void transmit_packet()
{
  // doing the read without a timer makes it non-blocking, so
  // you can do other stuff in loop() as well.
  xbee.readPacket();
  // so the read above will set the available up to
  // work when you check it.
  if (xbee.getResponse().isAvailable()) {
    // got something
    // I commented out the printing of the entire frame, but
    // left the code in place in case you want to see it for
    // debugging or something.  The actual code is down below.
    //showFrameData();
    Serial.print("Frame Type is ");
    // Andrew calls the frame type ApiId, it's the first byte
    // of the frame specific data in the packet.
    Serial.println(xbee.getResponse().getApiId(), HEX);

    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      // got a zb rx packet, the kind this code is looking for

      // now that you know it's a receive packet
      // fill in the values
      xbee.getResponse().getZBRxResponse(rx);

      // this is how you get the 64 bit address out of
      // the incoming packet so you know which device
      // it came from
      Serial.print("Got an rx packet from: ");
      XBeeAddress64 senderLongAddress = rx.getRemoteAddress64();
      print32Bits(senderLongAddress.getMsb());
      Serial.print(" ");
      print32Bits(senderLongAddress.getLsb());

      // this is how to get the sender's
      // 16 bit address and show it
      uint16_t senderShortAddress = rx.getRemoteAddress16();
      Serial.print(" (");
      print16Bits(senderShortAddress);
      Serial.println(")");

      // The option byte is a bit field
      if (rx.getOption() & ZB_PACKET_ACKNOWLEDGED)
        // the sender got an ACK
        Serial.println("packet acknowledged");
      if (rx.getOption() & ZB_BROADCAST_PACKET)
        // This was a broadcast packet
        Serial.println("broadcast Packet");

      Serial.print("checksum is ");
      Serial.println(rx.getChecksum(), HEX);

      // this is the packet length
      Serial.print("packet length is ");
      Serial.print(rx.getPacketLength(), DEC);

      // this is the payload length, probably
      // what you actually want to use
      Serial.print(", data payload length is ");
      Serial.println(rx.getDataLength(), DEC);

      // this is the actual data you sent
      Serial.println("Received Data: ");
      for (int i = 0; i < rx.getDataLength(); i++) {
        print8Bits(rx.getData()[i]);
        Serial.print(' ');
      }

      // and an ascii representation for those of us
      // that send text through the XBee
      Serial.println();
      for (int i = 0; i < rx.getDataLength(); i++) {
        Serial.write(' ');
        if (iscntrl(rx.getData()[i]))
          Serial.write(' ');
        else
          Serial.write(rx.getData()[i]);
        Serial.write(' ');
      }
      Serial.println();
      // So, for example, you could do something like this:
      handleXbeeRxMessage(rx.getData(), rx.getDataLength());
      Serial.println();
    }
    else if (xbee.getResponse().getApiId() == ZB_IO_SAMPLE_RESPONSE) {
      xbee.getResponse().getZBRxIoSampleResponse(ioSample);
      Serial.print("Received I/O Sample from: ");
      // this is how you get the 64 bit address out of
      // the incoming packet so you know which device
      // it came from
      XBeeAddress64 senderLongAddress = ioSample.getRemoteAddress64();
      print32Bits(senderLongAddress.getMsb());
      Serial.print(" ");
      print32Bits(senderLongAddress.getLsb());

      // this is how to get the sender's
      // 16 bit address and show it
      // However, end devices that have sleep enabled
      // will change this value each time they wake up.
      uint16_t senderShortAddress = ioSample.getRemoteAddress16();
      Serial.print(" (");
      print16Bits(senderShortAddress);
      Serial.println(")");
      // Now, we have to deal with the data pins on the
      // remote XBee
      if (ioSample.containsAnalog()) {
        Serial.println("Sample contains analog data");
        // the bitmask shows which XBee pins are returning
        // analog data (see XBee documentation for description)
        uint8_t bitmask = ioSample.getAnalogMask();
        for (uint8_t x = 0; x < 8; x++) {
          if ((bitmask & (1 << x)) != 0) {
            Serial.print("position ");
            Serial.print(x, DEC);
            Serial.print(" value: ");
            Serial.print(ioSample.getAnalog(x));
            Serial.println();
          }
        }
      }
      // Now, we'll deal with the digital pins
      if (ioSample.containsDigital()) {
        Serial.println("Sample contains digtal data");
        // this bitmask is longer (16 bits) and you have to
        // retrieve it as Msb, Lsb and assemble it to get the
        // relevant pins.
        uint16_t bitmask = ioSample.getDigitalMaskMsb();
        bitmask <<= 8;  //shift the Msb into the proper position
        // and in the Lsb to give a 16 bit mask of pins
        // (once again see the Digi documentation for definition
        bitmask |= ioSample.getDigitalMaskLsb();
        // this loop is just like the one above, but covers all
        // 16 bits of the digital mask word.  Remember though,
        // not all the positions correspond to a pin on the XBee
        for (uint8_t x = 0; x < 16; x++) {
          if ((bitmask & (1 << x)) != 0) {
            Serial.print("position ");
            Serial.print(x, DEC);
            Serial.print(" value: ");
            // isDigitalOn takes values from 0-15
            // and returns an On-Off (high-low).
            Serial.print(ioSample.isDigitalOn(x), DEC);
            Serial.println();
          }
        }
      }
      Serial.println();
    }

    else {
      Serial.print("Got frame id: ");
      Serial.println(xbee.getResponse().getApiId(), HEX);
    }
  }
  else if (xbee.getResponse().isError()) {
    // some kind of error happened, I put the stars in so
    // it could easily be found
    Serial.print("************************************* error code:");
    Serial.println(xbee.getResponse().getErrorCode(), DEC);
  }
  else {
    // I hate else statements that don't have some kind
    // ending.  This is where you handle other things
  }
}

void handleXbeeRxMessage(uint8_t *data, uint8_t length) {
  // this is just a stub to show how to get the data,
  // and is where you put your code to do something with
  // it.
  for (int i = 0; i < length; i++) {
    //    Serial.print(data[i]);
  }
  //  Serial.println();
}

void showFrameData() {
  Serial.println("Incoming frame data:");
  for (int i = 0; i < xbee.getResponse().getFrameDataLength(); i++) {
    print8Bits(xbee.getResponse().getFrameData()[i]);
    Serial.print(' ');
  }
  Serial.println();
  for (int i = 0; i < xbee.getResponse().getFrameDataLength(); i++) {
    Serial.write(' ');
    if (iscntrl(xbee.getResponse().getFrameData()[i]))
      Serial.write(' ');
    else
      Serial.write(xbee.getResponse().getFrameData()[i]);
    Serial.write(' ');
  }
  Serial.println();
}

// these routines are just to print the data with
// leading zeros and allow formatting such that it
// will be easy to read.
void print32Bits(uint32_t dw) {
  print16Bits(dw >> 16);
  print16Bits(dw & 0xFFFF);
}

void print16Bits(uint16_t w) {
  print8Bits(w >> 8);
  print8Bits(w & 0x00FF);
}

void print8Bits(byte c) {
  uint8_t nibble = (c >> 4);
  if (nibble <= 9)
    Serial.write(nibble + 0x30);
  else
    Serial.write(nibble + 0x37);

  nibble = (uint8_t) (c & 0x0F);
  if (nibble <= 9)
    Serial.write(nibble + 0x30);
  else
    Serial.write(nibble + 0x37);
}

void writefile(char *buffer)
{
  File DataLogger;

  DataLogger = SD.open(FILENAME, FILE_WRITE);
  if (DataLogger) {
    Serial.println("Escrevendo arquivo...");

    DataLogger.println(buffer);
    DataLogger.close(); // close the file
    Serial.println("Feito.");
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("erro ao abrir " );
  }
}

char * readfile()
{
  File DataLogger;
    // Reading the file
  DataLogger = SD.open(FILENAME);
  if (DataLogger) {
    Serial.println("lendo:");
    // Reading the whole file
    while (DataLogger.available()) {
      Serial.write(DataLogger.read());
   }
    DataLogger.close();
  }
  else {
    Serial.println("erro ao abrir ");
  }
  
}

