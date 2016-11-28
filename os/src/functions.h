void TimerManager();
//! Fun��o utilizada para executar rotinas a cada 24hs
/*! Esta fun��o esta na chamada da classe Alarm
  \author Marcelo Silveira.
  \since 01/10/2016
  \version 1.0.0
*/
void MorningAlarm();
//! Fun��o utilizada para contar o tempo de utiliza��o do sistema
/*! Esta fun��o apenas incrementa uma variavel minuto a minuto para futura convers�o em horas
  \author Marcelo Silveira.
  \since 24/11/2016
  \version 1.0.0
*/
void ElapsedCounter();

void CallPluvcouter();

void PluvControl(unsigned int CMD);
//! Fun��o utilizada para buscar a press�o e temperatura do BMP180
/*! Esta fun��o se utiliza da classe SFE_BMP180 para buscar dados no sensor e traduzi-lo para hPa
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
void getPressure();

//! Fun��o gettemphum()
/*! Esta fun��o se utiliza da classe DHT para buscar dados no sensor e traduz�-lo para ºC e % de URA
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
void gettemphum();

//! Fun��o getcommandfrombuffer()
/*! Fun��o utiliza a variavel global commandbuffer para o empilhamento de comandos este s�o retornados e uma cascata � feita
  ! para colocar o proximo comando para execu��o
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
int getcommandfrombuffer();

//! Fun��o addcommandtobuffer()
/*! Fun��o utiliza a variavel global commandbuffer para o empilhamento de comandos, o comando � recebido e empilhado na
  !posi��o mais alta disponivel
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
void addcommandtobuffer(const int command); //Fun��o para adi��o de de comando na pilha "commandbuffer[64]"
//! Fun��o scheduler()
/*! Fun��o utilizada para executar rotinas internas e outras se necess�rio for
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/
void scheduler();
//! Fun��o writedata()
/*! Fun��o utilizada para executar rotinas internas e outras se necess�rio for
  \author Marcelo Silveira.
  \since 30/09/2016
  \version 1.0.0
*/

void writedata();

void configmodule();

void ReadConfigModule();
void ReadOTT();
