#define DEBUG_ON true // se true exibe mensagens de debug pela serial se false não
unsigned int OS_READ_INTERVAL; //tempo padrão de leitura de todos o equipamentos este valor não pode ser menor 2 min ***atenção***.
char         OS_LOCATION[45];
int          OS_READ_INTERVAL_TYPE;
/*                                      0 Executa as leituras no momento que ativado
                                        1 Executa as leituras a cada quarto cheio 15 em 15 min
                                        2 Executa as leitura na hora cheia
*/
double       OS_PLUVIOMETER_VOLUME;
int          OS_PLUVIOMETER_READ_INTERVAL;
char OS_GPRS_ISP[20];
char   OS_GPRS_APN[50];
char OS_GPRS_USERNAME[25];
char OS_GPRS_PASSWORD[15];
char OS_FTP_SERVER[20];
char OS_FTP_USERNAME[25];
char OS_FTP_PASSWORD[20];
char OS_SMPT_SERVER[50];
char OS_SMTP_USER[25];
char OS_SMTP_PASSWORD[15];
double OS_OTT_FACTOR;

char ConfigBuffer[256]; 

struct datetimevar {
  char hora[10];
  char data[10];

};
struct temp_press {
  char temperatura[10];
  char humidade[10];
};
struct TempPBar {
  char PBar[10];
  char Temp[10];
};

double Profundidade;
static int commandbuffer[64];
temp_press dhtresult[1];
datetimevar regtime[1];
TempPBar resultPBar[1];
float PluviometerCouter;
int timerminuto;
bool relay1state=false;
int relay1time=100;
bool relay2state=false;
int relay2time=100;
bool relay3state=false;
int relay3time=100;
bool relay4state=false;
int relay4time=100;
int LastExecutionMinute;




