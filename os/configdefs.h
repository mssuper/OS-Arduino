#define DEBUG_ON true // se true exibe mensagens de debug pela serial se false não
unsigned int OS_READ_INTERVAL = 30; //tempo padrão de leitura de todos o equipamentos.
char         OS_LOCATION[] = "Local Default";
int          OS_READ_INTERVAL_TYPE  = 0;
/*                                      0 Executa as leituras no momento que ativado
                                        1 Executa as leituras a cada quarto cheio 15 em 15 min
                                        2 Executa as leitura na hora cheia
*/
double       OS_PLUVIOMETER_VOLUME = 0.2;
int          OS_PLUVIOMETER_READ_INTERVAL = 1000;

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

static int commandbuffer[16];
temp_press dhtresult[1];
datetimevar regtime[1];
TempPBar resultPBar[1];
float PluviometerCouter;





