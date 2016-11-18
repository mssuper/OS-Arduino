#define DEBUG_ON true // se true exibe mensagens de debug pela serial se false nÃ£o
unsigned int OS_READ_INTERVAL = 30; //tempo padrÃ£o de leitura de todos o equipamentos.
char OS_LOCATION[] = "Local Default";

struct datetimevar {
  char *hora;
  char *data;
  char *dia;
};
struct temp_press {
  char *temperatura;
  char *humidade;
};
struct TempPBar {
  char *PBar;
  char *Temp;
};

static int commandbuffer[16];
temp_press *dhtresult;
datetimevar *dtime;
TempPBar *resultPBar; 






