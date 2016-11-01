unsigned long OS_READ_INTERVAL = 30000; //tempo padrão de leitura de todos o equipamentos.

struct datetimevar {
  char *hora;
  char *data;
  char *dia;
};
struct temp_press {
  char *temperatura;
  char *humidade;
};

temp_press *dhtresult;
datetimevar *dtime;
double pressao;
int command = 0;
double baseline; // baseline pressure
int commandbuffer[16];
unsigned long elaptime;


