struct configmodel {
  int OS_READ_INTERVAL = 30; // intervalo de Leitura dos dispositivos
};
struct datetimevar {
  char *hora;
  char *data;
  char *dia;
};
extern configmodel *cfg;
extern datetimevar *dtime;
int command = 0;
double baseline; // baseline pressure

