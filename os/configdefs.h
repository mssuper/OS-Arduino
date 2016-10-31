struct configmodel {
  int OS_READ_INTERVAL = 30; // intervalo de Leitura dos dispositivos
};
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
 configmodel *cfg;
 datetimevar *dtime;
double pressao;
int command = 0;
double baseline; // baseline pressure
int commandbuffer[10];


