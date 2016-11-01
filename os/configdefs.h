unsigned int OS_READ_INTERVAL = 30; //tempo padrão de leitura de todos o equipamentos.

struct datetimevar {
  char *hora;
  char *data;
  char *dia;
};
struct temp_press {
  char *temperatura;
  char *humidade;
};

int *commandbuffer= new int[16];
temp_press *dhtresult;
datetimevar *dtime;
double pressao;






