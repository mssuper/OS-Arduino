

void init_rtc()
{
  //Aciona o relogio
  rtc.halt(false);
  //Definicoes do pino SQW/Out
  rtc.setSQWRate(SQW_RATE_1);
  rtc.enableSQW(true);

}

void init_bmp180()
{
  pressure.begin();
  baseline = getPressure();
}

void init_serial() {
  Serial.begin(9600);
}

void init_dht()
{
    dht.begin();
}
void init_variables()
{
  int command = 0;
  configmodel *cfg = new configmodel;
  datetimevar *dtime = get_time();
  temp_press *dhtresult;
  int pressao=0;
}

