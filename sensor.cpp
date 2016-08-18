//#include "sensor.h"

/*
// set sensor indicator (on PIO_A)
void Sensor::setSensorIndicator(device_t d, uint8_t indval) {
  #ifdef SERIAL_DEBUGXX
  Serial.print("setting indicator:");
  Serial.println(indval);
  #endif
  //#define PIO_A 0
  //#define PIO_B 1
  //d.port is the sensor read port
  //indicator port is the other one ??
  uint8_t indicator_port = d.port ? 0 : 1;
  #ifdef SERIAL_DEBUGXX
    Serial.print("setSensorIndicator:prot=");
    Serial.print(d.port);
    Serial.print(" indicator port=");
    Serial.println(indicator_port);
  #endif
  p_ow->writePIOtest(d.dev_rom, indicator_port, indval);  //WRITE TO PIO-x
}


float Sensor::getLastTemperature() {
  return last_temp;
}

char* Sensor::romFormat(char *buf, uint8_t rom[]) {
  char *bp = buf;
  int n = 0;
  for(int i=0;i<8;i++) {
    bp = &buf[n];
    int rb = rom[i];
    if(rb<16) *bp++='0';
    itoa((int)rb, bp, 16);
    n+=2;
    buf[n++]='-';
  }
  buf[n-1]=0;
  return buf;
}
*/
