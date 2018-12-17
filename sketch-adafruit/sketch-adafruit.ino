#include "Sandroide.h"

void setup() {
  // your code here ..
  setupSandroide(true);
  // .. or here
}

long lastsent=0;
void loop() {
  // your code here ..
  loopSandroide();
  // .. or here

  // example of custom message: send message every five seconds, contaning elapsed time in seconds
  if ((millis()-lastsent)>5000) {
    lastsent = millis();
    uint8_t buf[20];
    int len = sprintf((char *)buf,"W%d",lastsent/1000);
    sendCustomMessage(buf, len);
  }
}
