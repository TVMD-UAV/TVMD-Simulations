#include <Arduino.h>
#include "Communication.h"
#include "Website.h"

Communication comm = Communication();
Website website = Website();

void setup() {
  // put your setup code here, to run once:
  comm.init();
  website.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println(millis());
  // delay(100);
}