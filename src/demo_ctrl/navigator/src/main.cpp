#include <Arduino.h>
#include "Communication.h"
#include "Website.h"

#include "Commander.h"
#include "IdleController.h"

Communication comm = Communication();
Website website = Website();

Commander commander = Commander();

void setup() {
    // put your setup code here, to run once:
    comm.init();
    website.init();
}

void loop() {
    static auto last_update_time = millis();
    if (millis() - last_update_time > 50){
        website.update();
        last_update_time = millis();
    }
}