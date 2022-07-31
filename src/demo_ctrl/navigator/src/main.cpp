#include <Arduino.h>
#include "Communication.h"
#include "Website.h"

#include "Commander.h"
#include "DemoController.h"
#include "IdleController.h"


Communication comm = Communication();
Website website = Website();

Commander commander = Commander();

Controller *ctrl = new DemoController(&commander);

void setup()
{
    // put your setup code here, to run once:
    comm.init();
    website.init();
    commander.init();
}

void loop()
{
    static auto last_update_time = millis();
    if (millis() - last_update_time > 50) {
        last_update_time = millis();
    }

    website.update();
}