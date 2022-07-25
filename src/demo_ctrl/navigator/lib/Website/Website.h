#ifndef WEBSITE_H
#define WEBSITE_H

#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "config.h"

#include "Commander.h"
#include "Controller.h"
#include "IdleController.h"

class Website{
    public:

    enum Modes {None=0, Idle, Demo, Joystick, Fixwing};

    Website();
    void init();

    void spiffs_init();
    void routing();

    void update();

    protected:
    static Modes pre_mode;
    AsyncWebServer server;
    
    static Controller* _ctrl;
    static Commander _commander;
};

#endif