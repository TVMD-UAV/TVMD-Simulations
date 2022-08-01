#ifndef WEBSITE_H
#define WEBSITE_H

#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "config.h"

#include "Commander.h"
#include "Controller.h"
#include "DemoController.h"
#include "IdleController.h"
#include "JoystickController.h"


class Website
{
public:
    enum Modes { None = 0, Idle, Demo, Joystick, Fixwing };
    enum ControllerState { Normal = 0, Ready };

    Website();
    void init();

    void spiffs_init();
    void routing();

    void update();

protected:
    static Modes pre_mode;
    AsyncWebServer server;

    static ControllerState controller_state;
    static Controller *_ctrl_ready;
    static Controller *_ctrl;
    static Commander _commander;
};

#endif