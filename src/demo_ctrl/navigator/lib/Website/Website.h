#ifndef WEBSITE_H
#define WEBSITE_H

#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include "config.h"

class Website{
    public:
    Website();
    void init();

    void spiffs_init();
    void routing();

    protected:
    AsyncWebServer server;
};

#endif