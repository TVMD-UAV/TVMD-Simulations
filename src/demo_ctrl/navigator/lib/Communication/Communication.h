#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "WiFi.h"

/* SSID & Password */
#define WIFI_SSID "ESP32"
#define WIFI_PASS "12345678"

class Communication
{
public:
    Communication();
    void init();

protected:
    /* IP Address details */
    const IPAddress local_ip;
    const IPAddress gateway;
    const IPAddress subnet;
};
#endif