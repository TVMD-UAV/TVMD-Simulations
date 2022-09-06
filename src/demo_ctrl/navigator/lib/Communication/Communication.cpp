#include "Communication.h"

Communication::Communication()
    : local_ip(192, 168, 1, 1),
      gateway(192, 168, 1, 1),
      subnet(255, 255, 255, 0)
{
}


void Communication::init()
{
    // Initialize debug Serial
    Serial.begin(115200);

    // Initialize Wi-Fi AP
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    delay(100);
    Serial.println("Wifi AP Enabled.");
}