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
    commander.init();
}

void loop() {
    /*static auto last_update_time = millis();
    if (millis() - last_update_time > 50){
        website.update();
        last_update_time = millis();
    }*/
    static bool led_state = true;
    UartPacket_t up;

    uint8_t color = 254 * (double)millis()/1000 / 3.1415 / 2;
    uint8_t mx = 200*(sin((float)millis()/1000)+1);
    uint8_t my = 200*(cos((float)millis()/1000)+1);

    uint8_t sx = 20*(sin((float)millis()/1000)) + 127;
    uint8_t sy = 20*(cos((float)millis()/1000)) + 127;

    up.data = {
        .id = 0,
        .upper_motor = mx,
        .lower_motor = my,
        .center_servo = sy,
        .outer_servo = sx,
        .h = color,
        .s = 254,
        .l = 127//(led_state ? (uint8_t)127 : (uint8_t)0)
    };
    commander.send_single_commands(&up);
    led_state = !led_state;
    delay(20);
}