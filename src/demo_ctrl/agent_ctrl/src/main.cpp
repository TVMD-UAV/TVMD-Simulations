#include <Arduino.h>
#include <IsrLed.h>
#include <Actuators.h>

IsrLed leds = IsrLed();
Actuators actuator = Actuators();

void demo_program();

void setup() {
    // put your setup code here, to run once:
    leds.init();
    actuator.init();
    Serial.begin(9600);
}

void loop() {
    // put your main code here, to run repeatedly:
    static auto last_update = millis();
    if (millis() - last_update > 10){
        last_update = millis();
        demo_program();
    }
    actuator.update();
}

void demo_program(){
    int color = 360 * (double)millis()/1000 / 3.1415 / 2;
    leds.set_hsl(color % 360, 1, 0.5);

    float mx = 200*(sin((float)millis()/1000)+1);
    float my = 200*(cos((float)millis()/1000)+1);

    float sx = 10*(sin((float)millis()/1000));
    float sy = 10*(cos((float)millis()/1000));

    actuator.set_angles(sx, sy);
    actuator.set_thrusts(mx, my);
}